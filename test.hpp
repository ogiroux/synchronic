/*

Copyright (c) 2014, NVIDIA Corporation
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef TEST_HPP
#define TEST_HPP

#include <synchronic>
#include <mutex>

template <bool truly>
struct dumb_mutex {

    dumb_mutex() : locked(false) {
    }

    dumb_mutex(const dumb_mutex&) = delete;
    dumb_mutex& operator=(const dumb_mutex&) = delete;

    void lock() {

        while (1) {
            bool state = false;
            if (locked.compare_exchange_weak(state, true, std::memory_order_acquire))
                return;
            while (locked.load(std::memory_order_relaxed))
                if (!truly)
                    std::this_thread::yield();
        };
    }

    void unlock() {

        locked.store(false, std::memory_order_release);
    }

private:
    std::atomic<bool> locked;
};

#ifdef WIN32
#include <windows.h>
#include <synchapi.h>
struct srw_mutex {

    srw_mutex() {
        InitializeSRWLock(&_lock);
    }

    void lock() {
        AcquireSRWLockExclusive(&_lock);
    }
    void unlock() {
        ReleaseSRWLockExclusive(&_lock);
    }

private:
    SRWLOCK _lock;
};
#endif

#if defined(__linux__) || (defined(WIN32) && _WIN32_WINNT >= 0x0602)

class simple_mutex
{
    std::atomic<int> word;
public:
    simple_mutex() : word(0) { }
    void lock()
    {
        // try to atimically swap 0 -> 1
        int value1 = 0;
        if (word.compare_exchange_strong(value1, 1, std::memory_order_acquire, std::memory_order_relaxed))
            return; // success
                    // wasn't zero -- somebody held the lock
        do {
            int value2 = 1;
            // assume lock is still taken, try to make it 2 and wait
            if (value1 == 2 || word.compare_exchange_strong(value2, 2, std::memory_order_acquire, std::memory_order_relaxed))
                // let's wait, but only if the value is still 2
                std::experimental::__synchronic_wait(&word, 2);
            // try (again) assuming the lock is free
            value1 = 0;
        } while (!word.compare_exchange_strong(value1, 2, std::memory_order_acquire, std::memory_order_relaxed));
        // we are here only if transition 0 -> 2 succeeded
    }
    void unlock() {
        if (word.fetch_add(-1, std::memory_order_release) != 1) {
            word.store(0, std::memory_order_release);
            std::experimental::__synchronic_wake_one(&word);
        }
    }
};

#endif

struct alignas(64) ttas_mutex {
    ttas_mutex() : locked(0) { }
    ttas_mutex(const ttas_mutex&) = delete;
    ttas_mutex& operator=(const ttas_mutex&) = delete;
    void lock() {
        while (1) {
            int state = 0;
            if (locked.compare_exchange_weak(state, 1, std::memory_order_acquire))
                return;
            if (state != 0)
                sync.wait_for_change(locked, state, std::memory_order_relaxed);
        }
    }
    void unlock() {
        sync.notify_one(locked, 0, std::memory_order_release);
    }
private:
    std::atomic<int> locked;
    std::experimental::synchronic<int> sync;
};

template <class T>
struct semaphoric {

    enum C : int {
        Uncontended = 0,
        Contended
    };

    struct payload {
        payload() : value(), state() { }
        payload(T t, C c) : value(t), state(c) { }
        bool operator==(payload const& p) const noexcept { return value == p.value && state == p.state; }
        bool operator!=(payload const& p) const noexcept { return value != p.value || state != p.state; }
        T value;
        C state;
    };

    std::atomic<payload> atom;
    std::experimental::synchronic<payload> sync;

    semaphoric() = default;
    constexpr semaphoric(T t) noexcept : atom(payload(t, Uncontended)) {
    }
    semaphoric(semaphoric const&) = delete;
    semaphoric& operator=(semaphoric const&) = delete;

    T operator=(T t) noexcept {
        store(t);
        return *this;
    }
    operator T() const noexcept {
        return load();
    }
    T load(std::memory_order o = std::memory_order_seq_cst) const noexcept {
        auto p = atom.load(o);
        return p.value;
    }
    void store(T t, std::memory_order o = std::memory_order_seq_cst) noexcept {
        sync.notify_all(atom, [t, o](std::atomic<payload>& a) {
            a.store(payload(t, Uncontended), o);
        });
    }
    T exchange(T t, std::memory_order o = std::memory_order_seq_cst) noexcept {
        sync.notify_all(atom, [&t, o](std::atomic<payload>& a) {
            t = a.exchange(payload(t, Uncontended), o);
        });
        return t;
    }
    bool compare_exchange_weak(T& t_, T t, std::memory_order o = std::memory_order_seq_cst) noexcept {
        bool b;
        payload p(t_, Uncontended);
        sync.notify_all(atom, [t, o, &b, &p](std::atomic<payload>& a) {
            b = a.compare_exchange_weak(p, payload(t, Uncontended), o);
        });
        t_ = p.value;
        return b;
    }
    bool compare_exchange_strong(T& t_, T t, std::memory_order o = std::memory_order_seq_cst) noexcept {
        bool b;
        payload p(t_, Uncontended);
        sync.notify_all(atom, [t, o, &b, &p](std::atomic<payload>& a) {
            b = a.compare_exchange_strong(p, payload(t, Uncontended), o);
        });
        t_ = p.value;
        return b;
    }
    void wait_for_change(T t, std::memory_order o = std::memory_order_seq_cst) const {
        payload p(t, Uncontended);
        sync.wait_for_change(atom, p, o);
    }
};

#if defined(__linux__) || (_WIN32_WINNT >= 0x0602)

template <class T>
struct semaphoric2 {

    struct payload {
        payload() : value(), contention_count() { }
        payload(T t, int count = 0) : value(t), contention_count(count) { }
        bool operator==(payload const& p) const noexcept { return value == p.value && contention_count == p.contention_count; }
        bool operator!=(payload const& p) const noexcept { return value != p.value || contention_count != p.contention_count; }
        T value;
        int contention_count;
    };

    mutable std::atomic<payload> atom;
    //mutable std::experimental::synchronic<payload, std::experimental::synchronic_type::optimized_for_long_wait> sync;

    semaphoric2() = default;
    constexpr semaphoric2(T t) noexcept : atom(payload(t, 0)) { }
    semaphoric2(semaphoric2 const&) = delete;
    semaphoric2& operator=(semaphoric2 const&) = delete;
    T operator=(T t) noexcept { store(t); return *this; }
    operator T() const noexcept { return load(); }

    T load(std::memory_order o = std::memory_order_seq_cst) const noexcept {

        return atom.load(o).value;
    }
    void store(T t, std::memory_order o = std::memory_order_seq_cst) noexcept {

        exchange(t, o);
    }
    T exchange(T t, std::memory_order o = std::memory_order_seq_cst) noexcept {

        T t_ = load(std::memory_order_relaxed);
        while (!compare_exchange_weak(t_, t, o))
            ;
        return t_;
    }
    bool compare_exchange_weak(T& t_, T t, std::memory_order o = std::memory_order_seq_cst) noexcept {

        return compare_exchange_strong(t_, t, o);
    }
    bool compare_exchange_strong(T& t_, T t, std::memory_order o = std::memory_order_seq_cst) noexcept {

        payload p(t_, 0);
        if (atom.compare_exchange_strong(p, payload(t, 0), o))
            return true;

        bool b = false;
        if (p.value == t_) {
            do {
                b = atom.compare_exchange_weak(p, payload(t, p.contention_count), o);
            } while (!b && p.value == t_);
            std::experimental::__synchronic_wake_all(&atom);
        }

        t_ = p.value;
        return b;
    }
    void wait_for_change(T t, std::memory_order o = std::memory_order_seq_cst) const {

        if (std::experimental::__synchronic_spin_for_change(*this, t, o))
            return;

        payload p = atom.load(std::memory_order_relaxed);
        //increment
        while (1) {
            if (p.value != t)
                return;
            if (atom.compare_exchange_weak(p, payload(p.value, p.contention_count + 1), o))
                break;
        }
        //wait
        while(1) { 
            p = atom.load(std::memory_order_relaxed);
            if (p.value != t)
                break;
            std::experimental::__synchronic_wait(&atom, p);
        }
        //decrement
        while (!atom.compare_exchange_weak(p, payload(p.value, p.contention_count - 1), o))
            ;
    }
};

struct alignas(64) ttas_mutex2 {
    ttas_mutex2() : locked(0) { }
    ttas_mutex2(const ttas_mutex2&) = delete;
    ttas_mutex2& operator=(const ttas_mutex2&) = delete;
    void lock() {
        while (1) {
            int state = 0;
            if (locked.compare_exchange_weak(state, 1, std::memory_order_acquire))
                return;
            if (state != 0)
                locked.wait_for_change(state, std::memory_order_relaxed);
        }
    }
    void unlock() {
        //locked.store(0, std::memory_order_release);
        int i = 1;
        locked.compare_exchange_strong(i, 0, std::memory_order_release);
    }
private:
    semaphoric2<int> locked;
};

#endif

//#define ttas_mutex ttas_mutex2

struct ticket_mutex {

    ticket_mutex() : active(0), queue(0) {
    }

    ticket_mutex(const ticket_mutex&) = delete;
    ticket_mutex& operator=(const ticket_mutex&) = delete;

    void lock() {

        int const me = queue.fetch_add(1, std::memory_order_relaxed);
        sync.wait(active, me, std::memory_order_acquire);
    }

    void unlock() {

        sync.notify_all(active, [](std::atomic<int>& atom) {
            atom.fetch_add(1, std::memory_order_release);
        });
    }
private:
    std::atomic<int> active, queue;
    std::experimental::synchronic<int> sync;
};

struct mcs_mutex {

    mcs_mutex() : head(nullptr) {
    }

    mcs_mutex(const mcs_mutex&) = delete;
    mcs_mutex& operator=(const mcs_mutex&) = delete;

    struct unique_lock {

        unique_lock(mcs_mutex & m) : m(m), next(nullptr), ready(false) {

            unique_lock * const head = m.head.exchange(this, std::memory_order_acquire);
            if (head != nullptr) {

                head->sync_next.notify_one(head->next, this);
                sync_ready.wait(ready, true);
            }
        }

        unique_lock(const unique_lock&) = delete;
        unique_lock& operator=(const unique_lock&) = delete;

        ~unique_lock() {

            unique_lock * head = this;
            if (!m.head.compare_exchange_strong(head, nullptr, std::memory_order_release, std::memory_order_relaxed)) {

                unique_lock * n = next.load(std::memory_order_acquire);
                if (n == nullptr) {

                    sync_next.wait_for_change(next, nullptr, std::memory_order_acquire);
                }
                n->sync_ready.notify_one(n->ready, true);
            }
        }

    private:
        mcs_mutex & m;
        std::atomic<unique_lock*> next;
        std::atomic<bool> ready;
        std::experimental::synchronic<unique_lock*> sync_next;
        std::experimental::synchronic<bool> sync_ready;
    };

private:
    std::atomic<unique_lock*> head;
};

namespace std {
    template<>
    struct unique_lock<mcs_mutex> : mcs_mutex::unique_lock {
        unique_lock(mcs_mutex & m) : mcs_mutex::unique_lock(m) {
        }
        unique_lock(const unique_lock&) = delete;
        unique_lock& operator=(const unique_lock&) = delete;
    };
}
/*
class webkit_mutex {
public:
    enum State : std::uint8_t {
        Unlocked,
        Blocked,
        Locked,
        LockedAndBlocked
    };

    webkit_mutex() : m_state(Unlocked) {
    }
    
    bool try_lock() {
        State s = Unlocked;
        if (__synchronic_expect(m_state.compare_exchange_strong(s, Locked, std::memory_order_acquire), 1))
            return true;
        if (s == Blocked &&
            __synchronic_expect(m_state.compare_exchange_strong(s, LockedAndBlocked, std::memory_order_acquire), 1))
            return true;
        return false;
    }

    void lock()
    {
        if(try_lock())
            return;
        lock_slow();
    }

    void lock_slow() {

        while (1) {
            State s = m_state.load(std::memory_order_relaxed);
            if (s == Unlocked && try_lock())
                return;
            if (s == LockedAndBlocked ||
                m_state.compare_exchange_strong(s, LockedAndBlocked, std::memory_order_relaxed) ||
                s == LockedAndBlocked)
                sync.wait(m_state, Unlocked, std::memory_order_relaxed);
        }
    }

    void unlock()
    {
        State s = Locked;
        if (__synchronic_expect(m_state.compare_exchange_strong(s, Unlocked, std::memory_order_release), 1))
            return;

        unlock_slow();
    }

    void unlock_slow() {

        sync.notify_all(m_state, Unlocked, std::memory_order_release);
    }

private:
    std::atomic<State> m_state;
    std::experimental::synchronic<State> sync;
};
*/
#endif //TEST_HPP
