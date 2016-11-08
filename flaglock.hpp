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

#include <atomic>
#include <chrono>
#include <thread>
#include <cassert>

#ifdef WIN32
#include <windows.h>
#endif //WIN32

#ifdef __linux__
#include <time.h>
#include <unistd.h>
#include <linux/futex.h>
#include <sys/syscall.h>
#include <sys/types.h>
#include <climits>
#endif //__linux__

namespace std {
	namespace experimental {
		inline namespace concurrency_v2 {

#ifdef __linux__
			inline void __atomic_yield() { sched_yield(); }
#else
			inline void __atomic_yield() { this_thread::yield(); }
#endif

#if defined(_MSC_VER)
	#if (defined(_M_X86) || defined(_M_X64))
		#define __atomic_x86
	#elif defined(_M_ARM)
		#define __atomic_arm
	#endif
#elif defined(__GNUC__) 
	#if (defined(__i386__) || defined(__x86_64__))
		#define __atomic_x86
	#elif defined(__arm__)
		#define __atomic_arm
	#endif
#endif

#ifdef __GNUC__
	#define __atomic_gcc_inline __attribute__((always_inline))
	#define __atomic_gcc_dont_inline __attribute__((noinline))
#else
	#define __atomic_gcc_inline inline
	#define __atomic_gcc_dont_inline
#endif

#if defined(__GNUC__)
	#define __atomic_expect __builtin_expect
#else
	#define __atomic_expect(c,e) (c)
#endif

#ifdef __linux__
			// On Linux, we make use of the kernel memory wait operations. These have been available for a long time.
			template < class Rep, class Period>
			timespec __atomic_to_timespec(chrono::duration<Rep, Period> const& delta) {
				struct timespec ts;
				ts.tv_sec = static_cast<long>(chrono::duration_cast<chrono::seconds>(delta).count());
				ts.tv_nsec = static_cast<long>(chrono::duration_cast<chrono::nanoseconds>(delta).count());
				return ts;
			}
			inline void __atomic_wait(const void* p, int v) {
				syscall(SYS_futex, p, FUTEX_WAIT_PRIVATE, v, 0, 0, 0);
			}
			template < class Rep, class Period>
			void __atomic_wait_timed(const void* p, int v, const chrono::duration<Rep, Period>& t) {
				syscall(SYS_futex, p, FUTEX_WAIT_PRIVATE, v, __atomic_to_timespec(t), 0, 0);
			}
			inline void __atomic_wake_one(const void* p) {
				syscall(SYS_futex, p, FUTEX_WAKE_PRIVATE, 1, 0, 0, 0);
			}
			inline void __atomic_wake_all(const void* p) {
				syscall(SYS_futex, p, FUTEX_WAKE_PRIVATE, INT_MAX, 0, 0, 0);
			}
	#define __atomic_flag_fast_path
#endif // __linux__

#if defined(WIN32) && _WIN32_WINNT >= 0x0602
			// On Windows, we make use of the kernel memory wait operations as well. These first became available with Windows 8.
			template <class V>
			void __atomic_wait(const void* p, V v) {
				WaitOnAddress((PVOID)p, (PVOID)&v, sizeof(v), -1);
			}
			template <class V, class Rep, class Period>
			void __atomic_wait_timed(const void* p, V v, chrono::duration<Rep, Period> const& delta) {
				WaitOnAddress((PVOID)p, (PVOID)&v, sizeof(v), chrono::duration_cast<chrono::milliseconds>(delta).count());
			}
			inline void __atomic_wake_one(const void* p) {
				WakeByAddressSingle((PVOID)p);
			}
			inline void __atomic_wake_all(const void* p) {
				WakeByAddressAll((PVOID)p);
			}
	#define __atomic_flag_fast_path
#endif // defined(WIN32) && _WIN32_WINNT >= 0x0602

//#undef __atomic_flag_fast_path

#if defined(__arm__) || defined(_MSC_VER) || !defined(__atomic_flag_fast_path)
			// A simple exponential back-off helper that is designed to cover the space between (1<<__magic_number_3) and __magic_number_4
			class __atomic_exponential_backoff {
				int microseconds = 50;
			public:
				void sleep(int us = 0) {
					if (us != 0)
						microseconds = us;
					this_thread::sleep_for(chrono::microseconds(microseconds));
					// Avoiding the use of std::min here, to keep includes minimal
					auto next_microseconds = microseconds + (microseconds >> 2);
					microseconds = next_microseconds < 2048 ? next_microseconds : 2048;
				}
			};
#endif

			enum class atomic_notify {
				all, one, none
			};

			struct atomic_flag {

				typedef uint32_t base_t;

				static constexpr base_t valubit = 1;
#ifdef __atomic_flag_fast_path
				static constexpr base_t contbit = 2;
				static constexpr base_t lockbit = 4;
#endif

				mutable std::atomic<base_t> atom;

				__atomic_gcc_inline bool test_and_set(memory_order order = memory_order_seq_cst, atomic_notify notify = atomic_notify::all) noexcept {
					base_t old = 0;
					bool const success = atom.compare_exchange_weak(old, valubit, order, memory_order_relaxed);
					bool retcode = (old & valubit) == 1;
#ifdef __atomic_flag_fast_path
					if(__atomic_expect(!success && !retcode,0))
						retcode = test_and_set_slow(old, order, notify);
#endif
#ifdef __atomic_arm
					if(!retcode) {
						__asm__ __volatile__(
							"   dsb\n"
							"   sev"
						);
					}
#endif
					return retcode;
				}

#ifdef __atomic_flag_fast_path
				__atomic_gcc_dont_inline bool test_and_set_slow(base_t old, memory_order order, atomic_notify notify) noexcept {
					while ((old & valubit) == 0) {
						old &= contbit;
						base_t const lock = (old & contbit ? lockbit : 0);
						if (atom.compare_exchange_weak(old, valubit | lock, order, memory_order_relaxed)) {
							if (lock) {
								switch (notify) {
								case atomic_notify::all: __atomic_wake_all(&atom); break;
								case atomic_notify::one: __atomic_wake_one(&atom); break;
								case atomic_notify::none: break;
								}
								atom.fetch_and(~lock, memory_order_relaxed);
							}
							return false;
						}
					}
					return true;
				}
#endif

			__atomic_gcc_inline void clear(memory_order order = memory_order_seq_cst, atomic_notify notify = atomic_notify::all) noexcept {
#ifdef __atomic_flag_fast_path
				base_t old = valubit;
				bool const success = atom.compare_exchange_weak(old, 0, order, memory_order_relaxed);
				if (__atomic_expect(!success, 0)) {
					bool const success2 = ((old & ~valubit) == 0) && atom.compare_exchange_weak(old, 0, order, memory_order_relaxed);
					if (__atomic_expect(!success2, 0))
						clear_slow(old, order, notify);
				}
#else
				atom.store(0, order);
#endif
#ifdef __atomic_arm
				__asm__ __volatile__(
					"   dsb\n"
					"   sev"
				);
#endif
			}

#ifdef __atomic_flag_fast_path
			__atomic_gcc_dont_inline void clear_slow(base_t old, memory_order order, atomic_notify notify) noexcept {
				while (1) {
					old &= (contbit | valubit);
					base_t const lock = (old & contbit) ? lockbit : 0;
					if (atom.compare_exchange_weak(old, lock, order, memory_order_relaxed)) {
						if (lock) {
							switch (notify) {
							case atomic_notify::all: __atomic_wake_all(&atom); break;
							case atomic_notify::one: __atomic_wake_one(&atom); break;
							case atomic_notify::none: break;
							}
							atom.fetch_and(~lock, memory_order_relaxed);
						}
						break;
					}
				}
			}
#endif

			__atomic_gcc_inline void wait(bool set, memory_order order = memory_order_seq_cst) const noexcept {

				base_t old = atom.load(order);
				base_t const expectbit = (set ? valubit : 0);
				if (__atomic_expect(old == expectbit, 1))
					return;
#ifdef __atomic_arm
                                if ((old & valubit) != expectbit)                                
                                    for (int i = 0; i < 4; ++i) {
                                        base_t const tmp = old;
                                        __asm__ __volatile__(
                                             "ldrex %0, [%1]\n"
                                             "cmp %0, %2\n"
                                             "it eq\n"
                                             "wfeeq.n\n"
                                             "nop.w\n"
                                             : "=&r" (old) : "r" (&atom), "r" (tmp) : "cc"
                                         );
                                         if ((old & valubit) == expectbit) { atomic_thread_fence(order); break; }
                                    }
#endif
				if (__atomic_expect(old != expectbit, 0))
					wait_slow(old, expectbit, order);

			}

			__atomic_gcc_dont_inline void wait_slow(base_t old, base_t expectbit, memory_order order) const noexcept {

                                if ((old & valubit) != expectbit)
                                    for (int i = 0; i < 32; ++i) {
                                        __atomic_yield();
                                        old = atom.load(order);
                                        if ((old & valubit) == expectbit) break;
                                    }
				if ((old & valubit) != expectbit) {
					__atomic_exponential_backoff b;
#ifdef __atomic_flag_fast_path
					for (int i = 0; i < 2; ++i) {
#else
                                        while(1) {
#endif
						b.sleep();
						old = atom.load(order);
						if ((old & valubit) == expectbit) break;
					}
				}
#ifdef __atomic_flag_fast_path
				if ((old & valubit) != expectbit) {
					while(1) {
						old = atom.fetch_or(contbit, memory_order_relaxed) | contbit;
						if ((old & valubit) == expectbit) break;
						__atomic_wait(&atom, old);
						old = atom.load(order);
						if ((old & valubit) == expectbit) break;
					}
				}
				while (old & lockbit)
					old = atom.load(memory_order_relaxed);
#endif
			}
			bool test(memory_order order = memory_order_seq_cst) noexcept {

				return atom.load(order) & valubit;
			}

			template <class Clock, class Duration>
			void wait_until(bool set, chrono::time_point<Clock, Duration> const& abs_time) const;

			template <class Rep, class Period>
			void wait_for(bool set, chrono::duration<Rep, Period> const& rel_time) const;
			//        using clock = conditional<chrono::high_resolution_clock::is_steady,
			//            chrono::high_resolution_clock, chrono::steady_clock>::type;
			/*
			__synchronic_exponential_backoff b;
			auto end = clock::now() + rel_time;
			do {
			b.sleep();
			if (object.load(order) != nval)
			return true;
			} while (clock::now() < end);
			*/

			bool test_and_set(memory_order order = memory_order_seq_cst, atomic_notify notify = atomic_notify::all) volatile noexcept;
			void wait(bool set, memory_order order = memory_order_seq_cst) const volatile;
			void clear(memory_order order = memory_order_seq_cst, atomic_notify notify = atomic_notify::all) volatile noexcept;
			bool test(memory_order order = memory_order_seq_cst) volatile noexcept;
			template <class Clock, class Duration>
			void wait_until(bool set, chrono::time_point<Clock, Duration> const& abs_time) const volatile;
			template <class Rep, class Period>
			void wait_for(bool set, chrono::duration<Rep, Period> const& rel_time) const volatile;

			atomic_flag(base_t init) noexcept : atom(init) { }
			atomic_flag() noexcept = default;
			atomic_flag(const atomic_flag&) = delete;
			atomic_flag& operator=(const atomic_flag&) = delete;
//			atomic_flag& operator=(const atomic_flag&) volatile = delete;
				};

		//#define ATOMIC_FLAG_INIT 0

			} // namespace concurrency_v2
		} // namespace experimental
	} // namespace std

	struct alignas(64) atomic_flag_lock {

		void lock() {

			while (__atomic_expect(f.test_and_set(std::memory_order_acquire), 0))
				//  ;                                       //this is the C++17 version
				f.wait(false, std::memory_order_relaxed);   //this is the C++20 version maybe!
		}

		void unlock() {

			f.clear(std::memory_order_release);
		}

	private:
		std::experimental::atomic_flag f = ATOMIC_FLAG_INIT;
	};
