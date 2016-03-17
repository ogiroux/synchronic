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

#define _WIN32_WINNT 0x0601

#include "test.hpp"

#include <string>
#include <atomic>
#include <random>
#include <chrono>
#include <iostream>
#include <iomanip>


#ifdef __linux__
    #include <unistd.h>
    #include <sys/times.h>
    typedef tms cpu_time;
    cpu_time get_cpu_time() { 
        cpu_time t; 
        times(&t); 
        return t; 
    }
    std::chrono::nanoseconds cpu_time_consumed(cpu_time start, cpu_time end) {
        auto nanoseconds_per_clock_tick = 1000000000 / sysconf(_SC_CLK_TCK);
        auto clock_ticks_elapsed = (end.tms_utime - start.tms_utime) + (end.tms_stime - start.tms_stime);
        return std::chrono::nanoseconds(clock_ticks_elapsed * nanoseconds_per_clock_tick);
    }
#endif

#ifdef _MSC_VER
    static HANDLE self = GetCurrentProcess();
    typedef std::pair<FILETIME,FILETIME> cpu_time;
    cpu_time get_cpu_time() {
        cpu_time t;
        FILETIME ftime, fsys, fuser;
        GetProcessTimes(self, &ftime, &ftime, &fsys, &fuser);
        memcpy(&t.first, &fsys, sizeof(FILETIME));
        memcpy(&t.second, &fuser, sizeof(FILETIME));
        return t;
    }
    std::uint64_t make64(std::uint64_t low, std::uint64_t high) {
        return low | (high << 32);
    }
    std::uint64_t make64(FILETIME ftime) {
        return make64(ftime.dwLowDateTime, ftime.dwHighDateTime);
    }
    std::chrono::nanoseconds cpu_time_consumed(cpu_time start, cpu_time end) {

        auto nanoseconds_per_clock_tick = 100; //100-nanosecond intervals
        auto clock_ticks_elapsed = (make64(end.first) - make64(start.first)) + (make64(end.second) - make64(start.second));
        return std::chrono::nanoseconds(clock_ticks_elapsed * nanoseconds_per_clock_tick);
    }
#endif

using my_clock = std::conditional<std::chrono::high_resolution_clock::is_steady,
    std::chrono::high_resolution_clock, std::chrono::steady_clock>::type;

static constexpr int default_count = 1 << 28;

template <class R>
std::chrono::nanoseconds compute_work_item_cost(R r)  {

    auto start = my_clock::now();

    //perform work
    for (int i = 0; i < default_count; ++i)
        r.discard(1);

    auto end = my_clock::now();
       
    return std::chrono::nanoseconds(end - start) / default_count;
}

class run {
    std::atomic<bool> go = false, stop = false;
    std::atomic<std::uint64_t> running = 0, iterations = 0;
public :
    struct report {
        std::chrono::nanoseconds wall_time, cpu_time;
        std::uint64_t steps;
    };
    template <class F>
    report time(int threads, F f) {
        for (int i = 0; i < threads; ++i)
            std::thread([&,f, i]() mutable {
                running++;
                while (go != true) std::this_thread::yield();
                while (stop != true) {
                    f(i);
                    iterations.fetch_add(1, std::memory_order_relaxed);
                }
                running--;
            }).detach();
        while (running != threads) std::this_thread::yield();
        auto cpu_start = get_cpu_time();
        auto start = my_clock::now();
        go = true;
        std::uint64_t it1 = iterations;
        if (threads)
            std::this_thread::sleep_for(std::chrono::seconds(5));
        else {
            for (int i = 0; i < default_count; ++i) {
                f(0);
                iterations.fetch_add(1, std::memory_order_relaxed);
            }
        }
        std::uint64_t it2 = iterations;
        auto cpu_end = get_cpu_time();
        auto end = my_clock::now();
        stop = true;
        while (running != 0) std::this_thread::yield();
        std::this_thread::sleep_for(std::chrono::seconds(1));

        report r;
        r.wall_time = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
        r.cpu_time = cpu_time_consumed(cpu_start, cpu_end);
        r.steps = (it2 - it1);

        return r;
    }
};

template <class F>
void do_run(std::string const& what, int threads, F f, std::chrono::nanoseconds expected)
{
    std::cout << "measuring " << what << "...\r";
    auto r = run().time(threads, f);
    std::cout << what << " (" << threads << "T)" << std::endl;
    std::cout << "\ttotal progress : " << r.steps << " steps in " << r.wall_time.count() << "ns" << std::endl;
    std::cout << "\toverheads on " << expected.count() << " ns/step : " << std::endl;
    auto wall_time = (r.wall_time / r.steps - expected).count();
    std::cout << "\t\t" << std::left << std::setfill(' ') << std::setw(14) << "wall-time = " << wall_time << " ns/step (" << double(wall_time) / expected.count() * 100 << "%)" << std::endl;
    auto cpu_time = (r.cpu_time / r.steps - expected).count();
    std::cout << "\t\t" << std::left << std::setfill(' ') << std::setw(14) << "cpu-time =" << cpu_time << " ns/step (" << double(cpu_time) / expected.count()*100 << "%)" << std::endl;
    std::cout << std::endl;
}

int main(int argc, const char * argv[]) {

    std::mt19937 r;

    std::cout << "measuring work item cost...\r";
    auto cost = compute_work_item_cost(r);
    std::cout << "work item cost : " << cost.count() << " nanoseconds per iteration\n";
    std::cout << std::endl;

    do_run("control 0-thread", 1, [=](auto) mutable {
        r.discard(1); }, cost);
    do_run("control 1-thread", 1, [=](auto) mutable { 
        r.discard(1); }, cost);

    std::mutex m1;
    do_run("std::mutex uncontended 1-thread", 1, [=,&m1](auto) mutable {
        { std::unique_lock<std::mutex>(m1); }
        r.discard(1); 
    }, cost);
    ttas_mutex m2;
    do_run("ttas_mutex uncontended 1-thread", 1, [=,&m2](auto) mutable {
        { std::unique_lock<ttas_mutex>(m2); }
        r.discard(1); 
    }, cost);

    auto const N = std::thread::hardware_concurrency();
    constexpr int k_limit = 1024;
    if (N > k_limit) {
        std::cout << "ERROR: Cannot continue because there are more than " << k_limit << " hardware threads on this system.\n";
        std::cout << "ERROR: This is an arbitrary limit. Feel free to raise it, recompile and rerun.\n";
    }

    do_run("control N-thread", N, [=](auto i) mutable {
        r.discard(1); 
    }, cost);
    std::mutex m1N[k_limit];
    do_run("std::mutex uncontended N-thread", N, [=,&m1N](auto i) mutable {
        auto& m = m1N[i];
        { std::unique_lock<std::mutex> l(m); }
        r.discard(1); 
    }, cost);
    ttas_mutex m2N[k_limit];
    do_run("ttas_mutex uncontended N-thread", N, [=,&m2N](auto i) mutable {
        auto& m = m2N[i];
        { std::unique_lock<ttas_mutex> l(m); }
        r.discard(1);
    }, cost);

    do_run("std::mutex low-p contended N-thread", N, [=,&m1N](auto i) mutable {
        auto& m = m1N[r() & 0x7];
        { std::unique_lock<std::mutex> l(m); }
    }, cost);
    do_run("ttas_mutex low-p contended N-thread", N, [=, &m2N](auto i) mutable {
        auto& m = m2N[r() & 0x7];
        { std::unique_lock<ttas_mutex> l(m); }
    }, cost);

    auto short_time = std::chrono::nanoseconds(300);
    auto short_count = short_time / cost;
    do_run("std::mutex short contended N-thread", N, [=, &m1](auto) mutable {
        std::unique_lock<std::mutex> l(m1);
        r.discard(short_count);
    }, short_time);
    do_run("ttas_mutex short contended N-thread", N, [=, &m2](auto) mutable {
        std::unique_lock<ttas_mutex> l(m2); 
        r.discard(short_count);
    }, short_time);

    auto long_time = std::chrono::milliseconds(10);
    auto long_count = long_time / cost;
    do_run("std::mutex long contended N-thread", N, [=, &m1](auto) mutable {
        std::unique_lock<std::mutex> l(m1);
        r.discard(long_count);
    }, long_time);
    do_run("ttas_mutex long contended N-thread", N, [=, &m2](auto) mutable {
        std::unique_lock<ttas_mutex> l(m2);
        r.discard(long_count);
    }, long_time);

    return 0;
}
