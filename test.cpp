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


#if defined(__linux__) || defined(__APPLE__)
    #include <unistd.h>
    #include <sys/times.h>
    typedef tms cpu_time;
    cpu_time get_cpu_time() { 
        cpu_time t; 
        times(&t); 
        return t; 
    }
    double cpu_time_consumed(cpu_time start, cpu_time end) {
        auto nanoseconds_per_clock_tick = double(1000000000) / sysconf(_SC_CLK_TCK);
        auto clock_ticks_elapsed = (end.tms_utime - start.tms_utime) + (end.tms_stime - start.tms_stime);
        return clock_ticks_elapsed * nanoseconds_per_clock_tick;
    }
#endif

#ifdef __linux__
    #include <sched.h>
    void set_affinity(std::uint64_t cpu) {
        
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        
        cpu %= sizeof(int) * 8;
        CPU_SET(cpu, &cpuset);
        
        sched_setaffinity(0, sizeof(cpuset), &cpuset);
    }
#endif

#ifdef __APPLE__
    #include <mach/thread_policy.h>
    #include <pthread.h>

    extern "C" kern_return_t thread_policy_set( thread_t                thread,
                                                thread_policy_flavor_t  flavor,
                                                thread_policy_t         policy_info,
                                                mach_msg_type_number_t  count);

    void set_affinity(std::uint64_t cpu) {

        cpu %= sizeof(integer_t) * 8;
        integer_t count = (1 << cpu);
        thread_policy_set(pthread_mach_thread_np(pthread_self()), THREAD_AFFINITY_POLICY, (thread_policy_t)&count, 1);
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
    double cpu_time_consumed(cpu_time start, cpu_time end) {

        double nanoseconds_per_clock_tick = 100; //100-nanosecond intervals
        auto clock_ticks_elapsed = (make64(end.first) - make64(start.first)) + (make64(end.second) - make64(start.second));
        return clock_ticks_elapsed * nanoseconds_per_clock_tick;
    }
    void set_affinity(std::uint64_t cpu) {

        cpu %= sizeof(std::uint64_t) * 8;
        SetThreadAffinityMask(GetCurrentThread(), 1 << cpu);
    }
#endif

using my_clock = std::conditional<std::chrono::high_resolution_clock::is_steady,
    std::chrono::high_resolution_clock, std::chrono::steady_clock>::type;

static constexpr int measure_count = 1 << 30;
static constexpr int time_target_in_seconds = 5;

template <class R>
double compute_work_item_cost(R r)  {

    auto start = my_clock::now();

    //perform work
    for (int i = 0; i < measure_count; ++i) r.discard(1);

    auto end = my_clock::now();
       
    return std::chrono::nanoseconds(end - start).count() / double(measure_count);
}

class run {
    std::atomic<bool> go, stop;
    std::atomic<std::uint64_t> running, iterations;
public :
    run() : go(false), stop(false), running(0), iterations(0) { }
    struct report {
        double wall_time, cpu_time;
        std::uint64_t steps;
    };
    template <class F>
    report time(int threads, F f, std::uint64_t target_count) {
        for (int i = 0; i < threads; ++i)
            std::thread([&,f, i]() mutable {
                set_affinity(i);
                running++;
                while (go != true) std::this_thread::yield();
                while (stop != true) {
                    f(i);
                    iterations.fetch_add(1, std::memory_order_relaxed);
                }
                running--;
            }).detach();
        while (running != threads) std::this_thread::yield();
        go = true;
        auto cpu_start = get_cpu_time();
        auto start = my_clock::now();
        std::uint64_t it1 = iterations;
        if (threads)
            std::this_thread::sleep_for(std::chrono::seconds(time_target_in_seconds));
        else {
            for (int i = 0; i < target_count; ++i) {
                f(0);
                iterations.fetch_add(1, std::memory_order_relaxed);
            }
        }
        std::uint64_t it2 = iterations;
        auto end = my_clock::now();
        auto cpu_end = get_cpu_time();
        stop = true;
        while (running != 0) std::this_thread::yield();
        std::this_thread::sleep_for(std::chrono::seconds(1));

        report r;
        r.wall_time = double(std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count());
        r.cpu_time = cpu_time_consumed(cpu_start, cpu_end);
        r.steps = (it2 - it1);

        return r;
    }
};

template <class F>
double do_run(std::string const& what, int threads, F f, double expected)
{
    std::cout << what << " (" << threads << "T, " << time_target_in_seconds << "s)" << std::endl;
    auto r = run().time(threads, f, std::uint64_t(time_target_in_seconds * 1E9 / expected));
    std::cout << "\ttotal progress : " << r.steps << " steps in " << r.wall_time << "ns" << std::endl;
    auto time_per_step = r.wall_time / r.steps;
    std::cout << "\trate : " << time_per_step << " ns/step" << std::endl;
    std::cout << "\toverhead over " << expected << " ns/step : " << std::endl;
    auto wall_time = time_per_step - expected;
    std::cout << "\t\t" << std::left << std::setfill(' ') << std::setw(26) << "wall-time = " << wall_time << " ns/step (" << wall_time / expected * 100 << "%)" << std::endl;
    auto cpu_time = r.cpu_time / r.steps - expected;
    std::cout << "\t\t" << std::left << std::setfill(' ') << std::setw(26) << "cpu-time =" << cpu_time << " ns/step (" << cpu_time / expected * 100 << "%)" << std::endl;
    std::cout << std::endl;
    return time_per_step;
}

int main(int argc, const char * argv[]) {

    std::mt19937 r;

    std::cout << "Warming up...\r" << std::flush;
    compute_work_item_cost(r);
    
    std::cout << "Measuring work item cost...\r" << std::flush;
    auto cost = compute_work_item_cost(r);
    
    auto target_count = std::uint64_t(5E1 / cost);
    std::cout << "Work item cost : " << cost << " ns/iteration, targeting " << target_count << " iterations/step.\n";
    std::cout << std::endl;

    auto trial_1 = do_run("Control #1 run for 1-thread", 1, [=](auto) mutable {
        for (int i = 0; i < target_count; ++i) r.discard(1);
    }, target_count * cost) / target_count;
    
    auto cost_1 = (std::min)(trial_1,do_run("Control #2 run for 1-thread", 1, [=](auto) mutable {
        for (int i = 0; i < target_count; ++i) r.discard(1);
    }, target_count * cost) / target_count);

    auto target_count_1 = std::uint64_t(5E1 / (std::min)(cost_1, cost));
    if(cost_1 < cost) {
        std::cout << "Adjusting cost to " << cost_1 << " ns/iteration, and retargeting to " << target_count_1 << " iterations/step.\n";
        std::cout << std::endl;
    }
    
    std::mutex m1;
    do_run("Measuring std::mutex uncontended 1-thread", 1, [=,&m1](auto) mutable {
        { std::unique_lock<std::mutex>(m1); }
        for (int i = 0; i < target_count_1; ++i) r.discard(1);
    }, target_count_1 * cost_1);
    ttas_mutex m2;
    do_run("Measuring ttas_mutex uncontended 1-thread", 1, [=,&m2](auto) mutable {
        { std::unique_lock<ttas_mutex>(m2); }
        for (int i = 0; i < target_count_1; ++i) r.discard(1);
    }, target_count_1 * cost_1);

    auto const N = std::thread::hardware_concurrency();
    constexpr int k_limit = 1024;
    if (N > k_limit) {
        std::cout << "ERROR: Cannot continue because there are more than " << k_limit << " hardware threads on this system.\n";
        std::cout << "ERROR: This is an arbitrary limit. Feel free to raise it, recompile and rerun.\n";
    }

    auto trial_n = do_run("Control #1 for N-thread", N, [=](auto i) mutable {
        for (int i = 0; i < target_count; ++i) r.discard(1);
    }, target_count_1 * cost_1) / target_count_1;
    
    auto cost_n = (std::min)(trial_n,do_run("Control #2 for N-thread", N, [=](auto i) mutable {
        for (int i = 0; i < target_count; ++i) r.discard(1);
    }, target_count_1 * cost_1) / target_count_1);
    
    auto target_count_n = std::uint64_t(5E1 / (std::min)(cost_n, cost_1));
    if(cost_n < cost) {
        std::cout << "NOTE: Based purely on these numbers, your system appears to have hyper-threads enabled.\n";
        std::cout << "NOTE: Will use the N-thread control cost as the base cost.\n";
        std::cout << std::endl;
        std::cout << "Adjusting cost to " << cost_n << " ns/iteration, and retargeting to " << target_count_n << " iterations/step.\n";
        std::cout << std::endl;
    }
    
    std::mutex m1N[k_limit];
    do_run("Measuring std::mutex uncontended N-thread", N, [=,&m1N](auto i) mutable {
        auto& m = m1N[i];
        { std::unique_lock<std::mutex> l(m); }
        for (int i = 0; i < target_count_n; ++i) r.discard(1);
    }, target_count_n * cost_n);
    ttas_mutex m2N[k_limit];
    do_run("Measuring ttas_mutex uncontended N-thread", N, [=,&m2N](auto i) mutable {
        auto& m = m2N[i];
        { std::unique_lock<ttas_mutex> l(m); }
        for (int i = 0; i < target_count_n; ++i) r.discard(1);
    }, target_count_n * cost_n);
    
    auto mask = ~(~0 << std::ilogb(N));
    do_run("Measuring std::mutex low-p contended N-thread", N, [=,&m1N](auto i) mutable {
        auto& m = m1N[r() & mask];
        { std::unique_lock<std::mutex> l(m); }
        for (int i = 0; i < target_count_n-1; ++i) r.discard(1);
    }, target_count_n * cost_n);
    do_run("Measuring ttas_mutex low-p contended N-thread", N, [=, &m2N](auto i) mutable {
        auto& m = m2N[r() & mask];
        { std::unique_lock<ttas_mutex> l(m); }
        for (int i = 0; i < target_count_n-1; ++i) r.discard(1);
    }, target_count_n * cost_n);

    auto short_time = 3E2;
    auto short_count = int(short_time / cost_n);
    do_run("Measuring std::mutex short contended N-thread", N, [=, &m1](auto) mutable {
        std::unique_lock<std::mutex> l(m1);
        for (int i = 0; i < short_count; ++i) r.discard(1);
    }, short_time);
    do_run("Measuring ttas_mutex short contended N-thread", N, [=, &m2](auto) mutable {
        std::unique_lock<ttas_mutex> l(m2); 
        for (int i = 0; i < short_count; ++i) r.discard(1);
    }, short_time);

    auto long_time = 1E7;
    auto long_count = int(long_time / cost_n);
    do_run("Measuring std::mutex long contended N-thread", N, [=, &m1](auto) mutable {
        std::unique_lock<std::mutex> l(m1);
        for (int i = 0; i < long_count; ++i) r.discard(1);
    }, long_time);
    do_run("Measuring ttas_mutex long contended N-thread", N, [=, &m2](auto) mutable {
        std::unique_lock<ttas_mutex> l(m2);
        for (int i = 0; i < long_count; ++i) r.discard(1);
    }, long_time);

    return 0;
}
