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

//#undef _WIN32_WINNT
//#define _WIN32_WINNT 0x0602

unsigned next_table[] = 
    {
        0, 1, 2, 3,         //0-3
        4, 4, 6, 6,         //4-7
        8, 8, 8, 8,         //8-11
        12, 12, 12, 12,     //12-15
        16, 16, 16, 16,     //16-19
        16, 16, 16, 16,     //20-23
        24, 24, 24, 24,     //24-27
        24, 24, 24, 24,     //28-31
        32, 32, 32, 32,     //32-35
        32, 32, 32, 32,     //36-39
        40, 40, 40, 40,     //40-43
        40, 40, 40, 40,     //44-47
        48, 48, 48, 48,     //48-51
        48, 48, 48, 48,     //52-55
        56, 56, 56, 56,     //56-59
        56, 56, 56, 56,     //60-63
    };

//change this if you want to allow oversubscription of the system, by default only the range {1-(system size)} is tested
#define FOR_GAUNTLET(x) for(unsigned x = (std::min)(std::thread::hardware_concurrency()*2,unsigned(sizeof(next_table)/sizeof(unsigned))); x; x = next_table[x-1])

//set this to override the benchmark of barriers to use OMP barriers instead of n3998 std::barrier
//#define USEOMP

#ifdef USEOMP
#include <omp.h>
#endif

#include <iostream>
#include <sstream>
#include <algorithm>
#include <string>
#include <vector>
#include <map>
#include <cstring>
#include <ctime>

//#include <details/config>
//#undef __SYNCHRONIC_COMPATIBLE

#include <synchronic>
#include <n3998>

#include "test.hpp"

#if defined(__SYNCHRONIC_COMPATIBLE)
    #define PREFIX "futex-"
#else
    #define PREFIX "backoff-"
#endif

//this test uses a custom Mersenne twister to eliminate implementation variation
MersenneTwister mt;

double time_item()  {

    clock_t const start = clock();

    int const realcount = (int)1E8;
    for(int i = 0;i < realcount; ++i)
        mt.integer();

    clock_t const end = clock();
    double elapsed_seconds = (end - start) / double(CLOCKS_PER_SEC);

    return elapsed_seconds / realcount;
}


template <class mutex_type>
void testmutex_inner(mutex_type& m, std::atomic<int>& t,std::atomic<int>& wc,std::atomic<int>& wnc, int const num_iterations, 
                     int const num_items_critical, int const num_items_noncritical, MersenneTwister& mtc, MersenneTwister& mtnc, bool skip) {

    for(int k = 0; k < num_iterations; ++k) {

        if(num_items_noncritical) {
            // Do some work without holding the lock
            int workunits = (int) (mtnc.poissonInterval((float)num_items_noncritical) + 0.5f);
            for (int i = 1; i < workunits; i++)
                mtnc.integer();       // Do one work unit
            wnc.fetch_add(workunits,std::memory_order_relaxed);
        }

        if(!skip) {
            std::unique_lock<mutex_type> l(m);
            t.fetch_add(1,std::memory_order_relaxed);

            if(num_items_critical) {
                // Do some work while holding the lock
                int workunits = (int) (mtc.poissonInterval((float)num_items_critical) + 0.5f);
                for (int i = 1; i < workunits; i++)
                    mtc.integer();       // Do one work unit
                wc.fetch_add(workunits,std::memory_order_relaxed);
            }
        }
        else
            t.fetch_add(1,std::memory_order_relaxed);
    }
}
template <class mutex_type>
void testmutex_outer(std::map<std::string,std::vector<double>>& results, std::string const& name, double critical_fraction, double critical_duration) {

    std::ostringstream truename;
    truename << name << " (f=" << critical_fraction << ",d=" << critical_duration << ")";

    std::vector<double>& data = results[truename.str()];

    double const workItemTime = time_item();
    int const num_items_critical = int( critical_duration / workItemTime + 0.5 ),
              num_items_noncritical = int( critical_duration / critical_fraction / workItemTime + 0.5 );

    FOR_GAUNTLET(num_threads) {

        int const num_iterations = (num_items_critical + num_items_noncritical != 0) ? 
#ifdef __SYNCHRONIC_JUST_YIELD
                                        int( 1 / ( 8 * workItemTime ) / (num_items_critical + num_items_noncritical) / num_threads + 0.5 ) : 
#else
                                        int( 1 / ( 2 * workItemTime ) / (num_items_critical + num_items_noncritical) / num_threads + 0.5 ) : 
#endif
                                        int( double(5E5) / num_threads );
        std::cerr << "running " << truename.str() << " #" << num_threads << ", " << num_iterations << " * " << num_items_noncritical << "\r" << std::flush;

        std::atomic<int> t[2], wc[2], wnc[2];

        clock_t start[2], end[2];
        for(int pass = 0; pass < 2; ++pass) {

            t[pass] = 0;
            wc[pass] = 0;
            wnc[pass] = 0;

            srand(num_threads);
            std::vector<MersenneTwister> randomsnc(num_threads), 
                                         randomsc(num_threads);

            mutex_type m;

            start[pass] = clock();
#ifdef USEOMP
            omp_set_num_threads(num_threads);
            std::atomic<int> _j(0);
            #pragma omp parallel
            {
                int const j = _j.fetch_add(1,std::memory_order_relaxed);
                testmutex_inner(m, t[pass], wc[pass], wnc[pass], num_iterations, num_items_critical, num_items_noncritical, randomsc[j], randomsnc[j], pass==0);
                num_threads = omp_get_num_threads();
            }
#else
            std::vector<std::thread*> threads(num_threads);
            for(unsigned j = 0; j < num_threads; ++j)
                threads[j] = new std::thread([&,j](){ 
                        testmutex_inner(m, t[pass], wc[pass], wnc[pass], num_iterations, num_items_critical, num_items_noncritical, randomsc[j], randomsnc[j], pass==0);
                    }
                );
            for(unsigned j = 0; j < num_threads; ++j) {
                threads[j]->join();
                delete threads[j];
            }
#endif
            end[pass] = clock();
        }
        if(t[0] != t[1]) throw std::string("mismatched iteration counts");
        if(wnc[0] != wnc[1]) throw std::string("mismatched work item counts");

        double elapsed_seconds_0 = (end[0] - start[0]) / double(CLOCKS_PER_SEC),
               elapsed_seconds_1 = (end[1] - start[1]) / double(CLOCKS_PER_SEC);
        double time = (elapsed_seconds_1 - elapsed_seconds_0 - wc[1]*workItemTime) / num_iterations;

        data.push_back(time);
        std::cerr << truename.str() << " : " << num_threads << "," << elapsed_seconds_1 / num_iterations << " - " << elapsed_seconds_0 / num_iterations << " - " << wc[1]*workItemTime/num_iterations << " = " << time << "                                                 \n";
    }
}

template <class barrier_type>
void testbarrier_inner(barrier_type& b, int const num_threads, int const j, std::atomic<int>& t,std::atomic<int>& w, 
                       int const num_iterations_odd, int const num_iterations_even,
                       int const num_items_noncritical, MersenneTwister& mt, bool skip) {

    for(int k = 0; k < (std::max)(num_iterations_even,num_iterations_odd); ++k) {

        if(k >= (~j & 0x1 ? num_iterations_odd : num_iterations_even )) {
            if(!skip)
                b.arrive_and_drop();
            break;
        }

        if(num_items_noncritical) {
            // Do some work without holding the lock
            int workunits = (int) (mt.poissonInterval((float)num_items_noncritical) + 0.5f);
            for (int i = 1; i < workunits; i++)
                mt.integer();       // Do one work unit
            w.fetch_add(workunits,std::memory_order_relaxed);
        }

        t.fetch_add(1,std::memory_order_relaxed);

        if(!skip) {
            int const thiscount = (std::min)(k+1,num_iterations_odd)*((num_threads>>1)+(num_threads&1)) + (std::min)(k+1,num_iterations_even)*(num_threads>>1);
            if(t.load(std::memory_order_relaxed) > thiscount)
                std::cerr << "FAILURE: some threads have run ahead of the barrier (" << t.load(std::memory_order_relaxed) << ">" <<  thiscount << ").\n";
#ifdef USEOMP
            #pragma omp barrier
#else
            b.arrive_and_wait();
#endif
            if(t.load(std::memory_order_relaxed) < thiscount)
                std::cerr << "FAILURE: some threads have fallen behind the barrier (" << t.load(std::memory_order_relaxed) << "<" << thiscount << ").\n";
        }
    }
}
template <class barrier_type>
void testbarrier_outer(std::map<std::string,std::vector<double>>& results, std::string const& name, double barrier_frequency, double phase_duration, bool randomIterations = false) {

    std::vector<double>& data = results[name];

    double const workItemTime = time_item();
    int const num_items_noncritical = int( phase_duration / workItemTime + 0.5 );

    FOR_GAUNTLET(num_threads) {

        int const num_iterations = int( barrier_frequency );
        std::cerr << "running " << name << " #" << num_threads << ", " << num_iterations << " * " << num_items_noncritical << "\r" << std::flush;

        srand(num_threads);

        MersenneTwister mt;
        int const num_iterations_odd = randomIterations ? int(mt.poissonInterval((float)num_iterations)+0.5f) : num_iterations,
                  num_iterations_even = randomIterations ? int(mt.poissonInterval((float)num_iterations)+0.5f) : num_iterations;

        std::atomic<int> t[2], w[2];
        std::chrono::time_point<std::chrono::high_resolution_clock> start[2], end[2];
        for(int pass = 0; pass < 2; ++pass) {

            t[pass] = 0;
            w[pass] = 0;

            srand(num_threads);
            std::vector<MersenneTwister> randoms(num_threads);

            barrier_type b(num_threads);

            start[pass] = std::chrono::high_resolution_clock::now();
#ifdef USEOMP
            omp_set_num_threads(num_threads);
            std::atomic<int> _j(0);
            #pragma omp parallel
            {
                int const j = _j.fetch_add(1,std::memory_order_relaxed);
                testbarrier_inner(b, num_threads, j, t[pass], w[pass], num_iterations_odd, num_iterations_even, num_items_noncritical, randoms[j], pass==0);
                num_threads = omp_get_num_threads();
            }
#else
            std::vector<std::thread*> threads(num_threads);
            for(unsigned j = 0; j < num_threads; ++j)
                threads[j] = new std::thread([&,j](){ 
                    testbarrier_inner(b, num_threads, j, t[pass], w[pass], num_iterations_odd, num_iterations_even, num_items_noncritical, randoms[j], pass==0);
                });
            for(unsigned j = 0; j < num_threads; ++j) {
                threads[j]->join();
                delete threads[j];
            }
#endif
            end[pass] = std::chrono::high_resolution_clock::now();
        }

        if(t[0] != t[1]) throw std::string("mismatched iteration counts");
        if(w[0] != w[1]) throw std::string("mismatched work item counts");

        int const phases = (std::max)(num_iterations_odd, num_iterations_even);

        std::chrono::duration<double> elapsed_seconds_0 = end[0]-start[0],
                                      elapsed_seconds_1 = end[1]-start[1];
        double const time = (elapsed_seconds_1.count() - elapsed_seconds_0.count()) / phases;

        data.push_back(time);
        std::cerr << name << " : " << num_threads << "," << elapsed_seconds_1.count() / phases << " - " << elapsed_seconds_0.count() / phases << " = " << time << "                                                 \n";
    }
}

template <class... T>
struct mutex_tester;
template <class F>
struct mutex_tester<F> {
    static void run(std::map<std::string,std::vector<double>>& results, std::string const name[], double critical_fraction, double critical_duration) {
        testmutex_outer<F>(results, *name, critical_fraction, critical_duration);
    }
};
template <class F, class... T>
struct mutex_tester<F,T...> {
    static void run(std::map<std::string,std::vector<double>>& results, std::string const name[], double critical_fraction, double critical_duration) {
        mutex_tester<F>::run(results, name, critical_fraction, critical_duration);
        mutex_tester<T...>::run(results, ++name, critical_fraction, critical_duration);
    }
};

int main(int argc, char * argv[]) {

    //warm up
    time_item();
    std::cerr << "measuring work item speed...\r";
    //measure up
    std::cerr << "work item speed is " << time_item() << " per item\n";
    try {

        std::pair<double,double> testpoints[] = { /*{1E-1, 10E-6}, {5E-1, 2E-6},*/ {3E-1, 100E-9}, {1, 0} };
        for(auto x : testpoints ) {

            std::map<std::string,std::vector<double>> results;

            //testbarrier_outer<std::barrier>(results, PREFIX"bar 1khz 100us", 1E3, x.second);

            std::string const names[] = { 
                PREFIX"tkt" , PREFIX"mcs", PREFIX"std",    PREFIX"ttas" //,    PREFIX"cas-spin",   PREFIX"cas-yield" 
            };

            //run -->

            mutex_tester<
                ticket_mutex , mcs_mutex, std::mutex,     ttas_mutex //,      dumb_mutex<true>,   dumb_mutex<false>
            >::run(results, names, x.first, x.second);

            //<-- run

            std::cout << "threads";
            for(auto & i : results)
                std::cout << ",\"" << i.first << '\"';
            std::cout << std::endl;
            int j = 0;
            FOR_GAUNTLET(num_threads) {
                std::cout << num_threads;
                for(auto & i : results)
                    std::cout << ',' << i.second[j];
                std::cout << std::endl;
                ++j;
            }
        }
    }
    catch(std::string & e) {
        std::cerr << "EXCEPTION : " << e << std::endl;
    }
    return 0;
}
