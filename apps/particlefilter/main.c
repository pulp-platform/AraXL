#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "runtime.h"
#include "kernel/particlefilter.h"
#ifndef SPIKE
#include "printf.h"
#else
#include "util.h"
#include <stdio.h>
#endif

#include "../common/macros/vector/riscv_util.h"
#include "../common/rivec/vector_defines.h"
//#include "../common/macros/vector/long_array.h"
//#include "../common/macros/vector/float_macros.h"

#define NPARTICLES 64
#define REPETITIONS 10            // run the kernel several times

#define VECTOR//SEQ

double weights[NPARTICLES]      __attribute__((aligned(4 * NR_LANES * NR_CLUSTERS), section(".l2")));
double likelihood[NPARTICLES]   __attribute__((aligned(4 * NR_LANES * NR_CLUSTERS), section(".l2")));
double CDF[NPARTICLES]          __attribute__((aligned(4 * NR_LANES * NR_CLUSTERS), section(".l2")));
double u[NPARTICLES]            __attribute__((aligned(4 * NR_LANES * NR_CLUSTERS), section(".l2")));
uint64_t locations_seq[NPARTICLES]    __attribute__((aligned(4 * NR_LANES * NR_CLUSTERS), section(".l2")));
uint64_t locations_vec[NPARTICLES]    __attribute__((aligned(4 * NR_LANES * NR_CLUSTERS), section(".l2")));

double randu()
{
    return (double)rand() / (double)RAND_MAX;
}


int main()
{
    srand(12);

    int Nparticles = NPARTICLES;
    int reps = 10;
    int x;
    uint64_t time = 0;

    int mismatch = 0;

    printf("Running scalar particle filter\n");
    for (int i = 0; i < reps; i++)
    {
        double sumWeights = 0.0;

        for (x = 0; x < Nparticles; x++)
        {
            likelihood[x] = 1.0 / (x + 1);
        }
            
        for (x = 0; x < Nparticles; x++)
        {
            weights[x] = likelihood[x];
            sumWeights += weights[x];
        }

        for (x = 0; x < Nparticles; x++)
        {
            weights[x] /= sumWeights;
        }

        // Build CDF
        CDF[0] = weights[0];

        for(x = 1; x < Nparticles; x++)
        {
            CDF[x] = CDF[x-1] + weights[x];
        }


        double u1 = (1.0 / Nparticles) * randu();


        for(int x = 0; x < Nparticles; x++)
        {
            u[x] = u1 + x / (double)Nparticles;
        }

        particleFilter(CDF, u, locations_seq, Nparticles);

        start_timer();

        particleFilter_vec(CDF, u, locations_vec, Nparticles);

        stop_timer();

        time += get_timer();

        // Check if scalar and vector execution are identical
        for(x = 0; x < Nparticles; x++)
        {
            if (locations_seq[x] != locations_vec[x]) 
            {
                mismatch++;
            }
        }
    }
    // Print result
    if (mismatch == 0) 
    {
        printf("SUCCESS, seq and vec execution are identical\n");
    } 
    else 
    {
        printf("MISMATCH, seq and vec execution are not identical\n");
    }

    printf("ParticleFilter vector cycles: %d \n\n", time);

    return 0;
}