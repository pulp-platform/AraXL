// /**
//  * @file ex_particle_OPENMP_seq.c
//  * @author Michael Trotter & Matt Goodrum
//  * @brief Particle filter implementation in C/OpenMP 
//  */

// /*************************************************************************
// * RISC-V Vectorized Version
// * Author: Cristóbal Ramírez Lazo
// * email: cristobal.ramirez@bsc.es
// * Barcelona Supercomputing Center (2020)
// *************************************************************************/

// #include <stdlib.h>
// #include <stdio.h>
// #include <string.h>
// #include <math.h>
// #include <sys/time.h>
// #include <time.h> 

// #ifdef USE_RISCV_VECTOR
// #include <riscv_vector.h>
// #include "../../common/vector_defines.h"
// #endif

// #include "../../common/riscv_util.h"

// //#include <omp.h>
// #include <limits.h>
// #define PI 3.1415926535897932
// #define SEQ    // SEQ or VECTOR

// /**
// @var M value for Linear Congruential Generator (LCG); use GCC's value
// */
// long M = INT_MAX;
// /**
// @var A value for LCG
// */
// int A = 1103515245;
// /**
// @var C value for LCG
// */
// int C = 12345;

// int x;
// int j;

// double sumWeights = 0.0;
// double u1;

// double *weights;     // size: Nparticles
// double *CDF;         // size: Nparticles
// double *u;           // size: Nparticles

// int *locations;      // size: Nparticles
// int *seed;           // random number generator state

// int Nparticles = 128;




// int findIndex(double * CDF, int lengthCDF, double value){
//     int index = -1;
//     int x;

//     for(x = 0; x < lengthCDF; x++){
//         if(CDF[x] >= value){
//             index = x;
//             break;
//         }
//     }
//     if(index == -1){
//         return lengthCDF-1;
//     }
//     return index;
// }


// int main(int argc, char * argv[]){

//     weights   = (double *)malloc(Nparticles * sizeof(double));
//     CDF       = (double *)malloc(Nparticles * sizeof(double));
//     u         = (double *)malloc(Nparticles * sizeof(double));
//     locations = (int *)malloc(Nparticles * sizeof(int));

// #ifdef SEQ
//     printf("=    scalar    =\n");  
// #else
//     printf("=    vector    =\n");
// #endif


// double sumWeights = 0.0;

//     for (x = 0; x < Nparticles; x++){
//         weights[x] = 1.0 / Nparticles;
//     }

//     for (x = 0; x < Nparticles; x++){
//         sumWeights += weights[x];
//     }

//     for (x = 0; x < Nparticles; x++){
//         weights[x] /= sumWeights;
//     }

//     CDF[0] = weights[0];
//     for(x = 1; x < Nparticles; x++){
//         CDF[x] = weights[x] + CDF[x-1];
//     }    

// #ifdef SEQ
//     double u1 = (1.0 / Nparticles) * randu(seed, 0);

//     for (x = 0; x < Nparticles; x++)
//         u[x] = u1 + x / (double)Nparticles;

//     for (j = 0; j < Nparticles; j++) {
//         locations[j] = findIndex(CDF, Nparticles, u[j]);
//         if (locations[j] == -1)
//             locations[j] = Nparticles - 1;
//     }

// #else
//     for(i = 0; i < Nparticles; i=i+gvl){
//         gvl     = __riscv_vsetvl_e64m1(Nparticles-i);
//         vector_complete = 0;
//         xMask   = _MM_CAST_i1_i64(_MM_SET_i64(0,gvl));
//         xArray  = _MM_SET_i64(Nparticles-1,gvl);
//         xU      = _MM_LOAD_f64(&u[i],gvl);
//         for(j = 0; j < Nparticles; j++){    
//             xCDF = _MM_SET_f64(CDF[j],gvl);
//             xComp = _MM_VFGE_f64(xCDF,xU,gvl);
//             xComp = _MM_VMXOR_i64(xComp,xMask,gvl);
//             valid = _MM_VMFIRST_i64(xComp,gvl);
//             if(valid != -1)
//             {
//                 xArray = _MM_MERGE_i64(xArray,_MM_SET_i64(j,gvl),xComp,gvl);
//                 xMask = _MM_VMOR_i64(xComp,xMask,gvl);
//                 vector_complete = _MM_VMPOPC_i64(xMask,gvl);
//             }
//             if(vector_complete == gvl){ break; }
//         }
//         _MM_STORE_i64(&locations[i],xArray,gvl);
//     }
// #endif

//     return 0;
// }

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
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

#define NPARTICLES 16

#define VECTOR//SEQ

double weights[NPARTICLES]      __attribute__((aligned(4 * NR_LANES * NR_CLUSTERS), section(".l2")));
double likelihood[NPARTICLES]   __attribute__((aligned(4 * NR_LANES * NR_CLUSTERS), section(".l2")));
double CDF[NPARTICLES]          __attribute__((aligned(4 * NR_LANES * NR_CLUSTERS), section(".l2")));
double u[NPARTICLES]            __attribute__((aligned(4 * NR_LANES * NR_CLUSTERS), section(".l2")));
uint64_t locations[NPARTICLES]    __attribute__((aligned(4 * NR_LANES * NR_CLUSTERS), section(".l2")));

double randu()
{
    return (double)rand() / (double)RAND_MAX;
}


int main()
{
    srand(12);

    int Nparticles = NPARTICLES;
    int x;

    printf("Running scalar particle filter\n");

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

    // printf("Weights:\n");
    // for(x = 0; x < Nparticles; x++)
    // {
    //     printf("%f ", weights[x]);
    // }
    // printf("\n");

    // Build CDF
    CDF[0] = weights[0];

    for(x = 1; x < Nparticles; x++)
    {
        CDF[x] = CDF[x-1] + weights[x];
    }

    // printf("CDF:\n");
    // for(x = 0; x < Nparticles; x++)
    // {
    //     printf("%f ", CDF[x]);
    // }
    // printf("\n");

    double u1 = (1.0 / Nparticles) * randu();


    for(int x = 0; x < Nparticles; x++)
    {
        u[x] = u1 + x / (double)Nparticles;
    }

    // printf("u:\n");
    // for(x = 0; x < Nparticles; x++)
    // {
    //     printf("%f ", u[x]);
    // }
    // printf("\n");


#ifdef SEQ

    particleFilter(weights, CDF, u, locations, Nparticles);

#else
    particleFilter(weights, CDF, u, locations, Nparticles);
    printf("Selected locations:\n");
    for(x = 0; x < Nparticles; x++)
    {
        printf("%lu ", locations[x]);
    }
    printf("\n");
    particleFilter_vec(weights, CDF, u, locations, Nparticles);

#endif

    // Print result
    printf("Selected locations:\n");
    for(x = 0; x < Nparticles; x++)
    {
        printf("%lu ", locations[x]);
    }
    printf("\n");

    return 0;
}