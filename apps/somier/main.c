// Modified version of somier from RiVEC, adapted to AraXL
// environment. Author: Ivan Herger <iherger@ethz.ch> Check LICENSE
// for additional information

/*************************************************************************
* Somier - RISC-V Vectorized version
* Author: Jesus Labarta
* Barcelona Supercomputing Center
*************************************************************************/

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <inttypes.h>
#include <errno.h>
#include <assert.h>
#include "utils.h"
#include "kernel/somier.h"
#include "runtime.h"
#include "util.h"

#include "../common/macros/vector/riscv_util.h"
#include "../common/rivec/vector_defines.h"
#include "../common/macros/vector/long_array.h"
#include "../common/macros/vector/float_macros.h"

#ifndef SPIKE
#include "printf.h"
#else
#include <stdio.h>
#endif

#define USE_RISCV_VECTOR    // SEQ

double F[3][4][4][128] __attribute__((aligned(4 * NR_LANES * NR_CLUSTERS), section(".l2")));
double V[3][4][4][128] __attribute__((aligned(4 * NR_LANES * NR_CLUSTERS), section(".l2")));
double A[3][4][4][128] __attribute__((aligned(4 * NR_LANES * NR_CLUSTERS), section(".l2")));
double X[3][4][4][128] __attribute__((aligned(4 * NR_LANES * NR_CLUSTERS), section(".l2")));
double X_ref[3][4][4][128] __attribute__((aligned(4 * NR_LANES * NR_CLUSTERS), section(".l2")));


static int compare_X(
    int N1,
    int N2,
    double X_vec[3][4][4][128],
    double X_scalar[3][4][4][128])
{
    const double atol = 1e-12;
    const double rtol = 1e-10;

    int errors = 0;

    for (int d = 0; d < 3; d++) {
        for (int i = 0; i < N1; i++) {
            for (int j = 0; j < N1; j++) {
                for (int k = 0; k < N2; k++) {

                    double vec = X_vec[d][i][j][k];
                    double ref = X_scalar[d][i][j][k];

                    double diff = fabs(vec - ref);
                    double tol  = atol + rtol * fabs(ref);

                    if (diff > tol) {
                        printf(
                            "X MISMATCH [%d][%d][%d][%d]: "
                            "scalar=%f vector=%f "
                            "diff=%f\n",
                            d, i, j, k,
                            ref,
                            vec,
                            diff
                        );

                        errors++;
                    }
                }
            }
        }
    }

    if (errors == 0) {
        printf("\n====================================\n");
        printf("X comparison PASSED\n");
        printf("Scalar and vector results match.\n");
        printf("====================================\n\n");
    } else {
        printf("\n====================================\n");
        printf("X comparison FAILED: %d mismatches\n", errors);
        printf("====================================\n\n");
    }

    return errors;
}


int main() {

    int ntsteps = 3;
    int N1 = 4;
    int N2 = 128;

    int nt;

    // ========================================================================
    // 1. SCALAR REFERENCE
    // ========================================================================

    printf("\n================\n");
    printf("= Somier scalar reference =\n");
    printf("================\n\n");

    clear_4D(N1, N2, F);
    clear_4D(N1, N2, A);
    clear_4D(N1, N2, V);
    init_X(N1, N2, X);

    V[0][N1/2][N1/2][N2/2] = 0.1;
    V[1][N1/2][N1/2][N2/2] = 0.1;
    V[2][N1/2][N1/2][N2/2] = 0.1;

    for (nt = 0; nt < ntsteps - 1; nt++) {

        Xcenter[0] = 0;
        Xcenter[1] = 0;
        Xcenter[2] = 0;

        clear_4D(N1, N2, F);

        compute_forces(N1, N2, X, F);
        acceleration(N1, N2, A, F, M);
        velocities(N1, N2, V, A, dt);
        positions(N1, N2, X, V, dt);

        compute_stats(N1, N2, X, Xcenter);
    }

    // Save scalar X as reference
    memcpy(X_ref, X, sizeof(X_ref));


    // ========================================================================
    // 2. RESET EVERYTHING
    //
    // Important: vector execution must start from exactly the same state.
    // ========================================================================

    clear_4D(N1, N2, F);
    clear_4D(N1, N2, A);
    clear_4D(N1, N2, V);
    init_X(N1, N2, X);

    V[0][N1/2][N1/2][N2/2] = 0.1;
    V[1][N1/2][N1/2][N2/2] = 0.1;
    V[2][N1/2][N1/2][N2/2] = 0.1;


    // ========================================================================
    // 3. VECTOR EXECUTION
    // ========================================================================

    printf("\n================\n");
    printf("= Somier vector =\n");
    printf("================\n\n");

    for (nt = 0; nt < ntsteps - 1; nt++) {

        Xcenter[0] = 0;
        Xcenter[1] = 0;
        Xcenter[2] = 0;

        clear_4D(N1, N2, F);

        compute_forces_prevec(N1, N2, X, F);

        accel_intr(N1, N2, A, F, M);
        vel_intr  (N1, N2, V, A, dt);
        pos_intr  (N1, N2, X, V, dt);

        compute_stats(N1, N2, X, Xcenter);
    }


    // ========================================================================
    // 4. COMPARE VECTOR X AGAINST SCALAR X
    // ========================================================================

    int errors = compare_X(
        N1,
        N2,
        X,
        X_ref
    );


    printf(
        "XC = %f,%f,%f\n",
        Xcenter[0],
        Xcenter[1],
        Xcenter[2]
    );

    printf(
        "\tV= %f, %f, %f\t\t X= %f, %f, %f\n",
        V[0][N1/2][N1/2][N1/2],
        V[1][N1/2][N1/2][N1/2],
        V[2][N1/2][N1/2][N1/2],
        X[0][N1/2][N1/2][N1/2],
        X[1][N1/2][N1/2][N1/2],
        X[2][N1/2][N1/2][N1/2]
    );

    // Return failure if vector result differs from scalar result
    return errors != 0;
}



// int main() {

//   // Default arguments
//   int ntsteps = 3;
//   int N1 = 4;
//   int N2 = 64;

//   printf("\n");
//   printf("================\n");
//   printf("= Somier n2: %d=\n", N2);
// #ifdef SEQ
//   printf("=    scalar    =\n");  
// #else
//   printf("=    vector    =\n");
// #endif

//   printf("================\n");
//   printf("\n");
//   printf("\n");   
  
//   int nt;

//   int avl, vl;

//   uint64_t cycles;

//   clear_4D(N1, N2, F);
//   clear_4D(N1, N2, A);
//   clear_4D(N1, N2, V);
//   //clear_4D(N1, N2, F_ref);
//   init_X(N1, N2, X);

//   V[0][N1/2][N1/2][N2/2] = 0.1;  
//   V[1][N1/2][N1/2][N2/2] = 0.1;  
//   V[2][N1/2][N1/2][N2/2] = 0.1;

//   //start_timer();

//   for (nt=0; nt < ntsteps-1; nt++) {

//     //if(nt%1 == 0 && nt > 0) {
//     //  print_state(N1, N2, X, Xcenter, nt);
//     //}

//     //boundary(N1, N2, X, V);
//     Xcenter[0]=0, Xcenter[1]=0; Xcenter[2]=0;   //reset aggregate stats
//     clear_4D(N1, N2, F);
  

//   #ifdef SEQ
//     compute_forces(N1, N2, X, F);
//     //capture_4D_ref(N1, N2, F, F_ref);      // only necessary to ensure correctness of kernel
//     //test_4D_result(N1, N2, F, F_ref);      // only necessary to ensure correctness of kernel
//     acceleration(N1, N2, A, F, M);
//     velocities(N1, N2, V, A, dt);
//     positions(N1, N2, X, V, dt);
//   #else 
//     //start_timer();
//     compute_forces_prevec(N1, N2, X, F); 

//     //print_state(N1, N2, F, Xcenter, 2);
//     //stop_timer();
//     //printf("Somier vector cycles %d \n\n", get_timer());
//     //print_state(N1, N2, F, Xcenter, nt);  
//     accel_intr  (N1, N2, A, F, M);   // Output A
//     //printf ("Computed Accelerations\n"); print_4D(N1, N2, "A", A); printf ("\n");
//     vel_intr  (N1, N2, V, A, dt);    // Output V  
//     // printf ("Computed Velocities\n"); print_4D(N1, N2,"V", V); printf ("\n");
//     pos_intr  (N1, N2, X, V, dt);    // Output X
//     //print_4D(N1, N2,"X", X); printf ("\n"); 
//   #endif

//   compute_stats(N1, N2, X, Xcenter);
//   }

//    double sum0, sum1, sum2 = 0.0;
//   // for (int i = 1; i<3; i++) {

//   //     for (int j = 1; j<2; j++) {
//   //             sum2 = 0;
//   //        for (int k = 0; k<N2; k++) {
//   //           //Xcenter[0] += F[0][0][j][k];
//   //           //Xcenter[1] += F[1][0][j][k];
//   //           //sum2 += F[2][i][j][k];
//   //            printf("value is %d, is:%f \n", k, F[2][i][j][k]);
//   //        }


//   //     }

//   // }


//    printf ("XC= %f,%f,%f  \n", Xcenter[0],  Xcenter[1], Xcenter[2]);

//   //stop_timer();

//   #ifdef SEQ
//   printf("Somier scalar cycles %d \n\n", get_timer());
//   #else
//   printf("Somier vector cycles %d \n\n", get_timer());
//   #endif

//   print_state(N1, N2, F, Xcenter, 2);

//   printf ("\tV= %f, %f, %f\t\t X= %f, %f, %f\n",
//     V[0][N1/2][N1/2][N1/2], V[1][N1/2][N1/2][N1/2], V[2][N1/2][N1/2][N1/2],
//     X[0][N1/2][N1/2][N1/2], X[1][N1/2][N1/2][N1/2], X[2][N1/2][N1/2][N1/2]);

//   return 0;
// }