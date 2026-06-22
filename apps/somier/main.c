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

double F[3][10][10][32] __attribute__((aligned(4 * NR_LANES * NR_CLUSTERS), section(".l2")));
double V[3][10][10][32] __attribute__((aligned(4 * NR_LANES * NR_CLUSTERS), section(".l2")));
double A[3][10][10][32] __attribute__((aligned(4 * NR_LANES * NR_CLUSTERS), section(".l2")));
double X[3][10][10][32] __attribute__((aligned(4 * NR_LANES * NR_CLUSTERS), section(".l2")));
double F_ref[3][10][10][32] __attribute__((aligned(4 * NR_LANES * NR_CLUSTERS), section(".l2")));


int main() {
  printf("\n");
  printf("================\n");
  printf("=    Somier    =\n");
#ifdef SEQ
  printf("=    scalar    =\n");  
#else
  printf("=    vector    =\n");
#endif

  printf("================\n");
  printf("\n");
  printf("\n");   
  
  int nt;

  // Default arguments
  int ntsteps = 5;
  int N1 = 10;
  int N2 = 32;

  int avl, vl;

  uint64_t cycles;

  clear_4D(N1, N2, F);
  clear_4D(N1, N2, A);
  clear_4D(N1, N2, V);
  clear_4D(N1, N2, F_ref);
  init_X(N1, N2, X);

  V[0][N1/2][N1/2][N1/2] = 0.1;  
  V[1][N1/2][N1/2][N1/2] = 0.1;  
  V[2][N1/2][N1/2][N1/2] = 0.1;

  //start_timer();

  for (nt=0; nt < ntsteps-1; nt++) {

    //if(nt%1 == 0) {
    //  print_state(N1, N2, X, Xcenter, nt);
    //}

    //boundary(N1, N2, X, V);
    Xcenter[0]=0, Xcenter[1]=0; Xcenter[2]=0;   //reset aggregate stats
    clear_4D(N1, N2, F);
  
  start_timer();

  #ifdef SEQ
    compute_forces(N1, N2, X, F);
    //capture_4D_ref(N1, N2, F, F_ref);      // only necessary to ensure correctness of kernel
    //test_4D_result(N1, N2, F, F_ref);      // only necessary to ensure correctness of kernel
    acceleration(N1, N2, A, F, M);
    velocities(N1, N2, V, A, dt);
    positions(N1, N2, X, V, dt);
  #else 
    compute_forces_prevec(N1, N2, X, F); 
    //print_state(N1, N2, F, Xcenter, nt);  
    accel_intr  (N1, N2, A, F, M);   
    // printf ("Computed Accelerations\n"); print_4D(N1, N2, "A", A); printf ("\n");
    vel_intr  (N1, N2, V, A, dt);      
    // printf ("Computed Velocities\n"); print_4D(N1, N2,"V", V); printf ("\n");
    pos_intr  (N1, N2, X, V, dt);          
    //print_4D(N1, N2,"X", X); printf ("\n"); 
  #endif
  compute_stats(N1, N2, X, Xcenter);
  }

  stop_timer();

  #ifdef SEQ
  printf("Somier scalar cycles %d \n\n", get_timer());
  #else
  printf("Somier vector cycles %d \n\n", get_timer());
  #endif

    print_state(N1, N2, F, Xcenter, 4);

  printf ("\tV= %f, %f, %f\t\t X= %f, %f, %f\n",
    V[0][N1/2][N1/2][N1/2], V[1][N1/2][N1/2][N1/2], V[2][N1/2][N1/2][N1/2],
    X[0][N1/2][N1/2][N1/2], X[1][N1/2][N1/2][N1/2], X[2][N1/2][N1/2][N1/2]);

  return 0;
}







