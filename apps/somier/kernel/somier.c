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


#include "../utils.h"

double ref[1000][3];
double Xcenter[3];

double dt=0.1;   // 0.1;
double M=1.0;
double spring_K=10.0;

void init_X (int n1, int n2, double (*X)[n1][n1][n2])
{
   int i, j, k;

   Xcenter[0]=0, Xcenter[1]=0; Xcenter[2]=0;

   for (i = 0; i<n1; i++)
      for (j = 0; j<n1; j++)
         for (k = 0; k<n2; k++) {
            X[0][i][j][k] = i;
            X[1][i][j][k] = j;
            X[2][i][j][k] = k;
           
           if (k<n1) {
            Xcenter[0] += X[0][i][j][k];
            Xcenter[1] += X[1][i][j][k];
            Xcenter[2] += X[2][i][j][k];
           }
         }

    Xcenter[0] /= (n1*n1*n1);
    Xcenter[1] /= (n1*n1*n1);
    Xcenter[2] /= (n1*n1*n1);
}

//make sure the boundary nodes are fixed

void boundary(int n1, int n2, double (*X)[n1][n1][n2], double (*V)[n1][n1][n2])
{
   int i, j, k;
   i = 0;
   for (j = 0; j<n1; j++) {
      for (k = 0; k<n1; k++) {
         X[0][i][j][k] = i;   X[1][i][j][k] = j;   X[2][i][j][k] = k;
         V[0][i][j][k] = 0.0; V[1][i][j][k] = 0.0; V[2][i][j][k] = 0.0;
      }
   }
   j = 0;
   for (i = 0; i<n1; i++) {
      for (k = 0; k<n1; k++) {
         X[0][i][j][k] = i;   X[1][i][j][k] = j;   X[2][i][j][k] = k;
         V[0][i][j][k] = 0.0; V[1][i][j][k] = 0.0; V[2][i][j][k] = 0.0;
      }
   }
   k = 0;
   for (i = 0; i<n1; i++) {
      for (j = 0; j<n1; j++) {
         X[0][i][j][k] = i;   X[1][i][j][k] = j;   X[2][i][j][k] = k;
         V[0][i][j][k] = 0.0; V[1][i][j][k] = 0.0; V[2][i][j][k] = 0.0;
      }
   }
   k = n1-1;
   for (i = 0; i<n1; i++) {
      for (j = 0; j<n1; j++) {
         X[0][i][j][k] = i;   X[1][i][j][k] = j;   X[2][i][j][k] = k;
         V[0][i][j][k] = 0.0; V[1][i][j][k] = 0.0; V[2][i][j][k] = 0.0;
      }
   }
   i = n1-1;
   for (j = 0; j<n1; j++) {
      for (k = 0; k<n1; k++) {
         X[0][i][j][k] = i;   X[1][i][j][k] = j;   X[2][i][j][k] = k;
         V[0][i][j][k] = 0.0; V[1][i][j][k] = 0.0; V[2][i][j][k] = 0.0;
      }
   }
   j = n1-1;
   for (i = 0; i<n1; i++) {
      for (k = 0; k<n1; k++) {
         X[0][i][j][k] = i;   X[1][i][j][k] = j;   X[2][i][j][k] = k;
         V[0][i][j][k] = 0.0; V[1][i][j][k] = 0.0; V[2][i][j][k] = 0.0;
      }
   }

}

void force_contribution(int n1, int n2, double (*X)[n1][n1][n2], double (*F)[n1][n1][n2],
                   int i, int j, int k, int neig_i, int neig_j, int neig_k);


void compute_force_new(int n1, int n2, double (*X)[n1][n1][n2], double (*F)[n1][n1][n2])
{
   for (int i=1; i<n1-1; i++) {
      for (int j=1; j<n1-1; j++) {
         for (int k=1; k<n1-1; k++) {
            force_contribution (n1, n2, X, F, i, j, k, i-1, j,   k);
            force_contribution (n1, n2, X, F, i, j, k, i+1, j,   k);
            force_contribution (n1, n2, X, F, i, j, k, i,   j-1, k);
            force_contribution (n1, n2, X, F, i, j, k, i,   j+1, k);
            force_contribution (n1, n2, X, F, i, j, k, i,   j,   k-1);
            force_contribution (n1, n2, X, F, i, j, k, i,   j,   k+1);
         }
      }
   }
}

void acceleration(int n1, int n2, double (*A)[n1][n1][n2], double (*F)[n1][n1][n2], double M)
{
   int i, j, k;
//#dear compiler: please fuse next two loops if you can
   for (i = 0; i<n1; i++)
      for (j = 0; j<n1; j++)
         for (k = 0; k<n1; k++) {
            A[0][i][j][k]= F[0][i][j][k]/M;
            A[1][i][j][k]= F[1][i][j][k]/M;
            A[2][i][j][k]= F[2][i][j][k]/M;
	 }

}


void velocities(int n1, int n2, double (*V)[n1][n1][n2], double (*A)[n1][n1][n2], double dt)
{
   int i, j, k;
//#dear compiler: please fuse next two loops if you can
   for (i = 0; i<n1; i++)
//      #pragma omp task
//      #pragma omp unroll
      for (j = 0; j<n1; j++) {
	 #pragma omp simd
         for (k = 0; k<n1; k++) {
               V[0][i][j][k] += A[0][i][j][k]*dt;
               V[1][i][j][k] += A[1][i][j][k]*dt;
               V[2][i][j][k] += A[2][i][j][k]*dt;
            }
     }
}

void positions(int n1, int n2, double (*X)[n1][n1][n2], double (*V)[n1][n1][n2], double dt)
{
   int i, j, k;
//#dear compiler: please fuse next two loops if you can
   for (i = 0; i<n1; i++)
      for (j = 0; j<n1; j++)
         for (k = 0; k<n1; k++) {
               X[0][i][j][k] += V[0][i][j][k]*dt;
               X[1][i][j][k] += V[1][i][j][k]*dt;
               X[2][i][j][k] += V[2][i][j][k]*dt;
            }
}

void compute_stats(int n1, int n2, double (*X)[n1][n1][n2], double Xcenter[3])
{
   double sum0, sum1, sum2 = 0.0;
   for (int i = 0; i<n1; i++) {
      for (int j = 0; j<n1; j++) {
         for (int k = 0; k<n1; k++) {
            Xcenter[0] += X[0][i][j][k];
            Xcenter[1] += X[1][i][j][k];
            Xcenter[2] += X[2][i][j][k];
         }
      }
   }
   
   Xcenter[0] /= (n1*n1*n1);
   Xcenter[1] /= (n1*n1*n1);
   Xcenter[2] /= (n1*n1*n1);
}