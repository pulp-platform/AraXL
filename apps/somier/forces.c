#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <inttypes.h>
#include <errno.h>
#include <assert.h>
#include "kernel/somier.h"

void force_contribution(int n1, int n2, double (*X)[n1][n1][n2], double (*F)[n1][n1][n2],
                   int i, int j, int k, int neig_i, int neig_j, int neig_k)
{
   double dx, dy, dz, dl, spring_F, FX, FY,FZ;

   assert (i >= 1); assert (j >= 1); assert (k >= 0);
   assert (i <  n1-1); assert (j <  n1-1); assert (k <  n2-1);
   assert (neig_i >= 0); assert (neig_j >= 0); assert (neig_k >= 0);
   assert (neig_i <  n1); assert (neig_j <  n1); assert (neig_k <  n2);

   dx=X[0][neig_i][neig_j][neig_k]-X[0][i][j][k];
   dy=X[1][neig_i][neig_j][neig_k]-X[1][i][j][k];
   dz=X[2][neig_i][neig_j][neig_k]-X[2][i][j][k];
   dl = sqrt(dx*dx + dy*dy + dz*dz);
   spring_F = 0.25 * spring_K*(dl-1);
   FX = spring_F * dx/dl; 
   FY = spring_F * dy/dl;
   FZ = spring_F * dz/dl; 
   F[0][i][j][k] +=FX;
   F[1][i][j][k] +=FY;
   F[2][i][j][k] +=FZ;
}

void compute_forces(int n1, int n2, double (*X)[n1][n1][n2], double (*F)[n1][n1][n2])
{
   for (int i=1; i<n1-1; i++) {
      for (int j=1; j<n1-1; j++) {
         for (int k=1; k<n2-1; k++) {
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