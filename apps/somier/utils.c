#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <inttypes.h>
#include <errno.h>
#include <assert.h>
#include "printf.h"
#include "utils.h"

void capture_ref_result(double *y, double* y_ref, int n)
{
   int i;
   for (i=0; i<n; i++) {
      y_ref[i]=y[i];
   }
}

void print_4D (int n1, int n2, char*name, double (*y)[n1][n1][n2])
{
   int i,j,k, dim;
   for (dim=0; dim<3; dim++) {
      for (i = 0; i<n1; i++) {
         for (j = 0; j<n1; j++) {
            printf("\n%s[%d][%d][%d][0..%d] ", name, dim, i, j, n1-1);
            for (k = 0; k<n1; k++) {
               printf("%.4f ", y[dim][i][j][k]);
            }
         }
      }
   }
   printf ("\n");
}

void capture_4D_ref (int n1, int n2, double (*y)[n1][n1][n2], double (*y_ref)[n1][n1][n2])
{
   int i,j,k, dim;
   for (dim=0; dim<3; dim++) {
      for (i = 0; i<n1; i++) {
         for (j = 0; j<n1; j++) {
            for (k = 0; k<n1; k++) {
               y_ref[dim][i][j][k] = y[dim][i][j][k];
            }
         }
      }
   }
}

void test_4D_result(int n1, int n2, double (*y)[n1][n1][n2], double (*y_ref)[n1][n1][n2])
{
   int nerrs=0;
   int i,j,k, dim;

   for (dim=0; dim<3; dim++) {
      for (i = 0; i<n1; i++) {
         for (j = 0; j<n1; j++) {
            for (k = 0; k<n1; k++) {
               double error = y[dim][i][j][k] - y_ref[dim][i][j][k];

               if (fabs(error) > 0.0000001)  {
                  printf("F[%d][%d][%d][%d]=%.16f != F_ref[%d][%d][%d][%d]=%.16f  INCORRECT RESULT (abs error=%.16f) !!!! \n ",
                      dim,i,j,k,y[dim][i][j][k], dim,i,j,k, y_ref[dim][i][j][k], error);
                  nerrs++;
               }
               if (nerrs == 100) break;
            }
         }
      }
   }
   if (nerrs == 0) printf ("Result ok !!!\n");
}

void clear_4D(int n1, int n2, double (*X)[n1][n1][n2])
{
   int i, j, k;
   for (i = 0; i<n1; i++) 
      for (j = 0; j<n1; j++)
         for (k = 0; k<n2; k++) { 
            X[0][i][j][k] = 0.0; X[1][i][j][k] = 0.0; X[2][i][j][k] = 0.0;
         }

}


void print_state(int n1, int n2, double (*X)[n1][n1][n2], double Xcenter[3], int nt){
   printf ("t=%d\t", nt);
   printf ("XC= %f,%f,%f X[n/2-1] = %f,%f,%f  X[n/2] = %f,%f,%f X[n/2+1] = %f,%f,%f \n",
            Xcenter[0],            Xcenter[1],            Xcenter[2],
            X[0][n1/2-1][n1/2][n1/2], X[1][n1/2-1][n1/2][n1/2], X[2][n1/2-1][n1/2][n1/2],
            X[0][n1/2][n1/2][n1/2],   X[1][n1/2][n1/2][n1/2],   X[2][n1/2][n1/2][n1/2],
            X[0][n1/2+1][n1/2][n1/2], X[1][n1/2+1][n1/2][n1/2], X[2][n1/2+1][n1/2][n1/2]);
}

