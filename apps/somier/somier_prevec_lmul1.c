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
#include "kernel/somier.h"
//#include "somier_v.h"

#ifdef USE_RISCV_VECTOR
#define NR_CLUSTERS          16
#include "../common/rivec/vector_defines.h"
#endif

#include "printf.h"

static int count=0;

//reference scalar code
void force_contr_prevec(int n, double (*X)[n][n][n], double (*F)[n][n][n], int i, int j, int neig_i, int neig_j)
{
   double dx, dy, dz, dl, spring_F, FX, FY,FZ;

   for (int k=1; k<n-1; k++) {

      dx=X[0][neig_i][neig_j][k]-X[0][i][j][k];
      dy=X[1][neig_i][neig_j][k]-X[1][i][j][k];
      dz=X[2][neig_i][neig_j][k]-X[2][i][j][k];
      dl = sqrt(dx*dx + dy*dy + dz*dz);
      spring_F = 0.25 * spring_K*(dl-1);
      FX = spring_F * dx/dl;
      FY = spring_F * dy/dl;
      FZ = spring_F * dz/dl;
      F[0][i][j][k] += FX;
      F[1][i][j][k] += FY;
      F[2][i][j][k] += FZ;
   }
}

#ifdef USE_RISCV_VECTOR
inline void force_contr_vec(int n1, int n2, double (*X)[n1][n1][n2], double (*F)[n1][n1][n2], int i, int j, int neig_i, int neig_j);
void force_contr_vec(int n1, int n2, double (*X)[n1][n1][n2], double (*F)[n1][n1][n2], int i, int j, int neig_i, int neig_j)

{

   unsigned long gvl = _MMR_VSETVL_E64M1(n2);

   _MMR_f64 v_1        = _MM_SET_f64(1.0, gvl);
   _MMR_f64 v_spr_K    = _MM_SET_f64(0.25*spring_K, gvl);

   _MMR_f64 v_FX_up    = _MM_SET_f64(0.0, gvl);    // initialization
   _MMR_f64 v_FY_up    = _MM_SET_f64(0.0, gvl);
   _MMR_f64 v_FZ_up    = _MM_SET_f64(0.0, gvl);

   for (int k=1; k<n2-1; ) {

      // Loaded F needs to be larger than actual vl which is used for computing
      // otherwise the top element would be lost during slideup
      if (n2 > NR_CLUSTERS * 32)              // This benchmark works on 64-bit elements
      {                                       // Due to edge elements only half of the possible vl is used
         gvl = (n2 - k > NR_CLUSTERS * 32) ? NR_CLUSTERS * 32 : n2 - k;
      }
      else
      {
         gvl = n2 - k;
      }

      gvl              = _MMR_VSETVL_E64M1(gvl);
      _MMR_f64 v_FX    = _MM_LOAD_f64(&F[0][i][j][k], gvl);
      _MMR_f64 v_FY    = _MM_LOAD_f64(&F[1][i][j][k], gvl);
      _MMR_f64 v_FZ    = _MM_LOAD_f64(&F[2][i][j][k], gvl);

      // Load the data on which is worked on
      if (k == 1 || (n2 - k <= NR_CLUSTERS * 32))
      {
         gvl = _MMR_VSETVL_E64M1(gvl - 1);
      }

      _MMR_f64 v_x1    = _MM_LOAD_f64(&X[0][neig_i][neig_j][k], gvl);
      _MMR_f64 v_x2    = _MM_LOAD_f64(&X[0][i][j][k], gvl);
      _MMR_f64 v_x3    = _MM_LOAD_f64(&X[1][neig_i][neig_j][k], gvl);
      _MMR_f64 v_x4    = _MM_LOAD_f64(&X[1][i][j][k], gvl);
      _MMR_f64 v_x5    = _MM_LOAD_f64(&X[2][neig_i][neig_j][k], gvl);
      _MMR_f64 v_x6    = _MM_LOAD_f64(&X[2][i][j][k], gvl);   

      _MMR_f64 v_dx    = _MM_SUB_f64(v_x1, v_x2, gvl);
      _MMR_f64 v_dy    = _MM_SUB_f64(v_x3, v_x4, gvl);
      _MMR_f64 v_dz    = _MM_SUB_f64(v_x5, v_x6, gvl);

      _MMR_f64 v_dx2   = _MM_MUL_f64(v_dx, v_dx, gvl);
      _MMR_f64 v_dy2   = _MM_MUL_f64(v_dy, v_dy, gvl);
      _MMR_f64 v_dz2   = _MM_MUL_f64(v_dz, v_dz, gvl);

      _MMR_f64 v_dl    = _MM_ADD_f64(v_dx2, v_dy2, gvl);
      v_dl             = _MM_ADD_f64(v_dl, v_dz2, gvl);
      v_dl             = _MM_SQRT_f64(v_dl, gvl);
      _MMR_f64 v_dl1   = _MM_SUB_f64(v_dl, v_1, gvl);
      _MMR_f64 v_spr_F = _MM_MUL_f64(v_spr_K, v_dl1, gvl);

      _MMR_f64 v_dFX   = _MM_DIV_f64(v_dx, v_dl, gvl);
      _MMR_f64 v_dFY   = _MM_DIV_f64(v_dy, v_dl, gvl);
      _MMR_f64 v_dFZ   = _MM_DIV_f64(v_dz, v_dl, gvl);

      v_FX             = _MM_MACC_f64(v_FX, v_spr_F, v_dFX,  gvl);
      v_FY             = _MM_MACC_f64(v_FY, v_spr_F, v_dFY,  gvl);
      v_FZ             = _MM_MACC_f64(v_FZ, v_spr_F, v_dFZ,  gvl); 

      if (k == 1)
      {
         // Necessary to ensure correct data layout and aligned store
         gvl           = _MMR_VSETVL_E64M1(gvl + 1);
         v_FX_up       = _MM_VSLIDEUP_f64(v_FX_up, v_FX, 1, gvl);
         v_FY_up       = _MM_VSLIDEUP_f64(v_FY_up, v_FY, 1, gvl);
         v_FZ_up       = _MM_VSLIDEUP_f64(v_FZ_up, v_FZ, 1, gvl);    
         _MM_STORE_f64(&F[0][i][j][k-1], v_FX_up, gvl);
         _MM_STORE_f64(&F[1][i][j][k-1], v_FY_up, gvl);
         _MM_STORE_f64(&F[2][i][j][k-1], v_FZ_up, gvl);
         k += gvl - 1;
      } 
      else
      {
         _MM_STORE_f64(&F[0][i][j][k], v_FX, gvl);
         _MM_STORE_f64(&F[1][i][j][k], v_FY, gvl);
         _MM_STORE_f64(&F[2][i][j][k], v_FZ, gvl);
         k += gvl;
      }
   }
}


inline void k_minus_force_contr_vec(int n1, int n2, double (*X)[n1][n1][n2], double (*F)[n1][n1][n2], int i, int j);
void k_minus_force_contr_vec(int n1, int n2, double (*X)[n1][n1][n2], double (*F)[n1][n1][n2], int i, int j)
{
   long gvl = _MMR_VSETVL_E64M1(n2);

   _MMR_f64 v_xs, v_ys, v_zs, v_FX, v_FY, v_FZ, v_FX_up, v_FY_up, v_FZ_up;

   _MMR_f64 v_1        = _MM_SET_f64(1.0, gvl);
   _MMR_f64 v_spr_K    = _MM_SET_f64(0.25*spring_K, gvl);

   // _MMR_f64 v_FX_up    = _MM_SET_f64(0.0, gvl);    // initialization
   // _MMR_f64 v_FY_up    = _MM_SET_f64(0.0, gvl);
   // _MMR_f64 v_FZ_up    = _MM_SET_f64(0.0, gvl);

   for (int k=0; k<n2-2; ) {

      if (n2 - k >= NR_CLUSTERS * 32)
      {
         gvl = (n2 - k > NR_CLUSTERS * 32) ? NR_CLUSTERS * 32 + 1 : NR_CLUSTERS * 32;
      }
      else
      {
         gvl = n2 - k;
      }

      if (k != 0) 
      {
         gvl += 1;
      }

      gvl = _MMR_VSETVL_E64M1(gvl);

      if (k == 0)
      {
         v_xs   = _MM_LOAD_f64(&X[0][i][j][k], gvl);
         v_ys   = _MM_LOAD_f64(&X[1][i][j][k], gvl);
         v_zs   = _MM_LOAD_f64(&X[2][i][j][k], gvl);
      }
      else
      {
         v_xs   = _MM_LOAD_f64(&X[0][i][j][k-1], gvl);
         v_ys   = _MM_LOAD_f64(&X[1][i][j][k-1], gvl);
         v_zs   = _MM_LOAD_f64(&X[2][i][j][k-1], gvl);
      }

      gvl = _MMR_VSETVL_E64M1(gvl - 2);

      if (k == 0)
      {
         v_FX   = _MM_LOAD_f64(&F[0][i][j][k+1], gvl);
         v_FY   = _MM_LOAD_f64(&F[1][i][j][k+1], gvl);
         v_FZ   = _MM_LOAD_f64(&F[2][i][j][k+1], gvl);

         v_FX_up    = _MM_LOAD_f64(&F[0][i][j][k], gvl);
         v_FY_up    = _MM_LOAD_f64(&F[1][i][j][k], gvl);
         v_FZ_up    = _MM_LOAD_f64(&F[2][i][j][k], gvl);
      }
      else
      {
         v_FX   = _MM_LOAD_f64(&F[0][i][j][k], gvl);
         v_FY   = _MM_LOAD_f64(&F[1][i][j][k], gvl);
         v_FZ   = _MM_LOAD_f64(&F[2][i][j][k], gvl);
      }

      _MMR_f64 v_x     = _MM_VSLIDEDOWN_f64(v_xs, 1, gvl);
      _MMR_f64 v_y     = _MM_VSLIDEDOWN_f64(v_ys, 1, gvl);
      _MMR_f64 v_z     = _MM_VSLIDEDOWN_f64(v_zs, 1, gvl);

      _MMR_f64 v_dx    = _MM_SUB_f64(v_xs, v_x, gvl);
      _MMR_f64 v_dy    = _MM_SUB_f64(v_ys, v_y, gvl);
      _MMR_f64 v_dz    = _MM_SUB_f64(v_zs, v_z, gvl);

      _MMR_f64 v_dy2   = _MM_MUL_f64(v_dy, v_dy, gvl);
      _MMR_f64 v_dx2   = _MM_MUL_f64(v_dx, v_dx, gvl);
      _MMR_f64 v_dz2   = _MM_MUL_f64(v_dz, v_dz, gvl);

      _MMR_f64 v_dl    = _MM_ADD_f64(v_dx2, v_dy2, gvl);
      v_dl             = _MM_ADD_f64(v_dl, v_dz2, gvl);
      v_dl             = _MM_SQRT_f64(v_dl, gvl);
      _MMR_f64 v_dl1   = _MM_SUB_f64(v_dl, v_1, gvl);
      _MMR_f64 v_spr_F = _MM_MUL_f64(v_spr_K, v_dl1, gvl);

      _MMR_f64 v_dFX   = _MM_DIV_f64(v_dx, v_dl, gvl);
      _MMR_f64 v_dFY   = _MM_DIV_f64(v_dy, v_dl, gvl);
      _MMR_f64 v_dFZ   = _MM_DIV_f64(v_dz, v_dl, gvl);

      v_FX             = _MM_MACC_f64(v_FX, v_spr_F, v_dFX, gvl);
      v_FY             = _MM_MACC_f64(v_FY, v_spr_F, v_dFY, gvl);
      v_FZ             = _MM_MACC_f64(v_FZ, v_spr_F, v_dFZ, gvl);

      // NOTE: when doing this part with vector execution DIV produces a NaN result.
      v_xs             = _MM_VSLIDEDOWN_f64(v_xs, 2, gvl);
      v_ys             = _MM_VSLIDEDOWN_f64(v_ys, 2, gvl);
      v_zs             = _MM_VSLIDEDOWN_f64(v_zs, 2, gvl);

      v_dx             = _MM_SUB_f64(v_xs, v_x, gvl);
      v_dz             = _MM_SUB_f64(v_zs, v_z, gvl);
      v_dy             = _MM_SUB_f64(v_ys, v_y, gvl);

      v_dy2            = _MM_MUL_f64(v_dy, v_dy, gvl);
      v_dx2            = _MM_MUL_f64(v_dx, v_dx, gvl);
      v_dz2            = _MM_MUL_f64(v_dz, v_dz, gvl);

      v_dl             = _MM_ADD_f64(v_dx2, v_dy2, gvl);
      v_dl             = _MM_ADD_f64(v_dl, v_dz2, gvl);
      v_dl             = _MM_SQRT_f64(v_dl, gvl);
      v_dl1            = _MM_SUB_f64(v_dl, v_1, gvl);
      v_spr_F          = _MM_MUL_f64(v_spr_K, v_dl1, gvl);

      v_dFX            = _MM_DIV_f64(v_dx, v_dl, gvl);
      v_dFY            = _MM_DIV_f64(v_dy, v_dl, gvl);
      v_dFZ            = _MM_DIV_f64(v_dz, v_dl, gvl);

      v_FX             = _MM_MACC_f64(v_FX, v_spr_F, v_dFX, gvl);
      v_FY             = _MM_MACC_f64(v_FY, v_spr_F, v_dFY, gvl);
      v_FZ             = _MM_MACC_f64(v_FZ, v_spr_F, v_dFZ, gvl);

      // Necessary to ensure correct data layout and aligned store
      if (k == 0)
      {
         gvl              = _MMR_VSETVL_E64M1(gvl+1);
         v_FX_up          = _MM_VSLIDEUP_f64(v_FX_up, v_FX, 1, gvl);
         v_FY_up          = _MM_VSLIDEUP_f64(v_FY_up, v_FY, 1, gvl);
         v_FZ_up          = _MM_VSLIDEUP_f64(v_FZ_up, v_FZ, 1, gvl);

         _MM_STORE_f64(&F[0][i][j][k], v_FX_up, gvl);
         _MM_STORE_f64(&F[1][i][j][k], v_FY_up, gvl);
         _MM_STORE_f64(&F[2][i][j][k], v_FZ_up, gvl);
      }
      else 
      {
         _MM_STORE_f64(&F[0][i][j][k], v_FX, gvl);
         _MM_STORE_f64(&F[1][i][j][k], v_FY, gvl);
         _MM_STORE_f64(&F[2][i][j][k], v_FZ, gvl);
      }

      k += NR_CLUSTERS * 32;

   }
}

inline void k_plus_force_contr_vec(int n1, int n2, double (*X)[n1][n1][n2], double (*F)[n1][n1][n2], int i, int j);
void k_plus_force_contr_vec(int n1, int n2, double (*X)[n1][n1][n2], double (*F)[n1][n1][n2], int i, int j)
{
   long gvl = _MMR_VSETVL_E64M1(n2);

   _MMR_f64 v_xs, v_ys, v_zs, v_FX, v_FY, v_FZ, v_FX_up, v_FY_up, v_FZ_up;

   _MMR_f64 v_1        = _MM_SET_f64(1.0, gvl);
   _MMR_f64 v_spr_K    = _MM_SET_f64(0.25*spring_K, gvl);

   // _MMR_f64 v_FX_up    = _MM_SET_f64(0.0, gvl);    // initialization
   // _MMR_f64 v_FY_up    = _MM_SET_f64(0.0, gvl);
   // _MMR_f64 v_FZ_up    = _MM_SET_f64(0.0, gvl);

   for (int k=0; k<n2-2; ) {

      if (n2 - k >= NR_CLUSTERS * 32)
      {
         gvl = (n2 - k > NR_CLUSTERS * 32) ? NR_CLUSTERS * 32 + 1 : NR_CLUSTERS * 32;
      }
      else
      {
         gvl = n2 - k;
      }

      if (k != 0) 
      {
         gvl += 1;
      }

      gvl = _MMR_VSETVL_E64M1(gvl);

      if (k == 0)
      {
         v_xs   = _MM_LOAD_f64(&X[0][i][j][k], gvl);
         v_ys   = _MM_LOAD_f64(&X[1][i][j][k], gvl);
         v_zs   = _MM_LOAD_f64(&X[2][i][j][k], gvl);
      }
      else
      {
         v_xs   = _MM_LOAD_f64(&X[0][i][j][k-1], gvl);
         v_ys   = _MM_LOAD_f64(&X[1][i][j][k-1], gvl);
         v_zs   = _MM_LOAD_f64(&X[2][i][j][k-1], gvl);
      }

      gvl = _MMR_VSETVL_E64M1(gvl - 2);

      if (k == 0)
      {
         v_FX   = _MM_LOAD_f64(&F[0][i][j][k+1], gvl);
         v_FY   = _MM_LOAD_f64(&F[1][i][j][k+1], gvl);
         v_FZ   = _MM_LOAD_f64(&F[2][i][j][k+1], gvl);

         v_FX_up    = _MM_LOAD_f64(&F[0][i][j][k], gvl);
         v_FY_up    = _MM_LOAD_f64(&F[1][i][j][k], gvl);
         v_FZ_up    = _MM_LOAD_f64(&F[2][i][j][k], gvl);
      }
      else
      {
         v_FX   = _MM_LOAD_f64(&F[0][i][j][k], gvl);
         v_FY   = _MM_LOAD_f64(&F[1][i][j][k], gvl);
         v_FZ   = _MM_LOAD_f64(&F[2][i][j][k], gvl);
      }



      _MMR_f64 v_x     = _MM_VSLIDEDOWN_f64(v_xs, 2, gvl);
      _MMR_f64 v_y     = _MM_VSLIDEDOWN_f64(v_ys, 2, gvl);
      _MMR_f64 v_z     = _MM_VSLIDEDOWN_f64(v_zs, 2, gvl);

      _MMR_f64 v_xs     = _MM_VSLIDEDOWN_f64(v_xs, 1, gvl);
      _MMR_f64 v_ys     = _MM_VSLIDEDOWN_f64(v_ys, 1, gvl);
      _MMR_f64 v_zs     = _MM_VSLIDEDOWN_f64(v_zs, 1, gvl);

      _MMR_f64 v_dx    = _MM_SUB_f64(v_xs, v_x, gvl);
      _MMR_f64 v_dy    = _MM_SUB_f64(v_ys, v_y, gvl);
      _MMR_f64 v_dz    = _MM_SUB_f64(v_zs, v_z, gvl);

      _MMR_f64 v_dy2   = _MM_MUL_f64(v_dy, v_dy, gvl);
      _MMR_f64 v_dx2   = _MM_MUL_f64(v_dx, v_dx, gvl);
      _MMR_f64 v_dz2   = _MM_MUL_f64(v_dz, v_dz, gvl);

      _MMR_f64 v_dl    = _MM_ADD_f64(v_dx2, v_dy2, gvl);
      v_dl             = _MM_ADD_f64(v_dl, v_dz2, gvl);
      v_dl             = _MM_SQRT_f64(v_dl, gvl);
      _MMR_f64 v_dl1   = _MM_SUB_f64(v_dl, v_1, gvl);
      _MMR_f64 v_spr_F = _MM_MUL_f64(v_spr_K, v_dl1, gvl);

      _MMR_f64 v_dFX   = _MM_DIV_f64(v_dx, v_dl, gvl);
      _MMR_f64 v_dFY   = _MM_DIV_f64(v_dy, v_dl, gvl);
      _MMR_f64 v_dFZ   = _MM_DIV_f64(v_dz, v_dl, gvl);

      v_FX             = _MM_MACC_f64(v_FX, v_spr_F, v_dFX, gvl);
      v_FY             = _MM_MACC_f64(v_FY, v_spr_F, v_dFY, gvl);
      v_FZ             = _MM_MACC_f64(v_FZ, v_spr_F, v_dFZ, gvl);

      // NOTE: when doing this part with vector execution DIV produces a NaN result.
      // v_xs             = _MM_VSLIDEDOWN_f64(v_xs, 2, gvl);
      // v_ys             = _MM_VSLIDEDOWN_f64(v_ys, 2, gvl);
      // v_zs             = _MM_VSLIDEDOWN_f64(v_zs, 2, gvl);

      // v_dx             = _MM_SUB_f64(v_xs, v_x, gvl);
      // v_dz             = _MM_SUB_f64(v_zs, v_z, gvl);
      // v_dy             = _MM_SUB_f64(v_ys, v_y, gvl);

      // v_dy2            = _MM_MUL_f64(v_dy, v_dy, gvl);
      // v_dx2            = _MM_MUL_f64(v_dx, v_dx, gvl);
      // v_dz2            = _MM_MUL_f64(v_dz, v_dz, gvl);

      // v_dl             = _MM_ADD_f64(v_dx2, v_dy2, gvl);
      // v_dl             = _MM_ADD_f64(v_dl, v_dz2, gvl);
      // v_dl             = _MM_SQRT_f64(v_dl, gvl);
      // v_dl1            = _MM_SUB_f64(v_dl, v_1, gvl);
      // v_spr_F          = _MM_MUL_f64(v_spr_K, v_dl1, gvl);

      // v_dFX            = _MM_DIV_f64(v_dx, v_dl, gvl);
      // v_dFY            = _MM_DIV_f64(v_dy, v_dl, gvl);
      // v_dFZ            = _MM_DIV_f64(v_dz, v_dl, gvl);

      // v_FX             = _MM_MACC_f64(v_FX, v_spr_F, v_dFX, gvl);
      // v_FY             = _MM_MACC_f64(v_FY, v_spr_F, v_dFY, gvl);
      // v_FZ             = _MM_MACC_f64(v_FZ, v_spr_F, v_dFZ, gvl);

      // Necessary to ensure correct data layout and aligned store
      if (k == 0)
      {
         gvl              = _MMR_VSETVL_E64M1(gvl+1);
         v_FX_up          = _MM_VSLIDEUP_f64(v_FX_up, v_FX, 1, gvl);
         v_FY_up          = _MM_VSLIDEUP_f64(v_FY_up, v_FY, 1, gvl);
         v_FZ_up          = _MM_VSLIDEUP_f64(v_FZ_up, v_FZ, 1, gvl);

         _MM_STORE_f64(&F[0][i][j][k], v_FX_up, gvl);
         _MM_STORE_f64(&F[1][i][j][k], v_FY_up, gvl);
         _MM_STORE_f64(&F[2][i][j][k], v_FZ_up, gvl);
      }
      else 
      {
         _MM_STORE_f64(&F[0][i][j][k], v_FX, gvl);
         _MM_STORE_f64(&F[1][i][j][k], v_FY, gvl);
         _MM_STORE_f64(&F[2][i][j][k], v_FZ, gvl);
      }

      k += NR_CLUSTERS * 32;

   }
}

#endif

void k_force_contr_prevec(int n1, int n2, double (*X)[n1][n1][n2], double (*F)[n1][n1][n2], int i, int j)
{
   double dx, dy, dz, dl, spring_F, FX, FY,FZ;

   for (int k=1; k<n2-1; k++) {
      // k-1
      dx=X[0][i][j][k-1]-X[0][i][j][k];
      dy=X[1][i][j][k-1]-X[1][i][j][k];
      dz=X[2][i][j][k-1]-X[2][i][j][k];
      dl = sqrt(dx*dx + dy*dy + dz*dz);
      spring_F = 0.25 * spring_K*(dl-1);
      FX = spring_F * dx/dl;
      FY = spring_F * dy/dl;
      FZ = spring_F * dz/dl;
      F[0][i][j][k] += FX;
      F[1][i][j][k] += FY;
      F[2][i][j][k] += FZ;

      // k+1
      dx=X[0][i][j][k+1]-X[0][i][j][k];
      dy=X[1][i][j][k+1]-X[1][i][j][k];
      dz=X[2][i][j][k+1]-X[2][i][j][k];
      dl = sqrt(dx*dx + dy*dy + dz*dz);
      spring_F = 0.25 * spring_K*(dl-1);
      FX = spring_F * dx/dl;
      FY = spring_F * dy/dl;
      FZ = spring_F * dz/dl;
      F[0][i][j][k] += FX;
      F[1][i][j][k] += FY;
      F[2][i][j][k] += FZ;
   }
}

__attribute__((noinline)) void emit_event()
{
   __asm__("vadd.vi v0,v0,0");
}


#ifdef USE_RISCV_VECTOR
void compute_forces_prevec(int n1, int n2, double (*X)[n1][n1][n2], double (*F)[n1][n1][n2])
{
   for (int i=1; i<n1-1; i++) {
      for (int j=1; j<n1-1; j++) {
            force_contr_vec (n1, n2, X, F, i, j, i-1, j  ); 
            force_contr_vec (n1, n2, X, F, i, j, i+1, j  );
            force_contr_vec (n1, n2, X, F, i, j, i,   j-1);
            force_contr_vec (n1, n2, X, F, i, j, i,   j+1);
            k_minus_force_contr_vec (n1, n2, X, F, i, j);            // calculates k-1
            //k_plus_force_contr_vec (n1, n2, X, F, i, j);            // calculates k+1

            //k_force_contr_prevec(n1, n2, X, F, i, j);          // calculates k+1 (scalar execution => need to uncomment the k-1 part in "k_force_contr_prevec" )
      }
   }
}
#endif
