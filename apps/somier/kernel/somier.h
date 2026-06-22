/*************************************************************************
* Somier - RISC-V Vectorized version
* Author: Jesus Labarta
* Barcelona Supercomputing Center
*************************************************************************/

extern double M;
extern double dt;
extern double spring_K;
extern double Xcenter[3];

extern void init_X (int n1, int n2, double (*X)[n1][n1][n2]);

extern void compute_forces(int n1, int n2, double (*X)[n1][n1][n2], double (*F)[n1][n1][n2]);
extern void compute_forces_prevec(int n1, int n2, double (*X)[n1][n1][n2], double (*F)[n1][n1][n2]);

extern void acceleration(int n1, int n2, double (*A)[n1][n1][n2], double (*F)[n1][n1][n2], double M);
extern void velocities(int n1, int n2, double (*V)[n1][n1][n2], double (*A)[n1][n1][n2], double dt);
extern void positions(int n1, int n2, double (*X)[n1][n1][n2], double (*V)[n1][n1][n2], double dt);

extern void accel_intr(int n1, int n2, double (*A)[n1][n1][n2], double (*F)[n1][n1][n2], double M);
extern void vel_intr(int n1, int n2, double (*V)[n1][n1][n2], double (*A)[n1][n1][n2], double dt);
extern void pos_intr(int n1, int n2, double (*X)[n1][n1][n2], double (*V)[n1][n1][n2], double dt);

extern void compute_stats(int n1, int n2, double (*X)[n1][n1][n2], double Xcenter[3]);
