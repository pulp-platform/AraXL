extern void capture_ref_result(double *y, double* y_ref, int n);
extern void print_4D (int n1, int n2, char*name, double (*y)[n1][n1][n2]);
extern void capture_4D_ref (int n1, int n2, double (*y)[n1][n1][n2], double (*y_ref)[n1][n1][n2]);
extern void test_4D_result(int n1, int n2, double (*y)[n1][n1][n2], double (*y_ref)[n1][n1][n2]);
extern void clear_4D(int n1, int n2, double (*X)[n1][n1][n2]);
extern void init_X (int n1, int n2, double (*X)[n1][n1][n2]);
extern void boundary(int n1, int n2, double (*X)[n1][n1][n2], double (*V)[n1][n1][n2]);
extern void force_contribution(int n1, int n2, double (*X)[n1][n1][n2], double (*F)[n1][n1][n2],
                   int i, int j, int k, int neig_i, int neig_j, int neig_k);
extern void compute_force_new(int n1, int n2, double (*X)[n1][n1][n2], double (*F)[n1][n1][n2]);
extern void print_state(int n1, int n2, double (*X)[n1][n1][n2], double Xcenter[3], int nt);

