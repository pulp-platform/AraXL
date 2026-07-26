#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

void particleFilter(double *weights, double *CDF, double *u, uint64_t *locations, int Nparticles);

void particleFilter_vec(double *weights, double *CDF, double *u, uint64_t *locations, int Nparticles);

#endif