#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

void particleFilter(double *CDF, double *u, uint64_t *locations, int Nparticles);

void particleFilter_vec(double *CDF, double *u, uint64_t *locations, int Nparticles);

#endif