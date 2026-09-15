#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "printf.h"
#include "particlefilter.h"

#ifdef USE_RISCV_VECTOR
#include "../common/rivec/vector_defines.h"
#endif

#include "../common/macros/vector/vector_macros.h"
#include "../common/macros/vector/long_array.h"

static volatile uint64_t Mask[48] __attribute__((aligned(AXI_DWIDTH))) = {
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0
};

double findIndex(double *CDF, int lengthCDF, double value)
{
    double index = -1;

    for(int x = 0; x < lengthCDF; x++)
    {
        if(CDF[x] >= value)
        {
            index = x;
            break;
        }
    }

    if(index == -1)
        return lengthCDF - 1;

    return index;
}

void particleFilter(double *CDF, double *u, uint64_t *locations, int Nparticles)
{
    int x;
    int j;


    for(j = 0; j < Nparticles; j++)
    {
        locations[j] = findIndex(CDF, Nparticles, u[j]);

        if(locations[j] == -1)
            locations[j] = Nparticles - 1;
    }

}

void particleFilter_vec(double *CDF, double *u, uint64_t *locations, int Nparticles)
{
    long int vector_complete;
    long int valid = 0;

    int gvl;

    for(int i = 0; i < Nparticles - 1; ){

        asm volatile("vsetvli %0, %1, e64, m1, ta, ma" : "=r"(gvl) : "r"(Nparticles-i));
        vector_complete             = 0;    
        asm volatile ("vlm.v v2, (%0)"::"r"(&Mask[0]));                 // xMask, initally 0
        asm volatile ("vmv.v.x v3, %0" :: "r"(Nparticles-1));           // xArray, every lane has the maximum index as default value
        asm volatile ("vle64.v v4, (%0)"::"r"(&u[i]));                  // u, load the random numbers
 
        for(int j = 0; j < Nparticles; j++){    
            asm volatile("fld ft0, (%0)" :: "r"(&CDF[j]) : "ft0" );     // Load one value from the distribution
            asm volatile("vfmv.v.f v5, ft0");                           // Write this scalar value to all elements of the vector
            asm volatile ("vmfge.vv v6, v5, v4");                       // xComp: CDF[j] >= u[i] 
            asm volatile ("vmxor.mm v6, v2, v6");                       // xComp ^ xMask 
            asm volatile ("vfirst.m %[idx], v6" : [idx] "=r"(valid));   // check if there was an entry where CDF >= u
            
            if(valid != -1)
            {
                asm volatile("vmv.v.x v7, %[j]" :: [j] "r"(j));
                asm volatile("vmv.v.v v0, v6"); 
                asm volatile("vmerge.vvm v3, v3, v7, v0");              // update only the entries where CDF >= u   
                asm volatile("vmor.mm v2, v6, v2");                     // update the mask
                asm volatile("vcpop.m %[cnt], v2" : [cnt] "=r"(vector_complete));       // check how many particles did already belong to a lane
            }
            if(vector_complete == gvl)
            { 
                break; 
            }
        }
        asm volatile ("vse64.v v3, (%0)"::"r"(&locations[i]));          // locations 
        
        i = i + gvl;   
    }
}