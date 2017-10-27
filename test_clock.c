#include <stdio.h>
#include <time.h>
#include <stdint.h>

#define SEC2NANO 1000000000.0f

int main()
{
    struct timespec ts;
    struct timespec ts2;

    clock_gettime(CLOCK_MONOTONIC_RAW,&ts);
    double v1 = ts.tv_nsec ;
    double v2 = ts.tv_sec ;
    printf("%lu %lu\n", ts.tv_sec, ts.tv_nsec);;
    usleep(400000);
    clock_gettime(CLOCK_MONOTONIC_RAW,&ts2);
    printf("%lu %lu\n", ts2.tv_sec, ts2.tv_nsec);;
 
    double totaltime = ((ts2.tv_sec + ts2.tv_nsec/SEC2NANO) - (ts.tv_sec + ts.tv_nsec/SEC2NANO));
    printf("total %f\n", totaltime);


    return 0;
}
