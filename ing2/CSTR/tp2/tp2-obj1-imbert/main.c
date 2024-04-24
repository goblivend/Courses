#include <stdio.h>
#include "rcounter_c/rcounter.h"

void main(void) {
    int x, y;
    Rcounter__rcounter_out o;
    Rcounter__rcounter_mem s;
    Rcounter__rcounter_reset(&s);
    for(;;) {
        printf("Inputs:"); scanf("%d", &x);
        Rcounter__rcounter_step(x, &o, &s) ;
        printf("Result: o=%d\n", o.cnt);
    }

}
