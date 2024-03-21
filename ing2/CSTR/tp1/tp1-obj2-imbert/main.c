#include <stdio.h>
#include "sum2_c/sum2.h"

void main(void) {
    int i;
    Sum2__sum2_out o;
    Sum2__sum2_mem s;
    Sum2__sum2_reset(&s);

    for(;;){
        printf("Inputs:"); scanf("%d", &i);
        Sum2__sum2_step(i, &o,&s);
        printf("Result: o=%d\n", o.o);
    }

}
