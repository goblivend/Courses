#include <stdio.h>
#include <unistd.h>
#include "counter_c/counter.h"

void main(void) {
    Counter__counter_out o;
    Counter__counter_mem s;
    Counter__counter_reset(&s);

    for(;;){
        usleep(1000000);
        printf("Inputs:");
        Counter__counter_step(&o,&s);
        printf("Result: o=%d\n", o.cnt);
    }

}
