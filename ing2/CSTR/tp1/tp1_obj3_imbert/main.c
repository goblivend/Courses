#include <stdio.h>
#include "counter_c/counter.h"

void main(void) {
    First__counter_out o;
    First__counter_mem s;
    First__counter_reset(&s);

    for(;;){
        usleep(1);
        printf("Inputs:");
        First__counter_step(&o,&s);
        printf("Result: o=%d\n", o.icnt);
    }

}
