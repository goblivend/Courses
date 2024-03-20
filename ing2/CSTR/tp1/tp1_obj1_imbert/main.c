#include <stdio.h>
#include "myfun_c/myfun.h"

void main(void) {
    int x, y;
    first__myfun_out o;
    for(;;) {
        printf("Inputs:"); scanf("%d%d", &x, &y);
        First__myfun_step(x, y, &o) ;
        printf("Result: z=%d t=%d\n", o.z, o.t);
    }

}
