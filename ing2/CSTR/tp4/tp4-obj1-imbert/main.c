
#include <stdio.h>
#include "scheduler_c/scheduler.h"

void main(void) {
    char fs[5];
    Scheduler__main_out o;
    Scheduler__main_mem s;
    Scheduler__main_reset(&s);
    for(;;) {
        Scheduler__main_step(&o, &s) ;
        fgets(fs, 5, stdin);
    }

}
