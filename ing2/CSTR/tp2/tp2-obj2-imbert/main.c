#include <stdio.h>
#include "main_c/main.h"

void main(void) {
    int fs, id;
    Main__main_out o;
    Main__main_mem s;
    Main__main_reset(&s);
    for(;;) {
        // printf("Inputs:"); scanf("%d%d", &fs, &id);
        Main__main_step(&o, &s) ;
    }

}
