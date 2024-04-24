#include "externc.h"

#include <stdio.h>

void Externc__read_bool_step(int addr, Externc__read_bool_out* out) {
    printf("read_bool(%d):", addr);
    fflush(stdout);
    scanf("%d", &(out->value));
}

void Externc__f1_step(int i, Externc__f1_out* out){
    out->o = i + 5;
    printf("F1(%d)=%d\n", i, out->o);
}

void Externc__f2_step(int i, Externc__f2_out* out){
    out->o = i + 100;
    printf("F2(%d)=%d\n", i, out->o);
}

void Externc__g_step(Externc__g_out* out){
    static int s = 300;
    out->o = s;
    s += 50;
    printf("G()=%d\n", out->o);
}

void Externc__act_step(int addr, Externc__act_out* out){
    printf("ACT(%d)\n", addr);
}
