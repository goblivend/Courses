#include "externc.h"
#include <stdio.h>

void Externc__log_f_step(int idx, int i, int o, Externc__log_f_out* out) {
    printf("Fast %d (%d) = %d\n", idx, i, o);
}

void Externc__log_thermal_step(int idx, Externc__log_thermal_out* out) {
    printf("Thermal %d\n", idx);
}

void Externc__log_gnc_step(int idx, int i, int o , Externc__log_gnc_out* out) {
    printf("Gnc %d (%d) = %d\n", idx, i, o);
}
