typedef struct { }       Externc__log_f_out;
typedef struct { }       Externc__log_gnc_out;
typedef struct { }              Externc__log_thermal_out;

void Externc__log_f_step(int idx, int i, int o, Externc__log_f_out* out);
void Externc__log_thermal_step(int idx, Externc__log_thermal_out* out);
void Externc__log_gnc_step(int idx, int i, int o , Externc__log_gnc_out* out);
