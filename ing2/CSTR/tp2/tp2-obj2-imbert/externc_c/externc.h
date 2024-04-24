typedef struct { int value; }   Externc__read_bool_out;
typedef struct { int o; }       Externc__f1_out;
typedef struct { int o; }       Externc__f2_out;
typedef struct { int o; }       Externc__g_out;
typedef struct { }              Externc__act_out;

void Externc__read_bool_step(int addr, Externc__read_bool_out* out);
void Externc__f1_step(int i, Externc__f1_out* out);
void Externc__f2_step(int i, Externc__f2_out* out);
void Externc__g_step(Externc__g_out* out);
void Externc__act_step(int addr, Externc__act_out* out);
