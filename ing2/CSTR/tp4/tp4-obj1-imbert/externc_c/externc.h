#include "../scheduler_data_c/scheduler_data_types.h"


typedef struct { }        Externc__deadline_miss_log_out;
typedef struct { int v; } Externc__random_out;
typedef struct { }        Externc__print_scheduler_state_out;

void Externc__deadline_miss_log_step(int date, int task_id, Externc__deadline_miss_log_out* out);
void Externc__random_step(int max, Externc__random_out* out);
void Externc__print_scheduler_state_step(Scheduler_data__scheduler_state s , Externc__print_scheduler_state_out* out);
