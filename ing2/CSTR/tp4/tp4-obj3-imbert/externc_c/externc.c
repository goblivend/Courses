#include "externc.h"
#include <stdio.h>
#include <stdlib.h>

void Externc__deadline_miss_log_step(int date, int task_id, Externc__deadline_miss_log_out* out) {
    printf("Missed Deadline date = %d, task = %d\n", date, task_id);
}

void Externc__random_step(int max, Externc__random_out* out) {
    out->v = rand() % max + 1;
}

void Externc__print_scheduler_state_step(Scheduler_data__scheduler_state s , Externc__print_scheduler_state_out* out) {
    char **statuses = (char *[]){"Running", "Ready", "Waiting"};

    puts("Scheduler State: {");
    printf("\tcurrent_date = %d\n", s.current_date);
    puts("\ttasks = {");
    for (int i = 0; i < Scheduler_data__ntasks; i++) {
        printf("\t\t%d : %s deadline = %d, time left= %d\n", i, statuses[s.tasks[i].status], s.tasks[i].current_deadline, s.tasks[i].left);
    }
    puts("\t}\n}");
}

//   Scheduler_data__task_state status;
//   int current_deadline;
//   int left;
