CFLAGS= -Wno-unused-result
FILE=main

all: scheduler.ept main.c
	heptc -target c scheduler_data.ept
	heptc -c externc.epi
	heptc -target c scheduler.ept
	gcc $(CFLAGS) -I `heptc -where`/c -I externc_c -I scheduler_data_c scheduler_data_c/scheduler_data.c scheduler_c/scheduler.c main.c -o out externc_c/externc.c

clean:
	${RM} -r out *.epci *.log *.mls main_c scheduler_c scheduler_data_c *.obc
