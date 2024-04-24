CFLAGS= -Wno-unused-result
FILE=main

all: ${FILE}.ept main.c
	heptc -c externc.epi
	heptc -target c ${FILE}.ept
	gcc $(CFLAGS) -I `heptc -where`/c -I externc_c main.c ${FILE}_c/${FILE}.c -o out externc_c/externc.c

clean:
	${RM} -r out *.epci ${FILE}.log ${FILE}.mls ${FILE}_c ${FILE}.obc
