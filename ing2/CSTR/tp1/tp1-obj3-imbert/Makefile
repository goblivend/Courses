CFLAGS= -Wno-unused-result
FILE=counter

all: ${FILE}.ept main.c
	heptc -target c ${FILE}.ept
	gcc $(CFLAGS) -I `heptc -where`/c main.c ${FILE}_c/${FILE}.c -o main

clean:
	${RM} -r main ${FILE}.epci ${FILE}.log ${FILE}.mls ${FILE}_c ${FILE}.obc
