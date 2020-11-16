obj-m    += freq.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
	$(CC) -Os -Wall  main.c -o main

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
	rm -f main