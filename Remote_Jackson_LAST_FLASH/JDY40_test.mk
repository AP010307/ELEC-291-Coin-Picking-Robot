SHELL=cmd
OBJS=JDY40_test.o usart.o Software_UART.o
PORTN=$(shell type COMPORT.inc)

JDY40_test.elf: $(OBJS)
	avr-gcc -mmcu=atmega328 -Wl,-Map,JDY40_test.map $(OBJS) -o JDY40_test.elf
	avr-objcopy -j .text -j .data -O ihex JDY40_test.elf JDY40_test.hex
	@echo done!
	
JDY40_test.o: JDY40_test.c usart.h Software_UART.h
	avr-gcc -g -Os -mmcu=atmega328 -c JDY40_test.c

usart.o: usart.c usart.h
	avr-gcc -g -Os -Wall -mmcu=atmega328p -c usart.c

Software_UART.o: Software_UART.c Software_UART.h
	avr-gcc -g -Os -Wall -mmcu=atmega328p -c Software_UART.c

clean:
	@del *.hex *.elf *.o 2>nul

FlashLoad:
	@taskkill /f /im putty.exe /t /fi "status eq running" > NUL
	spi_atmega -CRYSTAL -p -v JDY40_test.hex
	@cmd /c start putty.exe -serial $(PORTN) -sercfg 115200,8,n,1,N

putty:
	@taskkill /f /im putty.exe /t /fi "status eq running" > NUL
	@cmd /c start putty.exe -serial $(PORTN) -sercfg 115200,8,n,1,N

dummy: JDY40_test.hex JDY40_test.map
	@echo Hello from dummy!
	