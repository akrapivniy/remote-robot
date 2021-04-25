CC      = sdcc
LD      = sdcc

CFLAGS		= -mstm8 --code-size 8192 -I../stm8-lib/
LDFLAGS		= -lstm8 --code-size 8192 -I../stm8-lib/
SOURCES = nrf24 i2c main ssd1306
OBJECTS=$(SOURCES:=.rel)


all: full

full: $(SOURCES)
	@ echo "[Linking]"
	$(LD)  -lstm8 -mstm8 --code-size 8192 --iram-size 1024 --out-fmt-ihx $(OBJECTS) -o main.ihx


.c:  $(OBJECTS)
	@ echo "[Compiling main code] "
	$(CC) -lstm8 -mstm8 --opt-code-size -I../stm8-lib/ -c $< -o $@.rel

clean:
	rm -f $(OBJECTS) *.hex *.map *.cdb *.ihx *.lk *.lst *.rel *.rst *.sym *.asm

prog: test
	./stm8flash -cstlink -pstm8l051f3 -w $(OBJECT).ihx

