FILENAME     = shutters
DEVICE       = atmega32
F_CPU        = 16000000
PROGRAMMER   = usbasp
AVRGCCPATH   = "D:\avr-gcc-10.1.0\bin"
# CC           = avr-gcc -Wall -Os -mmcu=$(DEVICE)
CC           = avr-gcc -Wall -Os -mmcu=$(DEVICE) -DF_CPU=$(F_CPU)UL

OUT_PATH     = ../build
HEADER_PATH  = ../headers

HEADERS      = 
DEPENDENCIES = $(OUT_PATH)/$(FILENAME).o

_DEPS        =

$(OUT_PATH)/%.o: %.c
	@echo compiling "$<"
	$(AVRGCCPATH)/$(CC) -c -o $@ $< -I$(HEADER_PATH)
	@echo finished.

compile: $(DEPENDENCIES)
	$(AVRGCCPATH)/$(CC) $^ -o $(OUT_PATH)/$(FILENAME).elf
	$(AVRGCCPATH)/avr-objcopy -j .text -j .data -O ihex $(OUT_PATH)/$(FILENAME).elf $(OUT_PATH)/$(FILENAME).hex

	@echo ""
	$(AVRGCCPATH)/avr-size $(OUT_PATH)/$(FILENAME).elf
	@echo ""

upload:
	avrdude -p $(DEVICE) -c $(PROGRAMMER) -e -U flash:w:$(OUT_PATH)/$(FILENAME).hex:i

all: compile upload

build: clean compile upload

clean:
	rm "$(OUT_PATH)/"*.o
	rm $(OUT_PATH)/$(FILENAME).elf
	rm $(OUT_PATH)/$(FILENAME).hex
