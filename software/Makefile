
################################################33
# PROGRAMMER selection...
#
# PROGRAMMER = -c avrisp2 -B 4   // too low for built-in clock
# PROGRAMMER = -c avrisp2 -B 8
  PROGRAMMER = -c avrisp2 -B 20
# PROGRAMMER = -c xil

################################################33
# PART selection (alwats 4313
#
PART = attiny4313

################################################33
# FUSE selection...
#
  FUSEL = 0xED
# FUSEL = 0xE2

ifeq ($(FUSEL),0xED)
  FUSEL_DESCR = "external Xtal, CLKPR ÷ 1"
else ifeq ($(FUSEL),"0xE2")
  FUSEL_DESCR = "internal RC oscillator, CLKPR ÷ 1"
else
  FUSEL_DESCR = "*** unsupported fuse value ***"
endif

TOOLS = /usr/lib/avr/bin
CXX = $(TOOLS)/avr-g++ -mmcu=$(PART)
CXX_FLAGS = -Os -I /usr/lib/avr/include -Wall -Werror -std=c++11
CXX_FLAGS += -D FUSEL=$(FUSEL)
OBJDUMP = $(TOOLS)/avr-objdump
OBJCOPY = $(TOOLS)/avr-objcopy
# DUDE = $(TOOLS)/avrdude -p $(PART) $(PROGRAMMER)
DUDE = avrdude -p $(PART) $(PROGRAMMER)

################################################33
# user preferences (as specified in file user_defined_parameters.hh)
#
AHIGH = $(shell grep -v '^//' user_defined_parameters.hh | grep ABSOLUTE_HIGH | awk '{print $$3}')
ALOW = $(shell grep -v '^//' user_defined_parameters.hh | grep ABSOLUTE_LOW | awk '{print $$3}')
RHIGH = $(shell grep -v '^//' user_defined_parameters.hh | grep RELATIVE_HIGH | awk '{print $$3}')
RLOW = $(shell grep -v '^//' user_defined_parameters.hh | grep RELATIVE_LOW | awk '{print $$3}')
SLOPE = $(shell grep -v '^//' user_defined_parameters.hh | grep SENSOR_SLOPE | awk '{print $$3}')
OFFSET = $(shell grep -v '^//' user_defined_parameters.hh | grep SENSOR_OFFSET | awk '{print $$3}')

all:	beeper_test.hex   \
	calibrate.hex     \
	freestyle.hex \
	eeprom.data       \
	OmFLA_printer

help:
	@echo "make targets:"
	@echo "    all:          compile the source files"
	@echo "    info:         show some useful avrdude commands"
	@echo "    clean:        remove generated files"
	@echo "    flash:        make all and flash it into the device.cc"
	@echo "    flash_beep:   all + flash the beeper test program"
	@echo "    flash_cal:    all + flash the CPU calibration program"
	@echo "    signature:    read and show the signature of the device"
	@echo "    fusel:        read and show the low fuse of the device"
	@echo "    wfusel:       write low fuse ($(FUSEL) = $(FUSEL_DESCR))"
	@echo "    eeprom:       read and show the EEPROM of the device"
	@echo "    weeprom:      write eeprom.data into the EEPROM of the device"
	@echo "    calibration:  read and show calibration values"
	@echo "    reset:        reset device (pulse)"
	@echo
	@echo "    !!! FOR DEVICE OPERATIONS THE DEVICE MUST BE POWERED ON !!!"
	@echo

freestyle.cc: printA.m4 freestyle.m4in
	[ -f $@ ] && chmod 644 $@ || true
	if [ `which m4` ]; then m4 printA.m4 freestyle.m4in > $@; \
	elif [ -f $@ ]; then touch $@; fi
	chmod 444 $@	# prevent accidental editing

string_table.incl: printB.m4 freestyle.m4in
	[ -f $@ ] && chmod 644 $@ || true
	if [ `which m4` ]; then m4 printB.m4 freestyle.m4in > $@; \
	elif [ -f $@ ]; then touch $@; fi
	chmod 444 $@	# prevent accidental editing

beeper_test.elf: beeper_test.cc Makefile
	$(CXX) -Wl,-Map,$*.map $(CXX_FLAGS) $< -o $@

calibrate.elf: calibrate.cc Makefile
	$(CXX) -Wl,-Map,$*.map $(CXX_FLAGS) $< -o $@

COMMON = freestyle.cc RFID_functions.cc user_defined_parameters.hh UART.cc

freestyle.elf: $(COMMON) enocean.cc Makefile
	$(CXX) -Wl,-Map,$*.map $(CXX_FLAGS) $< -o $@

%.lss: %.elf
	$(OBJDUMP) -h -S $< > $@
	avr-size --format=berkeley -t $<

%.dis: %.lss
	$(OBJDUMP) -D $(patsubst %.lss, %.elf, $<) > $@

%.hex: %.dis
	$(OBJCOPY) -R .eeprom -O ihex $(patsubst %.dis, %.elf, $<) $@

clean:
	rm -f *.map *.lss *.hex *.elf *.dis string_table.incl

# device programming (flashing) targets...
#
flash:	freestyle.hex eeprom.data
	$(DUDE) -U flash:w:$<:i
	$(DUDE) -U eeprom:w:eeprom.data:r
	true

flash_cal:	all
	$(DUDE) -U flash:w:calibrate.hex:i
	true

flash_beep:	all
	$(DUDE) -U flash:w:beeper_test.hex:i
	true

signature:
	$(DUDE) -U signature:r:-:h

fusel:
	$(DUDE) -U lfuse:r:-:h

eeprom:
	$(DUDE) -U eeprom:r:-:h

sflash:
	$(DUDE) -U flash:r:saved_flash.hex:i
	chmod 444 saved_flash.hex

seeprom:
	$(DUDE) -U eeprom:r:saved_eeprom.hex:i
	chmod 444 saved_eeprom.hex

weeprom: eeprom.data
	$(DUDE) -U eeprom:w:eeprom.data:r

# eeprom.data is a motorola S-record file with the following bytes:
#
# EEPROM
# Address   dec   hex   explanation
# -------------------------------
# [0]       255   FF  Atmel CPU frequency calibration (0xFF means none)
# [1]       130   82  Freestyle sensor slope
# [2]       -30   EC  Freestyle sensor offset
#
eeprom.data:	user_defined_parameters
	./user_defined_parameters > $@

calibration:
	$(DUDE) -U calibration:r:-:h

# default low fuse: 0x64 (= internal 8 MHz oscillator)             ÷8
#                   0xA2 internal 4 MHz oscillator                 ÷1
#                   0x62 internal 4 MHz oscillator                 ÷8
#                   0x68 external crystal (hangs if not present)   ÷8
#                   0x28 external crystal, output on pin 6         ÷8
wfusel:
	$(DUDE) -F -U lfuse:w:$(FUSEL).hex

# default high fuse: 0xDF
# wfuseh:
#	$(DUDE) -U hfuse:w:0xDF:m

reset: 
	$(DUDE) -Ereset
	$(DUDE) -Enoreset,vcc

user_defined_parameters: user_defined_parameters.cc user_defined_parameters.hh
	g++ -o $@ $<

OmFLA_printer: OmFLA_printer.cc string_table.incl
	g++ -o $@ $<

info:
	@echo
	@echo "    $(DUDE) -U flash:w:saved_flash.hex:i"
	@echo "    $(DUDE) -U eeprom:w:saved_eeprom.hex:r"
	@echo
