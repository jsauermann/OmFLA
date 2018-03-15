
################################################33
# PROGRAMMER selection...
#
# PROGRAMMER = avrisp2 -B 4   // too low for built-in clock
# PROGRAMMER = avrisp2 -B 8
  PROGRAMMER = avrisp2 -B 10
#P ROGRAMMER = avrisp2 -B 100

# PROGRAMMER = xil

################################################33
# PART selection...
#
PART = attiny4313

################################################33
# FUSE selection...
#
FUSEL = 0x62
FUSEL_DESCR = internal 4 MHz oscillator ÷1

TOOLS = /usr/lib/avr/bin
CXX = $(TOOLS)/avr-g++ -mmcu=$(PART)
CXX_FLAGS = -Os -I /usr/lib/avr/include -Wall -Werror -std=c++11
OBJDUMP = $(TOOLS)/avr-objdump
OBJCOPY = $(TOOLS)/avr-objcopy
DUDE = $(TOOLS)/avrdude -p $(PART) -c $(PROGRAMMER)

################################################33
# user preferences (as specified in file user_defined_parameters.hh)
#
AHIGH = $(shell grep -v '^//' user_defined_parameters.hh | grep ABSOLUTE_HIGH | awk '{print $$3}')
ALOW = $(shell grep -v '^//' user_defined_parameters.hh | grep ABSOLUTE_LOW | awk '{print $$3}')
RHIGH = $(shell grep -v '^//' user_defined_parameters.hh | grep RELATIVE_HIGH | awk '{print $$3}')
RLOW = $(shell grep -v '^//' user_defined_parameters.hh | grep RELATIVE_LOW | awk '{print $$3}')
SLOPE = $(shell grep -v '^//' user_defined_parameters.hh | grep SENSOR_SLOPE | awk '{print $$3}')
OFFSET = $(shell grep -v '^//' user_defined_parameters.hh | grep SENSOR_OFFSET | awk '{print $$3}')

# target variants:
#
# freestyle_deb: debugging (UART) enabled,   no enocean TCM 310 not installed
# freestyle_eno: debugging (UART) disabled,  enocean TCM 310 installed
# freestyle_nod: debugging (UART) disabled,  enocean TCM 310 not installed
# 
all:	beeper_test.hex   \
	calibrate.hex     \
	freestyle_nod.hex \
	freestyle_eno.hex \
	freestyle_deb.hex

help:
	@echo "make targets:"
	@echo "    all:          compile source files"
	@echo "    clean:        remove generated files"
	@echo "    flash_nod:    all + flash the no-debug version of freestyle.cc"
	@echo "    flash_deb:    all + flash the debug version of freestyle.cc"
	@echo "    flash_eno:    all + flash the encocean version of freestyle.cc"
	@echo "    flash_beep:   all + flash the beeper test program"
	@echo "    flash_cal:    all + flash the CPU calibration program"
	@echo "    signature:    read and show device signature"
	@echo "    fusel:        read and show low fuse"
	@echo "    wfusel:       write low fuse ($(FUSEL)= $(FUSEL_DESCR))"
	@echo "    eeprom:       read and show the device EEPROM"
	@echo "    weeprom:      write eeprom.data to device EEPROM"
	@echo "    calibration:  read and show calibration values"
	@echo "    reset:        reset device (pulse)"
	@echo
	@echo "    !!! DEVICE MUST BE POWERED ON !!!"
	@echo

beeper_test.elf: beeper_test.cc Makefile
	$(CXX) -Wl,-Map,$*.map $(CXX_FLAGS) $< -o $@

calibrate.elf: calibrate.cc Makefile
	$(CXX) -Wl,-Map,$*.map $(CXX_FLAGS) $< -o $@

COMMON = freestyle.cc RFID_functions.cc user_defined_parameters.hh

freestyle_deb.elf: $(COMMON) print_functions.cc Makefile
	$(CXX) -Wl,-Map,$*.map $(CXX_FLAGS) -D MODE=0 $< -o $@

freestyle_eno.elf: $(COMMON) enocean.cc Makefile
	$(CXX) -Wl,-Map,$*.map $(CXX_FLAGS) -D MODE=1 $< -o $@

freestyle_nod.elf: $(COMMON) Makefile
	$(CXX) -Wl,-Map,$*.map $(CXX_FLAGS) -D MODE=2 $< -o $@

%.lss: %.elf
	$(OBJDUMP) -h -S $< > $@
	avr-size --format=berkeley -t $<

%.dis: %.lss
	$(OBJDUMP) -D $(patsubst %.lss, %.elf, $<) > $@

%.hex: %.dis
	$(OBJCOPY) -R .eeprom -O ihex $(patsubst %.dis, %.elf, $<) $@

clean:
	rm -f *.map *.lss *.hex *.elf *.dis

# device programming (flashing) targets...
#
flash_deb:	freestyle_deb.hex eeprom.data
	$(DUDE) -U flash:w:$<
	$(DUDE) -U eeprom:w:eeprom.data:r
	true

flash_eno:	freestyle_eno.hex eeprom.data
	$(DUDE) -U flash:w:$<:i
	$(DUDE) -U eeprom:w:eeprom.data:r
	true

flash_nod:	freestyle_nod.hex eeprom.data
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
	$(DUDE) -F -U lfuse:w:$(FUSEL):m

# default high fuse: 0xDF
# wfuseh:
#	$(DUDE) -U hfuse:w:0xDF:m

reset: 
	$(DUDE) -Ereset
	$(DUDE) -Enoreset,vcc

user_defined_parameters: user_defined_parameters.cc user_defined_parameters.hh
	g++ -o $@ $<

