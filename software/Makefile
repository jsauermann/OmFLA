
################################################33
# PROGRAMMER selection...
#
# PROGRAMMER = avrisp2 -B 4   // too low for built-in clock
  PROGRAMMER = avrisp2 -B 8
# PROGRAMMER = avrisp2 -B 10
# PROGRAMMER = avrisp2 -B 100

# PROGRAMMER = xil

################################################33
# PART selection...
#
PART = attiny4313

################################################33
# FUSE selection...
#
FUSEL = 0xA2   # internal 4 MHz oscillator ÷1
FUSEL_DESCR = internal 4 MHz oscillator ÷1

################################################33
# CALIBRATION selection...
#
# CALIB = 0x47# for 9600 Baud
  CALIB = 0x3B# for 57600 Baud ~ 56.8 kHz

TOOLS = /usr/lib/avr/bin
CXX = $(TOOLS)/avr-g++ -mmcu=$(PART)
CXX_FLAGS = -Os -I /usr/lib/avr/include -Wall -Werror -std=c++11
OBJDUMP = $(TOOLS)/avr-objdump
OBJCOPY = $(TOOLS)/avr-objcopy
DUDE = $(TOOLS)/avrdude -p $(PART) -c $(PROGRAMMER)

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
	@echo "    all:          compile files"
	@echo "    clean:        remove generated files"
	@echo "    flash:        make all, then flash device"
	@echo "    flash_cal:    make all, then flash calibration program"
	@echo "    signature:    read and show device signature"
	@echo "    fusel:        read and show low fuse"
	@echo "    wfusel:       write low fuse ($(FUSEL) = $(FUSEL_DESCR))"
	@echo "    eeprom:       read and show eeprom"
	@echo "    eeprom0:      write eeprom calibration byte ($(CALIB))"
	@echo "    calibration:  read and show calibration values"
	@echo "    reset:        reset device (pulse)"
	@echo
	@echo "    !!! DEVICE MUST BE POWERED ON !!!"
	@echo

beeper_test.elf: beeper_test.cc Makefile
	$(CXX) -Wl,-Map,$*.map $(CXX_FLAGS) $< -o $@

calibrate.elf: calibrate.cc Makefile
	$(CXX) -Wl,-Map,$*.map $(CXX_FLAGS) $< -o $@

freestyle_deb.elf: freestyle.cc RFID_functions.cc print_functions.cc Makefile
	$(CXX) -Wl,-Map,$*.map $(CXX_FLAGS) -D MODE=0 $< -o $@

freestyle_eno.elf: freestyle.cc RFID_functions.cc enocean.cc Makefile
	$(CXX) -Wl,-Map,$*.map $(CXX_FLAGS) -D MODE=1 $< -o $@

freestyle_nod.elf: freestyle.cc RFID_functions.cc Makefile
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
flash_deb:	freestyle_deb.hex
	$(DUDE) -U flash:w:$<
	true

flash_eno:	freestyle_eno.hex
	$(DUDE) -U flash:w:$<
	true

flash_nod:	freestyle_nod.hex
	$(DUDE) -U flash:w:$<
	true

flash_cal:	all
	$(DUDE) -U flash:w:calibrate.hex
	true

flash_beep:	all
	$(DUDE) -U flash:w:beeper_test.hex
	true

signature:
	$(DUDE) -U signature:r:-:h

fusel:
	$(DUDE) -U lfuse:r:-:h

eeprom:
	$(DUDE) -U eeprom:r:-:h

eeprom0:
	$(DUDE) -U eeprom:w:$(CALIB):m

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

