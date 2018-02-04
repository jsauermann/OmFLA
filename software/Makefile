
################################################33
# PROGRAMMER selection...
#
# PROGRAMMER = avrisp2 -B 4   // too low for built-in clock
  PROGRAMMER = avrisp2 -B 8
% PROGRAMMER = avrisp2 -B 10
# PROGRAMMER = avrisp2 -B 100

# PROGRAMMER = xil

################################################33
# PART selection...
#
PART = attiny4313

################################################33
# FUSE selection...
#
FUSEL = 0x62
FUSEL_DEST = internal 4 MHz oscillator

################################################33
# CALIBRATION selection...
#
CALIB = 0x47

TOOLS = /usr/lib/avr/bin
CXX = $(TOOLS)/avr-g++ -mmcu=$(PART)
CXX_FLAGS = -Os -I /usr/lib/avr/include -Wall -Werror -std=c++11
OBJDUMP = $(TOOLS)/avr-objdump
OBJCOPY = $(TOOLS)/avr-objcopy
DUDE = $(TOOLS)/avrdude -p $(PART) -c $(PROGRAMMER)

all:	freestyle.lss freestyle.hex freestyle.dis \
	calibrate.lss calibrate.hex calibrate.dis

help:
	@echo "make targets:"
	@echo "    all:          compile files"
	@echo "    clean:        remove generated files"
	@echo "    flash:        make all, then flash device"
	@echo "    signature:    read and show device signature"
	@echo "    fusel:        read and show low fuse"
	@echo "    wfusel:       write low fuse ($(FUSEL) = $(FUSEL_DEST))"
	@echo "    eeprom:       read and show eeprom"
	@echo "    eeprom0:      write eeprom calibration byte ($(CALIB))"
	@echo "    calibration:  read and show calibration values"
	@echo "    reset:        reset device (pulse)"
	@echo
	@echo "    !!! DEVICE MUST BE POWERED ON !!!"
	@echo

%.elf: %.cc Makefile
	$(CXX) -Wl,-Map,$*.map $(CXX_FLAGS) $< -o $@

%.lss: %.elf
	$(OBJDUMP) -h -S $< > $@
	avr-size --format=berkeley -t $<

%.dis: %.elf
	$(OBJDUMP) -D $< > $@

%.hex: %.elf
	$(OBJCOPY) -R .eeprom -O ihex $< $@

clean:
	rm -f *.map *.lss *.hex *.elf *.dis

flash:	all
	$(DUDE) -U flash:w:freestyle.hex
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

# default low fuse: 0x64 (= internal 8 MHz oscillator)
#                   0x62 internal 4 MHz oscillator
#                    0x68 external crystal (nags if not present)
#                    0x28 external crystal, output on pin 6
wfusel:
	$(DUDE) -F -U lfuse:w:$(FUSEL):m

# default high fuse: 0xDF
# wfuseh:
#	$(DUDE) -U hfuse:w:0xDF:m

reset: 
	$(DUDE) -Ereset
	$(DUDE) -Enoreset,vcc

