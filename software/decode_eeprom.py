#!/usr/bin/python3
# vim: et:ts=4

# this script decodes the content if the OmFLA EEPROM. The OmFLA device stores
# its current state in its EEPROM, and the EEPROM can be read out (for trouble-
# shooting purposes) after a suspected error has occurred. Use it like this:
#
# 1. if you suspect that the OmFLA is mal-functioning, then turn it off
#    immediately. This prevents the current state of the device from being
#    overridden.
#
# 2. read the entire EEPROM content of the OmFLA device (see Makefile target
#    'seeprom:' for details. Store the EEPROM content into a file named
#    'saved_eeprom.hex' and file format 'intel-hex'. Since 1. has most likely
#    happened at night time, you can do step 2. later, but do not power
#    in the OmFLA device in the meantime.
#
# 3. run this script, which will display 3 areas:
#
#    a. the user-defined parameters,
#    b, the last trend table read from the Freestyle Libre Sensor
#    c. the conclusions that our algorithm has drawn from b, (our own
#       trend and history values as opposed to the trend and history
#       tables in the sensor).
#
infile = "saved_eeprom.hex"

#------------------------------------------------------------------------------
def print_block(block, data):
    addr = 2*0x20 + 8*(block - 3)
    d0 = data[addr   :addr +4]
    d1 = data[addr +4:addr +8]
    d2 = data[addr +8:addr+12]
    d3 = data[addr+12:addr+16]
    print("    [%2d] %s %s %s %s" % (block, d0, d1, d2, d3), end = "")
    if block == 3:          print("    ....-TTHH-....-....  (trend/history idx)")
    elif (block % 3 == 0):  print("    bbbb-GGGG-aaaa-bbbb")
    elif (block % 3 == 1):  print("    aaaa-bbbb-GGGG-aaaa")
    else:                   print("    GGGG-aaaa-bbbb-GGGG")

#------------------------------------------------------------------------------
def print_marker(frm, idx, to):
    if idx < frm:   return
    if idx >= to:   return
    print("                    ", end = "");
    for h in range(frm, idx):   print("    ", end = "")
    print("  *")


data = ""
with open(infile) as inf:
    for line in inf:
        data = "%s%s" % (data, line[9:-3])

    read_interval = 8*int(data[2*0x0D:2*0x0E], 16)
    print("User defined parameters:")
    print("    OSC calibration: %3d (ignored)"   %    int(data[2*0x00:2*0x01], 16))
    print("    sensor slope:  0.%3d mg%% per raw unit" % int(data[2*0x01:2*0x02], 16))
    print("    sensor offset:   %3d mg%%"     %    int(data[2*0x02:2*0x03], 16))
    print("    alarm HIGH:      %3d mg%%"     % (2*int(data[2*0x03:2*0x04], 16)))
    print("    alarm LOW:       %3d mg%%"     % (2*int(data[2*0x04:2*0x05], 16)))
    print("    margin HIGH:     %3d mg%%"     % (2*int(data[2*0x05:2*0x06], 16)))
    print("    margin LOW:      %3d mg%%"     % (2*int(data[2*0x06:2*0x07], 16)))
    print("    battery 1:      %4d cycles"    % (8*int(data[2*0x07:2*0x08], 16)))
    print("    battery 2:      %4d cycles"    % (8*int(data[2*0x08:2*0x09], 16)))
    print("    battery 3:      %4d cycles"    % (8*int(data[2*0x09:2*0x0A], 16)))
    print("    battery 4:      %4d cycles"    % (8*int(data[2*0x0A:2*0x0B], 16)))
    print("    battery 5:      %4d cycles"    % (8*int(data[2*0x0B:2*0x0C], 16)))
    print("    error retry:     %3d seconds"  % (8*int(data[2*0x0C:2*0x0D], 16)))
    print("    read interval:   %3d seconds"  % read_interval)

    print()
    print("Last Sensor Trend Table:");
    for block in range(3, 15):  print_block(block, data)
    print()

    npass = int(data[2*0x80:2*0x81], 16)
    trend_idx = int(data[2*0x85:2*0x86], 16)
    hist_idx  = int(data[2*0x86:2*0x87], 16)
    print("OmFLA State:")
    print("    Pass:            %3d (about %3d minutes or %.1f hours after power ON)"
           % (npass, npass*read_interval/60, npass*read_interval/3600.0))
    print("    current glucose: %3d mg%%" % (2*int(data[2*0x81:2*0x82], 16)))
    print("    initial glucose: %3d mg%% (at power-ON))" % (2*int(data[2*0x82:2*0x83], 16)))
    print("    alarm low:       %3d mg%%" % (2*int(data[2*0x83:2*0x84], 16)))
    print("    alarm high:      %3d mg%%" % (2*int(data[2*0x84:2*0x85], 16)))
    print("    trend idx:       %3d (see * below)" % trend_idx)

    print("    trend (mg%):    ", end = "");
    for t in range(8):
        addr = 2*(0xD0 + t)
        print(" %3d" % (2*int(data[addr:addr+2], 16)), end = "")
    print()
    print_marker(0, trend_idx, 8);

    print("                    ", end = "");
    for t in range(8):
        addr = 2*(0xD8 + t)
        print(" %3d" % (2*int(data[addr:addr+2], 16)), end = "")
    print()
    print_marker(8, trend_idx, 16);

    print("    history idx:     %3d (see * below)" % hist_idx)
    print("    history (mg%):  ", end = "")
    for h in range(8):
        addr = 2*(0xE0 + h)
        print(" %3d" % (2*int(data[addr:addr+2], 16)), end = "")
    print()
    print_marker(0, hist_idx, 8);

    print("                    ", end = "");
    for h in range(8):
        addr = 2*(0xE8 + h)
        print(" %3d" % (2*int(data[addr:addr+2], 16)), end = "")
    print()
    print_marker(8, hist_idx, 16);

    print("                    ", end = "");
    for h in range(8):
        addr = 2*(0xF0 + h)
        print(" %3d" % (2*int(data[addr:addr+2], 16)), end = "")
    print()
    print_marker(16, hist_idx, 24);

    print("                    ", end = "");
    for h in range(8):
        addr = 2*(0xF8 + h)
        print(" %3d" % (2*int(data[addr:addr+2], 16)), end = "")
    print()
    print_marker(24, hist_idx, 32);

