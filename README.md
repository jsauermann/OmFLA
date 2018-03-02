# OmFLA
**O**ne **m**ore **F**reestyle **L**ibre **A**larm

**NOTE: this repository is currently under construction. A working prototype
of the OmFLA device exists and is being tested. The source files used to build
the prototype have already been transferred into this repository, but the
documentation of the device is still ongoing.**

This project contains the hardware (schematic and PCB layout), micro-controller
software (C++ for an Atmel ATtiny 4313), and mechanics (openscad file for 3D
printers) of a small device that MAY be useful for users of Freestyle Libre
glucose sensors. The device beeps if the glucose levels reported by the
sensors exceed certain thresholds.

The device is only reading the sensors and can therefore co-exist with the
standard reader shipped by Abbott. As a consequence the device cannot
initialize new sensors. The primary purpose of the device is to be worn at
night and to warn the user about excessive glucose levels. The purpose is
NOT to replace the standard Abbott reader. In fact, the first action after
an alarm shall be to determine the glucose level with either the standard
reader, or even with a traditional blood glucose measurement.

The PCB of the device is prepared for an Enocean TCM 310 module that can
transmit the glucose levels and the battery level to a receiver withing,
say, 20 meters. Low-cost receiver modules, for example for Raspberry Pis
are available. Alternatively, one can used a second OmFLA PCB as receiver
and connect it to the serial port of a Raspberry Pi or of some
micro-controller.

**DISCLAIMER: This device is not a medical device and does not follow
any established standards for medical devices. It is a purely experimental
device and only meant to demonstrate technical possibilities.**

**The information in this project is distributed in the hope that it will be
useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.**

A distinguishing feature of the device described in this project is that
it works without any additional devices like expensive smart watches or even
smart-phones. One of the design goals was low cost as opposed to high
functionality, and the cost of the components for the prototype were well
below US-$ 50.00 (including the RFID reader, but excluding the PCB and the
optional TCM 310 radio transceiver).

We are offering OmFLA PCBs to diabetes self aid groups and to individuals suffering
from diabetes at roughly our purchase price + shipping, please send an email to
omfla@jürgen-sauermann.de for details.
