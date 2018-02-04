# OmFLA
One more Freestyle Libre Alarm

**NOTE: this repository is currently under construction. A working prototypr
exists, but the transfer of files is still ongoing.**

This project describes the hardware (schematic and PCB), micro-controller
software (for an Atmel ATtiny 4313), and mechanics (openscad file for 3D
printers) of a small device that MAY be useful for users of Freestyle Libre
glocoses sensors. The devive beeps if the glucose levels reported by the
sensors exceed certain thresholds.

The device is only reading the sensors and can therefore co-exist with the
standard reader shipped by Abbott. As a consequence the device cannot
initialize new sensors. The primary purpose of the device is to be worn at
night and warning the user about excessive glucose levels. The purpose is
NOT to replace the standard Abboty reader. In fact, the first action ofter
an alarm shall be to determine the glucose level with either the standard
reader, or even with a traditional blood glucose measurement.

The PCB of the device is prepared for an Enocean TCM 310 module that could
trsnsmit the glucose levels to a receiver withing, say, 20 meters. However,
the software provided in this project does not (yet) support the TCM 310.

**DISCALIMER: This device is not a medical device and does not follow
any established standards for medical devices. It is a purely experimental
device and only meant to demonstrate technical possibilities.

The information in this project is distributed in the hope that it will be
useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.**

A dististinguishing feature of the device described in this project is that
it works without any additional devices like expensive smart watches or even
smartphones. One of the design targets was low cost as opposed to high
functionality, and the components of our prototype were below $50 (including
the RFID reader, but excluding the PCB).

