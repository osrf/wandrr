# firmware
On the WANDRR robot, the motors are organized into open-ended chains for each
appendage: right leg, left leg, and the three-joint torso.

Each chain consists of a 100-volt DC bus that the power amplifiers hang from,
as well as a 48-volt DC bus that powers the controllers and sensors. Data is
passed up and down each chain using a custom UDP forwarding scheme implented in
the FPGA of each motor controller.

The motor controllers are designed around Altera Cyclone V FPGA's.  Each
controller implements five USB 1.1 root-host ports. These are used by the FPGA
to stream the joint's motor-shaft encoder and the other shaft encoder(s) on
each joint, in addition to other accessories on the link, such as foot-pressure
sensors, cooling fans, and so on. 

Since all of these accessories are just USB peripherals, they can be plugged
into standard PC hosts for best-effort performance via USB bulk transfers, or
they can be used with the WANDRR FPGA-based USB hosts for 12 kHz hard 
real-time.

The electrical schematics for each board are found in the firmware directory
for the corresponding board.
