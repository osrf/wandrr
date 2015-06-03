# wandrr
Low-level software, firmware, and electronics for the WANDRR robot

The electrical schematics for each board are found in the firmware
directory for the corresponding board.

Note that the software in this repository only provides robot-level low-level
control for collecting all of the sensor data into a single UDP stream and
for distributing a robot-level UDP command stream into the various appendage
UDP streams. This repository operates on the motor-current level. Everything 
higher than this (e.g., standing, trajectory following, footstep planning, 
etc.) was provided by the Institute for Human-Machine Cognition (IHMC).
