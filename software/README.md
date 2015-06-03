# WANDRR low-level software
This tree contains software for testing the WANDRR boards. 

The 'ros' directory contains various programs for robot-level systems. Note
that none of the actual high-speed WANDRR control data flows through ROS;
the programs in the 'ros' directory use ROS only for convenience functions
like command-line parsing, time measurement, and so on.
