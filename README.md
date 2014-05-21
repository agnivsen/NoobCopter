NoobCopter
==========

- A slightly noob-ish firmware for barebones flight of a quadrotor.

- A fresh PID implementation for stabilizing a quadcopter, based on APM 2.6.


==========

The code just uses RC input through pin A6, A7 and A8 for control.
Configure the hardware pin setup accordingly.

Push input of A7 to max for trigerring take-off.

Value of A6 acts as throttle thereafter.

Non-blocking interrupt has not been implemented yet.
Hence, we poll the RC pins after every 3 secs (bad idea, I know! Feel like contributing??? You're more than welcome).

Study the code before testing it on a hardware.

==========

Motors are attached to pin 7, 8, 11, 12.

==========

Stick to the code in the master branch, if readability is your concern.

'proto-dev-1' branch has a slightly improved functionality, with a module for landing the drone safely, but the code ain't clean.

===========

Right now, I'm not working on this project for the next month or so.

Meanwhile, if you want to tinker with the PID, test it or improve it into something better... you're efforts will be much appreciated.

Before running the NoobCopter.ino file in your Arduino workspace, make sure you import the BlockingRC, IMU and Stabilize as a library.
