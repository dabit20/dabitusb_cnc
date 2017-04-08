This directory contains the required LinuxCNC modules.

#### dabitusb.c module ####

 This is the LinuxCNC module required to communicate with the STM32F1 running the dabitusb firmware. It needs a PREEMPT-RT kernel to work, it will not work with RTAI kernels. 
 
 Before trying to install the module, open dabitusb.c, look for the 'commpackets.h' include, and adjust the path to match the commpackets.h from the firmware located in firmware\DabitUSB_CNC\Inc
 
 Then, install using 'sudo halcompile --install dabitusb.c'.
 
 Also, note that the driver needs access to the correct /dev/hidrawX device. Make sure you install a  udev rule, or change permissions manually.
 
 To test the module, a test script is included. Run this script using 'halrun -I dabitusb_test.hal'
 
 When loaded, the module creates the following HAL pins:
 
 	float IN position-cmd-joint0
	float IN position-cmd-joint1
	float IN position-cmd-joint2
	float IN position-cmd-joint3
	
	float OUT position-fb-joint0
	float OUT position-fb-joint1
	float OUT position-fb-joint2
	float OUT position-fb-joint3
These pins represent commanded and actual position and should connect to the corresponding pins of motions' axes/joints
    
	float OUT analogin0
	float OUT analogin1
	float OUT analogin2
	float OUT analogin3
Analog inputs connected to the STM32F1. Outpuit range is 0.0..1.0 which corrsponds to 0.0V .. STM32F1 Vdd (usually 3.3V).

	bit OUT digitalin0
	bit OUT digitalin1
	bit OUT digitalin2
	bit OUT digitalin3
	bit OUT digitalin4
	bit OUT digitalin5

	bit IN digout0
	bit IN digout1
	bit IN digout2
	bit IN digout3
These pins represent the digital inputs and outputs on the STM32F1 board. 
	
	s32 OUT handwheelcount0
	s32 OUT handwheelcount1
	s32 OUT handwheelcount2
When digital inputs are used as handwheel inputs, these pins show the handwheel count. 
	
	float IN pwm0
	float IN pwm1
These pins control the PWM outputs on the STM32F1 board. 0.0 represents 0%/all low output, 1.0 represents 100%/all high output.
	
	bit IN enable
Set to true to enable the module, false to disable it. 