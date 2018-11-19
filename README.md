# LabArm C++ API
Lab Arm is a multipurpose robotic arm. It is aim to be equiped on drone, UAV and combined with AI to achieved various task as maintenance, fruits picking...

<img src="/images/armpicture.PNG" width="450">

This github provide a simple c++ API to control the arm. The robotic arm use dynamixel servomotor for its 6 joints, the program was tested on a Raspberry PI 3B+, with the Dynamixel XM430 motors connected with [U2D2](http://www.robotis.us/u2d2/) (Serial-RS485).
This API is based on the developed c++ API for the XM430 servomotor available [HERE](https://github.com/BenbenIO/XM430-cpp-API). The datasheet references for the motors can be found [HERE](http://support.robotis.com/en/product/actuator/dynamixel_x/xm_series/xm430-w350.htm#bookmark5).
If you are looking for a python version, please go to the following link : (https://github.com/rasheeddo/LabRobotArmOfficial)

# Install && Dependencies
The programme depend on the dynamixel_sdk library. Installation information can be found on their [github](https://github.com/ROBOTIS-GIT/DynamixelSDK).  
<br />If you want to use a raspberry Pi please build and intall the SingleBoard Computer version (linux_sbc). For Joystick control, we based our function on [A minimal C++ object-oriented API onto joystick devices under Linux](https://github.com/drewnoakes/joystick), but the library is available on this repository.

<br />Once the install is done download this repository and make the MakeFile in the make_run directory, an run the code ./exampleArm
<br />You can add your own code to the project by adding: __SOURCES += yourcode.cpp__ in the MakeFile.

# Example Program
```c
//LabArm declaration:
	LabArm arm;
	//LabArm initialisation to mode 3 (Position Control)
	arm.MotorsInit(3);
	
	//LabArm Torque activation
	arm.TorqueON();
	arm.GripperON();
			
	//Moving the robot with the Awake-Standby-Home functions:
	printf("\nArm awaking: \n");
	arm.Awake();
	
	printf("\nArm go to Standby position: \n");
	arm.StandBy();	
	printf("\nArm go to Home position\n");
	arm.GoHome();
	usleep(1000000);
	
	//Moving the robot with XYZ coordonate: first set the wanted position and rotation of the gripper.
	float standbyXYZ[6] = {0, 340, 282, 10, 30, 0};
	arm.GotoXYZ(standbyXYZ);
	arm.ReadArmCurrent(mcurrent);
  
	//Closing the gripper.
	arm.GripperClose();
	usleep(2000000);
	
	//Going back home and disactivate the torque
	arm.GoHome();
	arm.TorqueOFF();
	arm.GripperOFF();
	
	//As we are using the motorMX430.h, we can also access motors function as follow:
	arm.motor1.PrintOperatingMode();
	arm.motor4.PrintProfile();
	
	//Joystick control Mode:
	arm.JoystickControl();
  ```
  Output of the program:

<img src="/images/ExampleRUN.PNG" width="450">

# API Description
