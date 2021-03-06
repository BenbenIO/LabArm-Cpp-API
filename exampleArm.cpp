#include "LabArm.h"

int main()
{
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
	usleep(1000000);
	
	printf("\nArm go to Standby position: \n");
	arm.StandBy();
	usleep(1000000);
	
	printf("\nArm go to Home position\n");
	arm.GoHome();
	usleep(1000000);
	
	//Moving the robot with XYZ coordonate: create a gripper postion table {X, Y, Z, rot_X, rot_Y, rot_Z} and run GotoXYZ.
	float wantedposition[6] = {0, 340, 282, 10, 30, 0};
	arm.GotoXYZ(wantedposition);
	usleep(2000000);
	
	//Closing the gripper.
	arm.GripperClose();
	usleep(2000000);
	
	//Going back home and disactivate the torque
	arm.GoHome();
	arm.TorqueOFF();
	arm.GripperOFF();
	
	//As we are using the motorMX430.h, we can also access motors function as follow:
	arm.motor1.PrintOperatingMode();
	arm.gripper.PrintOperatingMode();
	
	//Joystick control Mode:
	arm.JoystickControl();
	return(0);
}
