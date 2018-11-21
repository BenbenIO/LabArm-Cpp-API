# LabArm C++ API
Lab Arm is a multipurpose robotic arm. It is aim to be equiped on drone, UAV and combined with AI to achieved various task as maintenance, fruits picking...
<p align="center">
  <img src="/images/armpicture.PNG" width="300">
</p>

This github provide a simple c++ API to control the arm. The robotic arm use dynamixel servomotor for its 6 joints, the program was tested on a Raspberry PI 3B+, with the Dynamixel XM430 motors connected with [U2D2](http://www.robotis.us/u2d2/) (Serial-RS485).
This API is based on the developed c++ API for the XM430 servomotor available [HERE](https://github.com/BenbenIO/XM430-cpp-API). All the motors are using protocol 2 and a baudrate of 57600. The datasheet references for the motors can be found [HERE](http://support.robotis.com/en/product/actuator/dynamixel_x/xm_series/xm430-w350.htm#bookmark5).
<br />If you are looking for a python version, please go to the following link : (https://github.com/rasheeddo/LabRobotArmOfficial)
<br /> **Any resquest (new function to add) or issue report are very welcomed**

# Install && Dependencies
The programme depend on the dynamixel_sdk library. Installation information can be found on their [github](https://github.com/ROBOTIS-GIT/DynamixelSDK). If you want to use a raspberry Pi please build and intall the SingleBoard Computer version (linux_sbc). For Joystick control, we based our function on [A minimal C++ object-oriented API onto joystick devices under Linux](https://github.com/drewnoakes/joystick), but the library is available on this repository.
<br /> Once the install is done clone this repository, cd into the make_run directory.
<br /> Make the MakeFile, an run the code ./exampleArm
<br /> You can add other library to the project by adding: __SOURCES += yourcode.cpp__ in the MakeFile.

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
	
//Moving the robot with XYZ coordonate: create a gripper postion table {X, Y, Z, rot_X, rot_Y, rot_Z} and run GotoXYZ.
float wantedposition[6] = {0, 340, 282, 10, 30, 0};
arm.GotoXYZ(wantedposition);
  
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
arm.gripper.PrintOperatingMode();
	
//Joystick control Mode:
arm.JoystickControl();
  ```
# Currently implementing 
<br /> I am currently working on:
* Joystick linear control
* Teach point robot
* Smart gripper
* Arm avoiding
<br />If you want other functions, please feel free to ask :)

# API Description
### Kinematics functions
* __void RobotArmFWD(float motorAngle[ ], float positionGripper[ ]);__
<br />RobotArmFWD: calcul the forward kinematics of the arm. Used to get the XYZ and rotation of the gripper, 
<br />Input: motorAngle is an [6] float array which contain the 6 motors current position (degree), positionGripper[6] the float array of the result (passed by reference)
	
* __void armINV(float wantedXYZ[ ], float outputMotorAngle[ ]);__
<br />armINV: calcul the inverse kinematic of the arm. Used for moving the arm based on XYZ command.
<br />Input: wantedXYZ positon (mm | degree) for the gripper float array [6], outputMotor the [6] array which contained the motors angle (degree)

* __void printMatrix(double matrix[][4], int size) && void printMatrix3(double matrix[][3], int size);__
<br />printMatrix*: printf matrix contain in the console. Used for debugging.
<br />Input: matrix[4][4] or [3][3] depending on the function. (can be optimized into one function)
	 
* __void multiplyMatrix(double m1[][4], double m2[][4], double m12[][4], int size); &&void multiplyMatrix3(double m1[][3], double m2[][3], double m12[][3], int size);__
<br />multiplyMatrix*: multiply 2 matrix. Used for INV-Foward kinetics function.
<br />Input: m1, m2 the 2 matrix to multiply. m12 the matrix result (passed by reference) (can be optimized into one function)
	
* __void Solve(double A[][2], double B[], double X[]);__
<br />Solve: solve a Ax = b linear system. Used for INV-Forward kinetics function. (direct resultion by Gaussian Elimination)
<br />Input: the A matrix system [2][2], b [2] array and X [2] array result (passed by reference)
		
### Motion functions
* __void MotorsInit(int mode);__
<br />Init: set all the parameteres of the 7 motors depending on the wanted mode. The profile are all set to 0 (step motion).
<br />Input: wanted mode (Availiable mode: 3 (Position), 5 (current-based position). (othermode are not supported 0 (Current), 1 (Velocity),  4 (Extended position), 16 (PWM))

* __void TorqueON();__
<br />TorqueON: activate the torque inside all the 6 arm's motor. Without torque the arm will not move.

* __void TorqueOFF();__
<br />TorqueOFF: disactivate the torque inside all the 6 arm's motor. Without torque the arm will not move.

* __void ReadArmCurrent( int motorCurrent[ ]);__
<br />ReadArmCurrent: return all motor's current.Used for arm monitoring. The current value will be printed automatically, if unwanted delete the corresponding printf section in motorXM430.cpp
<br />Input: int array [6] passed by reference to get the current value for each motors.

* __void ReadAngle(float outputAngle[ ]);__
<br />ReadAngle: write the current angle of each motors (degree) in the passed array. 
<br />Input: float [6] array to contain the 6 motors angles.
	
* __void RunArm(float inputAngle[ ]);__
<br />RunArm: write the goal position in the 6 motors. This function make the arm move, and wait until all motors stop moving.
<br />Input: inputAngle array[6]  contain the wanted angle for each motors in degree.
	
* __float MAP(uint32_t angle, long in_min, long in_max, long out_min, long out_max);__
<br />MAP: return the mapped angle value into the other unit. Used for converting angle degree to angle motors.]
<br />Inputs: angle is the value we want to convert. in_min/max: the minimal/maximal angle value in the 1fst unit. out_min/max: the minimal/maximal angle value in the 2nd unit.
<br />Output: the mapped angle.
<br />Example: MAP(m_present_position, MIN_MOTOR_ANGLE, MAX_MOTOR_ANGLE, 0.0, 360.0)
	
* __void GoHome();__
<br />GoHome(): make the arm go to the home position (home1 -> home2). The function used the Home1 and Home2[ ] float array constitued of all the MOTOR_*_HOME* of the #define section.
	
* __void Awake();__
<br />Awake(): make the arm go to the awake position (home2 -> home1). The function used the Home1 and Home2[ ] float array constitued of all the MOTOR_*_HOME* of the #define section.
	
* __void StandBy();__
<br />StandBy: make the arm go to the Standby position. For now the standby position is hardcoded {180, 180, 180, 180, 180, 180}  (maybe better to create a choice for the user.
	 
* __float DeltaPosition(float prePosition, float goalPosition);__ 
<br />DeltaPosition: calcul the angle difference between the prePosition and the goal position. Used for trajectory calculation. (maybe more optimal to pass an array into that function)
<br />Input: the prePosition of the motor (usually the current position) and the goal position in degree.
<br />Output: the differences between the angle (degree)
	
* __int FindMaxDelta(int deltaPosition[ ]);__
<br />FindMaxDelta: search and return the array index of the biggest deltaposition. Used for trajectory calculation.
<br />Input: the deltaPosition[6] array which containt the differences between the the currentPosition and the goalposition
<br />Output: the array index corresponding to the bigges deltaposition.
	
* __void TrajectoryGeneration(float goalPosition[ ], float Vstd, float Astd);__
<br />TrajectoryGeneration: set all motor's profile (velocity/acceleration) based on the current and goal position. Used to generata smooth movement of the arm.
<br />Input: the goal position array[6] containing the wanted position of the 6 motors, the based velocity-acceleration for each motors
	
* __void Goto(float goalPosition[ ], int generateTrajectory, uint32_t Vstd, uint32_t Astd);__
<br />Goto: make the arm go to the wanted position. If generateTrajectory is 1, the generated movement will be smooth, if generateTrajectory is 0, all the motors will have the same profile (Vstd, Astd)
<br />Input: the goal position array[6] containing the wanted position of the 6 motors, the option for smooth movement, the standart profile for the motors.
	
* __int WorkSpaceLimitation(float X, float Y, float Z);__
<br />WorkSpaceLimitation: check if the wanted X,Y,Z position are in the working space of the arm. The workingspace parameter are fixed in the #define section
<br />Ouput: return 0: XYZ are in the working space /  return 1: out of work range in quadrant 2 or 3 / return 2:  Z is lower than the lowest range HMAX / return 3: // Y, Z axe is negatif / return4: X,Y,Z point is not in not in the workspace
	
* __int WorkSpaceHorizontalLimitation(float X, float Y, float Z);__
<br />WorkSpaceHorizontalLimitation: check if the wanted X,Y,Z position are in the horizontal working space of the arm. The workingspace parameter are fixed in  the #define section. Used to keep a camera horizontal.
<br />Ouput: return 0: XYZ are in the working space / return 1: Input out of the XZ plane / return 2: Y out of range / return 3: one of the input if negatif.
	
* __void GetXYZ(float gripperPosition[ ]);__
<br />GetXYZ: calcul the inverve kinematics of the arm and write the result in passed array.
<br />Input: gripperPosition array[6] result of the calculation. The array is construted as gripperPosition[ * ] = {X, Y, Z, x6_rot, y6_rot, z6_rot}
	
* __void GotoXYZ(float wantedXYZ[ ]);__
<br />GotoXYZ: make the arm go to the wanted XYZ position. The arm moves only if the XYZ are in the working space
<br />Input the wantedXYZ array contening the wanted gripperPosition {X, Y, Z, x6_rot, y6_rot, z6_rot}.

###  Gripper functions
* __void GripperON();__
<br />GripperON: activate the torque in gripper's motor. Without torque the gripper will not close or open.

* __void GripperOFF();__
<br />GripperOFF: disactivate the torque in gripper's motor. Without torque the gripper will not close or open.

* __void GripperOpen() && void GripperClose();__
<br />GripperOpen-close: make the gripper close or open. The function check the status of the gripper to see if it is already in the wanted state. The goal position for the close-open can be modified in the #define section.
	
* __int GripperGetCurrent();__
<br />GripperGetCurrent: read the current inside the gripper's motor. Not used yet but can be use for object size and/or object material recognition.
	
### Joystick control
* __int FindSelectedMotor(uint8_t buttonstate[ ]);__
<br />FindSelectedMotor: return the id corresponding to a high state of the array buttonstate. Used in JoystickControl to find the join to move.
<br />Input: the buttonstate array contening updated state of the joystick button

* __int JoystickControl();__
<br />JoystickControl: allow you to control the arm with a joystick. The control is done 1 join at the time.  (This function code is overcomplicated due to issue on reading event from the joystick (axis event oversampled))
<br />To move a join you need to keep pressed the angle corresponding to the join (see table) and move the left joystick right or left.

<p align="center">
  <img src="/images/joysticktable.PNG" width="250">
</p>
