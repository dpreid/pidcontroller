# pidcontroller

MotorController contains the base motor controls for driving the motor forwards and backwards at certain speeds (input 0-255).

Command list:

{"cmd":"set_position","param":50}		//IN PID MODE, position parameter between -1000 and 1000
{"cmd":"set_speed","param":50}		//speed parameter in rpm (pid_speed_mode) or 0-255 dc_motor mode)
{"cmd":"set_height","param":2}		//set the height of the governor in mm

{"cmd":"set_mode","param":"CALIBRATE"}		//zeroes the rotation and height of governor
{"cmd":"set_mode","param":"ZERO"}			//zeroes the rotation
{"cmd":"set_mode","param":"PID_POSITION_MODE"}
{"cmd":"set_mode","param":"PID_SPEED_MODE"}	//set_speed parameter in rpm
{"cmd":"set_mode","param":"DC_MOTOR_MODE"}	//set_speed parameter 0 - 255
{"cmd":"set_mode","param":"CONFIGURE"}		//sets the encoder index position to 0.
{"cmd":"set_mode","param":"STOP"}



{"cmd":"set_parameters","Kp":0.5, "Ki":0, "Kd":0.1, "dt":3, "N_errors":10}	//dt in ms

State descriptions:

STOPPED: The initial state and state from which all other states are reached. To swap from one state to another, must first enter the stopped state. Stopping from a state first enters awaiting_stop, which automatically reaches stop once complete. 

AWAITING_STOP: The state into which all states enter when the stop command is sent. Checks that motor has stopped and then enters STOPPED state.

CALIBRATE: rotates the disk until the index point is reached and sets encoder position to 0. Small calibration error allowed.

ZERO: rotates the disk until the index point is reached and sets encoder position to 0.

CONFIGURE: The state state sets the governor height to set_height.

PID_SPEED: The mode that allows the spinning disk speed to be controlled by the PID controller. Speed set within this mode is in revolutions per minute (rpm).

PID_POSITION: The mode that allows the disk position to be set and controlled by a PID controller. Disk position is from -1000 (half a rotation CCW) to 1000(half a rotation CW).

DC_MOTOR: Runs as a simple DC motor. Set speed is between 0 and 255 (0V and power supply voltage)

Schematic of the PIDController setup is shown below:

![Schematic](/images/PIDControllerSchematic.png)
