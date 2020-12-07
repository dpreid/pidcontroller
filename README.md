# pidcontroller

The DCMotorControl library contains the basic motor functions for driving the motor forwards and backwards at certain speeds (input 0-255).

Command list:

Each of the remote lab experiments has its own command list in the SerialCommands file.

State descriptions:

STOPPED: The initial state and state from which all other states are reached. To swap from one state to another, must first enter the stopped state. Stopping from a state first enters awaiting_stop, which automatically reaches stop once complete. 

AWAITING_STOP: The state into which all states enter when the stop command is sent. Checks that motor has stopped and then enters STOPPED state.

ZERO: rotates the disk until the index point is reached and sets encoder position to 0.

CONFIGURE: The state state sets the governor height to set_height.

PID_SPEED: The mode that allows the spinning disk speed to be controlled by the PID controller. Speed set within this mode is in revolutions per minute (rpm).

PID_POSITION: The mode that allows the disk position to be set and controlled by a PID controller. Disk position is from -1000 (half a rotation CCW) to 1000(half a rotation CW).

DC_MOTOR: Runs as a simple DC motor. Set speed is between 0 and 100% (0V to 6V assuming a 12V power supply)

Schematic of the PIDController setup is shown below:

![Schematic](/images/schematic.png)
