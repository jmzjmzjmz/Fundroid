/*
Motor port controls the motors via UART. It sends commands 
the describe a rotation and position.
The motor controller returns a value for success or failure.
Upon failure, the motor controller tells the coordinator where
it thinks the motors went.
*/
#define MOTOR_PORT Serial1

void setup()
{
	MOTOR_PORT.begin(9600);
}

void loop()
{
	
}