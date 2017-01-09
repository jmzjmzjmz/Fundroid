// Global Motion Controls
boolean Armed = false;
boolean InMotion = false;
long position = 0; // Position is relative and resets after motion
int rotation = 0; // Rotation is held globally and absolute.


/*
The Coordinator port is the port by which the motor communicates to
the coordinator. The coordinator sends commands here, and the motor 
controller responds with errors or successes. 
*/
#define COORDINATOR_PORT Serial1
enum MotorControls{RotateAbsolute = 0, MoveToPosition = 1, Stop = 2, Start = 3};
enum MotorErrors{MotionComplete = 0, RotationFailure = 1, MoveToPositionFailure = 2};

struct MotionValue
{
	MotorControls MotionType;
	int position;
};

// Return Success = 0, Failure = 1
int ParseCommand(char ControlByte, String ControlArgument)
{
	MotionValue motion;
	if(ControlByte == '0')
	{
		motion.MotionType = RotateAbsolute;
		motion.position = ControlArgument.toInt();
	}
	else if(ControlByte == '1')
	{
		motion.MotionType = MoveToPosition;
		motion.position = ControlArgument.toInt();
	}
	else if(ControlByte == '2')
	{
		Armed = false;
	}
	else if(ControlByte == '3')
	{
		Armed = true;
	}
	else
	{
		return 1; // Command Not Found
	}

	return 0;
}

// Return Success = 0, Failure = 1
int CheckForCommands()
{
	enum ReadStates{ReadCommand, ReadComma, ReadArgument, end};

	char ControlByte;
	String ControlArgument;

	ReadStates state = ReadCommand;
	if(COORDINATOR_PORT.available())
	{
		while(COORDINATOR_PORT.available())
		{
			if(state == ReadCommand)
			{
				ControlByte = COORDINATOR_PORT.read();
				state = ReadComma;
			}
			else if(state == ReadComma)
			{
				char comma = COORDINATOR_PORT.read();

				if(comma != ',')
				{
					return 1;
				}

				state = ReadArgument;
			}
			else if(state == ReadArgument)
			{
				ControlArgument = COORDINATOR_PORT.readStringUntil('&');
				state = end;
			}
			else
			{
				break;
			}
		}

		return ParseCommand(ControlByte, ControlArgument);
	}

	return 0;
}

void setup()
{
	COORDINATOR_PORT.begin(9600);
}

void loop()
{
	
}