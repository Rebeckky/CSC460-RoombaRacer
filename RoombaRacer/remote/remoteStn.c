/*
 * remoteStn.c
 *
 * Created: 3/27/2017 10:36:52 PM
 *  Author: Becky Croteau
 */
 #define F_CPU 16000000

 #include <avr/io.h>
 #include <avr/interrupt.h>
 #include <util/delay.h>
 #include "./roomba/roomba.h"
 #include "./uart/uart.h"
 #include "os.h"

 //Laser pin
 #define LASERPORT PORTC
 #define LASERPIN PC6		//digital pin 31

 //Logic analyzer testing pins
 #define LAPORT PORTC
 #define LASERTASKPIN PC5	// pin 32
 #define ROOMBAPIN PC4		// pin 33
 #define SENSORDATAPIN PC3	// pin 34
 #define READPIN PC2		// pin 35
 #define SERVOTASKPIN PC1	// pin 36



 //Additional testing pins
 #define TESTPORT PORTB	
 #define TESTPIN PB5	//digital pin 11
 #define TESTPORT2 PORTG	
 #define TESTPIN2 PG2	// digital pin 39

 //PWM pins for servos
 #define SERVOXPIN PH3		//pin 6
 #define SERVOYPIN PH4		//pin 7

 #define SERVO_DEFAULT_POS 316
 #define SERVO_LOW_POS 535
 #define SERVO_HIGH_POS 97

//data ID flags
#define LASER 2
#define SERVOX 3
#define SERVOY 4
#define ROOMBA 5
#define MODE 6


PID BluetoothReceivePID;
PID LaserTaskPID;
PID ServoXTaskPID;
PID ServoYTaskPID;
PID RoombaTaskPID;
PID GetSensorDataTaskPID;

volatile BOOL laserState;

volatile int servoXState;
volatile int prevServoXState;

volatile int servoYState;
volatile int prevServoYState;

volatile char roombaState;

volatile int bumperState;
volatile BOOL irState;
volatile uint8_t lbState;

volatile BOOL autoMode;


#define QueueSize 16
#define QueueHead 0

int laserQueue[QueueSize];
int laserTail;

int roombaQueue[QueueSize];
int roombaTail;

int servoXQueue[QueueSize];
int servoXTail;

int servoYQueue[QueueSize];
int servoYTail;

/*======================= Testing Pins ================================*/

void EnableTestPin()
{
	TESTPORT |= _BV(TESTPIN);
}
void DisableTestPin()
{
	TESTPORT &= ~_BV(TESTPIN);
}

void EnableTestPin2()
{
	TESTPORT2 |= _BV(TESTPIN2);
}
void DisableTestPin2()
{
	TESTPORT2 &= ~_BV(TESTPIN2);
}

void EnableReadTaskPin()
{
	LAPORT |= _BV(READPIN);
}
void DisableReadTaskPin()
{
	LAPORT &= ~_BV(READPIN);
}

void EnableLaserTaskPin()
{
	LAPORT |= _BV(LASERTASKPIN);
}
void DisableLaserTaskPin()
{
	LAPORT &= ~_BV(LASERTASKPIN);
}

void EnableRoombaTaskPin()
{
	LAPORT |= _BV(ROOMBAPIN);
}
void DisableRoombaTaskPin()
{
	LAPORT &= ~_BV(ROOMBAPIN);
}

void EnableServoTaskPin()
{
	LAPORT |= _BV(SERVOTASKPIN);
}
void DisableServoTaskPin()
{
	LAPORT &= ~_BV(SERVOTASKPIN);
}

void EnableSensorDataPin()
{
	LAPORT |= _BV(SENSORDATAPIN);
}
void DisableSensorDataPin()
{
	LAPORT &= ~_BV(SENSORDATAPIN);
}

/*========================== Laser Pin ==================================*/

void EnableLaserPin()
{
	LASERPORT |= _BV(LASERPIN);
}
void DisableLaserPin()
{
	LASERPORT &= ~_BV(LASERPIN);
}

/*========================= Helper Functions ============================*/

void Servo_Init() {
	// Setup ports and timers
	DDRH |= (_BV(SERVOXPIN) | _BV(SERVOYPIN)); //set pin 6 to output

	// Configure timer/counter4 as phase and frequency PWM mode
	TCNT4 = 0;
	TCCR4A |= (1<<COM4A1) | (1<<COM4B1) | (1<<WGM41);  //NON Inverted PWM
	TCCR4B |= (1<<WGM43) | (1<<WGM42) | (1<<CS41) | (1<<CS40); //PRESCALER=64 MODE 14 (FAST PWM)
	ICR4 = 4999;

	OCR4A = SERVO_DEFAULT_POS; // 90 Degrees
	OCR4B = SERVO_DEFAULT_POS;
}

void InitQueue(int *queue)
{
	for(int i = 0; i < QueueSize; i++)
	{
		queue[i] = -1;
	}
}

int Queue_isEmpty(int tail)
{
	if(tail == 0)
	{
		return 1;
	}
	return 0;
}

void Enqueue(int *queue, int value, int *tail)
{
	if(*tail == QueueSize - 1)
	{
		return;
	}
	queue[*tail] = value;
	*tail = (*tail + 1);
}

int Dequeue(int *queue, int *tail)
{
	int headData = queue[QueueHead];
	queue[QueueHead] = -1;
	int i = 1;
	while(queue[i] != -1)
	{
		if(i >= QueueSize -1) return -1;
		queue[i-1] = queue[i];
		i++;
	}
	
	*tail  = (*tail - 1);
	return headData;
}


/*========================= TASKS ======================================*/
void LaserTask()
{
	for(;;)
	{
		EnableLaserTaskPin();

		while(!Queue_isEmpty(laserTail))
		{
			
			laserState = Dequeue(laserQueue,&laserTail);

			if(laserState == 1)		//Turn laser on 
			{

				EnableLaserPin();			
			}
			else if(laserState == 0)	//Turn laser off
			{			
				DisableLaserPin();
			}
			else
			{
				continue;
			}
		}
		DisableLaserTaskPin();
		Task_Next();
	}
}

void ServoXTask()
{
	for(;;)
	{
		EnableServoTaskPin();

		while(!Queue_isEmpty(servoXTail))
		{						
			servoXState = Dequeue(servoXQueue, &servoXTail);

			if(servoXState > 510 && (prevServoXState >= (SERVO_HIGH_POS + 50)))	//move servo right
			{

				if(servoXState > 1000)
				{
					prevServoXState -= 4;
				}
				prevServoXState -= 1;
				//update pin 6 PWM pulse
				OCR4A = prevServoXState;
			}
			else if(servoXState < 490 && (prevServoXState <= (SERVO_LOW_POS - 50)))	//move servo left
			{
				if(servoXState < 20)
				{
					prevServoXState += 4;
				}
				prevServoXState += 1;
				//update pin 6 PWM pulse
				OCR4A = prevServoXState;
			}			
		}
		DisableServoTaskPin();
		Task_Next();
	}
}

void ServoYTask()
{
	for(;;)
	{
		while(!Queue_isEmpty(servoYTail))
		{
		
			servoYState = Dequeue(servoYQueue, &servoYTail);
		
			if(servoYState > 510 && (prevServoYState >= (SERVO_HIGH_POS + 50)))	//move servo up
			{
				if(servoYState > 1000)
				{
					prevServoYState -= 4;
				}
				prevServoYState -= 1;
				//update pin 7 PWM pulse
				OCR4B = prevServoYState;
			}
			else if(servoYState < 490 && (prevServoYState <= (SERVO_LOW_POS - 50)))	//move servo down
			{		
				if(servoYState < 20)
				{
					prevServoYState += 4;
				}
				prevServoYState += 1;
				//update pin 7 PWM pulse
				OCR4B = prevServoYState;
			}
		}
		Task_Next();
	}
}

void BluetoothReceiveTask()
{
	uint8_t task_flag;
	uint8_t laserData;

	uint16_t servoXData;
	uint16_t servoYData;

	uint8_t servoXDataL;
	uint8_t servoXDataH;

	uint8_t servoYDataL;
	uint8_t servoYDataH;

	char roombaData;
	 
	for(;;)
	{
		//there is data on the receive pin
		if(UCSR1A & (1<<RXC1))
		{
			//EnableReadTaskPin();			
			task_flag = Bluetooth_Receive_Byte();
			
			if(task_flag == LASER)
			{				
				laserData = Bluetooth_Receive_Byte();
				Enqueue(laserQueue,laserData,&laserTail);
			}
			else if(task_flag == SERVOX)
			{				
				servoXDataL = Bluetooth_Receive_Byte();
				servoXDataH = Bluetooth_Receive_Byte();
				servoXData = ( ((servoXDataH) << 8) | (servoXDataL) );
				Enqueue(servoXQueue,servoXData,&servoXTail);
			}
			else if(task_flag == SERVOY)
			{				
				servoYDataL = Bluetooth_Receive_Byte();
				servoYDataH = Bluetooth_Receive_Byte();
				servoYData = ( ((servoYDataH) << 8) | (servoYDataL) );
				Enqueue(servoYQueue,servoYData,&servoYTail);
			}
			else if(task_flag == ROOMBA)
			{
				roombaData = Bluetooth_Receive_Byte();
				Enqueue(roombaQueue,roombaData,&roombaTail);
			}
			else if(task_flag == MODE)
			{					
				autoMode = Bluetooth_Receive_Byte();

				while(!Queue_isEmpty(roombaTail))
				{
					Dequeue(roombaQueue,&roombaTail);
				}
			}
			else
			{
				continue;
			}
		}
		DisableReadTaskPin();
		Task_Next();
	}

}

/* ============================ Roomba =============================*/

//Test to verify Roomba communication is working
void Roomba_Test() {
	for(;;) {
		Roomba_Drive(75,DRIVE_STRAIGHT);
		_delay_ms(5000);
		Roomba_Drive(-75,DRIVE_STRAIGHT);
		_delay_ms(5000);
	}
}

void AutoDrive()
{
	//play imperial march
	Roomba_Play(0);
	//Drive forward at half of max speed
	Roomba_Drive(ROOMBA_SPEED,DRIVE_STRAIGHT);
}

void Reverse_Drive()
{
	if(roombaState == 'K')				// backward right
	{
		Roomba_Drive(-ROOMBA_SPEED, -TURN_RADIUS);
	}
	else if(roombaState == 'I')			// backward left
	{
		Roomba_Drive(-ROOMBA_SPEED, TURN_RADIUS);
	}
	else
	{
		Roomba_Drive(-ROOMBA_SPEED, DRIVE_STRAIGHT);
	}
}

void BackUp()
{
	Roomba_Drive(-ROOMBA_FULL_SPEED, DRIVE_STRAIGHT);
}

void TurnRight()
{
	Roomba_Drive(ROOMBA_SPEED, IN_PLACE_CW);
}
void TurnLeft()
{
	Roomba_Drive(ROOMBA_SPEED,IN_PLACE_CCW);
}

void SlowDown()
{
	Roomba_Drive(ROOMBA_HALF_SPEED,DRIVE_STRAIGHT);
}

void SlowRight()
{
	Roomba_Drive(ROOMBA_HALF_SPEED,-ROOMBA_TURN_FAST);
}
void SlowLeft()
{
	Roomba_Drive(ROOMBA_HALF_SPEED, ROOMBA_TURN_FAST);
}

void RoombaManualDrive()
{
	if(roombaState == 'X')								//stop
	{
		Roomba_Drive(0,0);
	}
	else if(roombaState == 'S')							//Slow
	{
		Roomba_Drive(ROOMBA_SPEED,DRIVE_STRAIGHT);
	}	
	else if(roombaState == 'F')							//forward
	{		
		Roomba_Drive(ROOMBA_FULL_SPEED,DRIVE_STRAIGHT);
	}
	else if(roombaState == 'B')							//backward
	{
		Roomba_Drive(-ROOMBA_FULL_SPEED,DRIVE_STRAIGHT);
	}
	else if(roombaState == 'L')							// left
	{
		Roomba_Drive(ROOMBA_TURN_FAST,IN_PLACE_CCW);
	}
	else if(roombaState == 'R')							//right
	{
		Roomba_Drive(ROOMBA_TURN_FAST, IN_PLACE_CW);
	}
	else if(roombaState == 'K')							// forward right
	{
		Roomba_Drive(ROOMBA_FULL_SPEED, -ROOMBA_TURN_FAST); 
	}
	else if(roombaState == 'I')							// forward left
	{
		Roomba_Drive(ROOMBA_FULL_SPEED, ROOMBA_TURN_FAST);  
	}
	else if(roombaState == 'G')							// backward left
	{
		Roomba_Drive(-ROOMBA_SPEED, TURN_RADIUS);
	}
	else if(roombaState == 'J')							// backward right
	{
		Roomba_Drive(-ROOMBA_SPEED, -TURN_RADIUS);
	}

}

void RoombaTask()
{
	for(;;)
	{
		//physical bumper triggered
		if((bumperState >= 1) && (bumperState <=3))
		{
			Reverse_Drive();	// reverse from current forward state

			if(autoMode == 1)
			{
				if(bumperState == 1)	//right bumper hit
				{
					TurnLeft();
				}
				else if(bumperState == 2)	//left bumper hit
				{
					TurnRight();
				}
				//both bumpers hit
				else
				{
					BackUp();
				}
				
			}	
		}
		else if(lbState > 0 ) //any light bumper triggered, currently ignoring center right and center left
		{
			//light bumper Left or Front Left triggered
			if((lbState == 1) || (lbState == 2))
			{
				SlowRight();
			}
			else if((lbState == 16) || (lbState == 32))	//light bumper Right or Front Right triggered
			{
				SlowLeft();
			}
			//Autonomous mode
			if(autoMode == 1)
			{
				if((lbState == 1) || (lbState == 2))
				{
					SlowRight();
				}
				else if((lbState == 16) || (lbState == 32))
				{
					SlowLeft();
				}
			}
		}
		else if(irState == TRUE)	//virtual wall detected
		{
			BackUp();
			if(autoMode == 1)
			{
				TurnRight();
			}	
		}
		else
		{
			if(autoMode == 1)
			{
				//auto drive
				AutoDrive();
			}
			else
			{	
				//Manual drive mode
				if(!Queue_isEmpty(roombaTail))
				{
					EnableTestPin();
					roombaState = Dequeue(roombaQueue,&roombaTail);
					RoombaManualDrive();
					DisableTestPin();
				}
			}
		}		
		Task_Next();
	}
}

void GetRoombaSensorDataTask()
{
	for(;;)
	{
		RoombaQueryList(45,7,13);
		
		lbState = Roomba_Receive_Byte();
		bumperState = Roomba_Receive_Byte();
		irState = Roomba_Receive_Byte();
		
		Task_Next();
	}
}


void a_main()
{
	//Initialize ports
	DDRC |= _BV(LASERPIN);
	PORTC &= ~_BV(LASERPIN);

	//Test pins
	DDRB |= _BV(TESTPIN);
	DDRG |= _BV(TESTPIN2);
	PORTB &= ~_BV(TESTPIN);
	PORTG &= ~_BV(TESTPIN2);

	Bluetooth_UART_Init();
	

	Roomba_UART_Init();
	//EnableTestPin2();
	Roomba_Init();
	//DisableTestPin2();
	Servo_Init();
	

	servoXState = SERVO_DEFAULT_POS;
	prevServoXState = SERVO_DEFAULT_POS;

	servoYState = SERVO_DEFAULT_POS;
	prevServoYState = SERVO_DEFAULT_POS;

	laserState = FALSE;
	roombaState = 'X';

	laserTail = 0;
	roombaTail = 0;
	servoXTail = 0;
	servoYTail = 0;

	InitQueue(laserQueue);
	InitQueue(roombaQueue);
	InitQueue(servoXQueue);
	InitQueue(servoYQueue);

	//Create Tasks
	BluetoothReceivePID = Task_Create_RR(BluetoothReceiveTask,0);

	ServoXTaskPID = Task_Create_Period(ServoXTask,0,15,2,310);
	ServoYTaskPID = Task_Create_Period(ServoYTask,0,15,2,308);
	RoombaTaskPID = Task_Create_Period(RoombaTask,0,15,9,301);
	LaserTaskPID = Task_Create_Period(LaserTask,0,15,2,313);

	GetSensorDataTaskPID = Task_Create_Period(GetRoombaSensorDataTask,0,15,3,305);
}
