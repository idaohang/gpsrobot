///////////////////////////////////////////////////////////////////////////////
//
// GPSGMBot Software
//
// Author: JGS
//
// Note: MY RABBIT 2000 is running at 18.432MHZ!!!
//
// Date: 03/23/04
//
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// SPI Defines (defined before #use SPI.lib)
///////////////////////////////////////////////////////////////////////////////

// (576kHz, Micromag SCLK Max 1MHz Freq) see spi.lib
#define SPI_CLK_DIVISOR 32 
#define SPI_SER_B			1

///////////////////////////////////////////////////////////////////////////////
// Includes
///////////////////////////////////////////////////////////////////////////////

#use SPI.lib

///////////////////////////////////////////////////////////////////////////////
// Data Type Definitions
///////////////////////////////////////////////////////////////////////////////

typedef unsigned char UINT8;   	// 8-bit unsigned character Range: 0 to 255 (0xFF)
typedef unsigned int	UINT16;		// 16-bit unsigned integer Range: 0 to +65,535
typedef int 			 		SINT16; 	// 16-bit signed integer Range: -32,768 to +32,767
typedef unsigned long UINT32;  	// 32-bit unsigned integer Range: 0 to 232 - 1
typedef long			 		SINT32;	   // 32-bit signed integer Range: -2,147,483,648 to +2,147,483,647
typedef float			 		FLOAT32;	// 32-bit IEEE floating point value: Range: 1.18 x 10-38 to 3.40 x 1038

typedef struct GPSDataType
{
	FLOAT32 latitudeInRad;
	FLOAT32 longitudeInRad;
	UINT8		EWDir;
	FLOAT32	EWMagInMph;
	UINT8		NSDir;
	FLOAT32	NSMagInMph;
};

typedef struct DesiredStateType
{
	FLOAT32 eastingInMeters;
	FLOAT32 northingInMeters;
};

typedef struct VehicleStateType
{
	FLOAT32 eastingInMeters;
	FLOAT32 northingInMeters;
	FLOAT32 gpsHeadingInDeg;
	FLOAT32 compassHeadingInDeg;
	UINT8	  wheelEncoderSpeedInMph;
};

typedef struct ErrorSignalType
{
	FLOAT32 distanceInMeters;
	FLOAT32 azimuthInDegrees;
};

///////////////////////////////////////////////////////////////////////////////
// Constants
///////////////////////////////////////////////////////////////////////////////

//
// Debugging
//

#define DEBUG 0

//
// Serial Port Buffers - Used by Rabbit C - Do not use!
//

#define AINBUFSIZE  511
#define AOUTBUFSIZE 511
#define BINBUFSIZE  511
#define BOUTBUFSIZE 511
#define CINBUFSIZE  511
#define COUTBUFSIZE 511
#define DINBUFSIZE  511
#define DOUTBUFSIZE 511
#define EINBUFSIZE  511
#define EOUTBUFSIZE 511
#define FINBUFSIZE  511
#define FOUTBUFSIZE 511

//
// Serial Port A Data Register
//

#define SADR 0xc0		//  RW  xxxxxxxx
//Serial Port A Address Register
#define SAAR  0xc1	//  W  xxxxxxxx
// Serial Port A Long Stop Register
#define SALR 0xc2		//  W  xxxxxxxx
// Serial Port A Status Register
#define SASR 0xc3		//	 R  0xx00000
// Serial Port A Control Register
#define SACR 0xc4		//  W  xx000000
// Serial Port A Extended Register
#define SAER 0xc5		//  W  00000000
// Serial Port B Data Register
#define SBDR 0xd0		//  RW  xxxxxxxx
// Serial Port B Address Register
#define SBAR 0xd1		//  W  xxxxxxxx
// Serial Port B Long Stop Register
#define SBLR 0xd2		//  W  xxxxxxxx
// Serial Port B Status Register
#define SBSR 0xd3		//	 R 0xx00000
// Serial Port B Control Register
#define SBCR 0xd4		//  W xx000000
// Serial Port B Extended Register
#define SBER 0xd5		//  W 00000000
// Serial Port C Data Register
#define SCDR 0xe0		//  RW xxxxxxxx
// Serial Port C Address Register
#define SCAR 0xe1		//  W xxxxxxxx
// Serial Port C Long Stop Register
#define SCLR 0xe2		//  W xxxxxxxx
// Serial Port C Status Register
#define SCSR 0xe3		//  R 0xx00000
// Serial Port C Control Register
#define SCCR 0xe4		//  W xx000000
// Serial Port C Extended Register
#define SCER 0xe5		//  W 00000000
//	Serial Port D Data Register
#define SDDR 0xf0		//  RW xxxxxxxx
// Serial Port D Address Register
#define SDAR 0xf1		//  W xxxxxxxx
// Serial Port D Long Stop Register
#define SDLR 0xf2		//  W xxxxxxxx
// Serial Port D Status Register
#define SDSR  0xf3	//  R 0xx00000
// Serial Port D Control Register
#define SDCR 0xf4		//  W xx000000
// Serial Port D Extended Register
#define SDER 0xf5		//  W 00000000
// Serial Port E Data Register
#define SEDR 0xc8		//  RW xxxxxxxx
// Serial Port E Address Register
#define SEAR 0xc9		//  W xxxxxxxx
// Serial Port E Long Stop Register
#define SELR 0xca		//  W xxxxxxxx
// Serial Port E Status Register
#define SESR 0xcb 	//  R 0xx00000
// Serial Port E Control Register
#define SECR 0xcc		//  W xx000000
// Serial Port E Extended Register
#define SEER 0xcd		//  W 000x000x
// Serial Port F Data Register
#define SFDR 0xd8		//  RW xxxxxxxx
// Serial Port F Address Register
#define SFAR 0xd9		//  W xxxxxxxx
// Serial Port F Long Stop Register
#define SFLR 0xda		//  W xxxxxxxx
// Serial Port F Status Register
#define SFSR 0xdb		//  R 0xx00000
// Serial Port F Control Register
#define SFCR 0xdc		//  W xx000000
// Serial Port F Extended Register
#define SFER 0xdd 	//  W 000x000x

//
// TX & RX for Serial Ports ABCD
//

//Port C Data Register
#define PCDR 0x50		// RW x1x1x1x1
// Port C Function Register 
#define PCFR 0x55		// W x0x0x0x0

//
// TX & RX CLK for ports EF 
//

// Port G Data Register
#define PGDR 0x48		// RW xxxxxxxx
// Port G Control Register
#define PGCR 0x4c		// W  xx00xx00
// Port G Function Register
#define PGFR 0x4d		//	W xxxxxxxx
// Port G Drive Control Register
#define PGDCR 0x4e	// W xxxxxxxx
// Port G Data Direction Register
#define PGDDR 0x4f	// W 00000000

//
// Rabbit 2000 Internal Control Register
//

#define SPCR	0x24
#define PEFR	0x75
#define PEDDR	0x77
#define PEB3R	0x7b
#define IB3CR	0x83

#define TAPR	0xa1
#define TACR   0xa4
#define TAT1R  0xa3		// serial prescaler
#define TAT2R  0xa5		// serial port E
#define TAT5R  0xab		// serial port B
#define TAT6R	0xad		// serial port C

//
// Serial Port ISR Registers
//

// ISR Tx/Rx Buffers
#define FRAME_BUFFER_SIZE 				256

// Serial Interrupts
#define SERIAL_PORT_A_INT_ADDR		0x0c
#define SERIAL_PORT_B_INT_ADDR		0x0d
#define SERIAL_PORT_C_INT_ADDR		0x0e
#define SERIAL_PORT_D_INT_ADDR		0x0f		
#define ENABLE_SERIAL_INTERRUPT		0x01
#define DISABLE_SERIAL_INTERRUPT	0x00
#define TRANSMITTER_BUSY					0x04
#define TRANSMITTER_FULL					0x08
#define RECEIVER_READY						0x80

//
// Utility Module Constants
//

#define DEG_TO_RAD (FLOAT32)(3.1415926/180)
#define RAD_TO_DEG (FLOAT32)(180/3.1415926)

//
// Host Interface Module Constants
//

// LEDs
#define LED_ON         					0xff
#define LED_OFF        					0x00 

// PC Frame Specifications
// [UART_SYNC_BYTE][START_BYTE][COMMAND][DATA][CHECKSUM][END_BYTE]
// [1 byte][1 byte][1 byte][16 bytes][1 byte][1 byte] = 21 bytes

#define PC_FRAME_SIZE			   			21

#define PC_UART_SYNC_BYTE_ELEMENT	0
#define PC_UART_SYNC_BYTE  				255

#define PC_START_BYTE_ELEMENT			1
#define PC_START_BYTE		  				254

#define PC_COMMAND_ELEMENT				2
#define DOWNLOAD_PATH							1
#define START_NAVIGATION					2
#define STOP_NAVIGATION						3
#define ACTIVATE_LEDS							4
#define DEACTIVATE_LEDS						5

#define PC_DATA_ELEMENT						3
#define PC_FRAME_DATA_SIZE				21

#define PC_CHECKSUM_ELEMENT				19

#define PC_END_BYTE_ELEMENT				20
#define PC_FRAME_END 							0x0a 

//
// Sensor Module Constants
//

// Sensor ISR constants
#define ENABLE_TIMER_B_ISR				0x03
#define DISABLE_TIMER_B_ISR				0x00
#define COUNT_ONE_HUNDRED_MS			113
#define USE_GPS							0
#define USE_COMPASS_AND_ENCODER		1

// Serial Port Baud Rates
#define SERIAL_BAUDRATE       		9600

// Size Data :: GPS Frame Elements
#define GPS_FRAME_SIZE						57

// Invalid Data :: GPS Frame Elements
#define INVALID_DATA							255

// Sync Data :: GPS Frame Element
#define SENTENCE_START						0

// Time Data :: GPS Frame Elements
#define YEAR_BYTE_0								1
#define YEAR_BYTE_1								2
#define MONTH_BYTE_0							3
#define MONTH_BYTE_1							4
#define DAY_BYTE_0								5
#define DAY_BYTE_1								6
#define HOUR_BYTE_0								7
#define HOUR_BYTE_1								8
#define MINUTE_BYTE_0							9
#define MINUTE_BYTE_1							10
#define SECOND_BYTE_0							11
#define SECOND_BYTE_1							12

// Position Data :: GPS Frame Elements
#define LATITUDE_HEMI							13
#define LATITUDE_POS_BYTE_0				14
#define LATITUDE_POS_BYTE_1				15
#define LATITUDE_POS_BYTE_2				16
#define LATITUDE_POS_BYTE_3				17
#define LATITUDE_POS_BYTE_4				18
#define LATITUDE_POS_BYTE_5				19
#define LATITUDE_POS_BYTE_6				20
#define LONGITUDE_HEMI						21
#define LONGITUDE_POS_BYTE_0			22
#define LONGITUDE_POS_BYTE_1			23
#define LONGITUDE_POS_BYTE_2			24
#define LONGITUDE_POS_BYTE_3			25
#define LONGITUDE_POS_BYTE_4			26
#define LONGITUDE_POS_BYTE_5			27
#define LONGITUDE_POS_BYTE_6			28
#define LONGITUDE_POS_BYTE_7			29
#define POSITION_STATUS						30
#define HORIZ_POS_ERROR_BYTE_0		31
#define HORIZ_POS_ERROR_BYTE_1		32
#define HORIZ_POS_ERROR_BYTE_2		33
#define ALTITUDE_SIGN							34
#define ALTITUDE_BYTE_0						35
#define ALTITUDE_BYTE_1						36
#define ALTITUDE_BYTE_2						37
#define ALTITUDE_BYTE_3						38		
#define ALTITUDE_BYTE_4						39

// Velocity Data :: GPS Frame Elements
#define EAST_WEST_VEL_DIR						40
#define EAST_WEST_VEL_MAG_BYTE_0		41
#define EAST_WEST_VEL_MAG_BYTE_1		42
#define EAST_WEST_VEL_MAG_BYTE_2		43
#define EAST_WEST_VEL_MAG_BYTE_3		44
#define NORTH_SOUTH_VEL_DIR					45
#define NORTH_SOUTH_VEL_MAG_BYTE_0	46
#define NORTH_SOUTH_VEL_MAG_BYTE_1	47
#define NORTH_SOUTH_VEL_MAG_BYTE_2	48
#define NORTH_SOUTH_VEL_MAG_BYTE_3	49
#define VERTICLE_VEL_DIR						50
#define VERTICLE_VEL_MAG_BYTE_0			51
#define VERTICLE_VEL_MAG_BYTE_1			52
#define VERTICLE_VEL_MAG_BYTE_2			53
#define VERTICLE_VEL_MAG_BYTE_3			54
#define SENTENCE_END_BYTE_0					55
#define SENTENCE_END_BYTE_1					56	

//End frame Data
#define GPS_FRAME_END								0x0a

//
// Comparator Module Constants
//

//
// Control Mechanism Module Constants
//

//
// Actuator Module Constants
//

// All possible servo and motor states
#define FORWARD_FAST    				175
#define FORWARD_MED     				155 //150
#define FORWARD_SLOW    				140

#define STOP					 					110

#define REVERSE_FAST    				9
#define REVERSE_MED     				70
#define REVERSE_SLOW   					95

// Constants used to represent steering wheels current state
// They represent a mapping between the steering wheel and the servo
#define L7 									254 	// Sharp Left
#define L6 									236
#define L5 									218
#define L4 									200
#define L3 									182
#define L2 									174
#define L1 									165//156
#define GO_STRAIGHT 				127
#define R1 									115//100
#define R2 									82
#define R3 									74
#define R4 									56
#define R5 									38
#define R6 									20
#define R7  								1 		// Sharp Right

// Small angle calculation
#define TURN_LEFT						0
#define TURN_RIGHT					1

// Servo
#define SYNC_BYTE      			255
#define SERVO_1		  				0
#define MIN_POSITION   			0  	// RC Car wheels turned Right 21
#define MAX_POSITION   			254	// RC Car wheels turned Left 241
#define SERVO_TIMEOUT  			300	// Time in ms between sending ctrl signal

// Motor
#define MOTOR_1        			1		// Go fast = 254, reverse = 0, stop = 127

///////////////////////////////////////////////////////////////////////////////
// Function Prototypes
///////////////////////////////////////////////////////////////////////////////

//
// Initialization functions
//

void InitializePorts();

//
// Utility functions
//

void Delay100ms();
void Delay10ms();
void BlinkLED();
UINT8 ReverseBits(UINT8 b);

//
// Host interface module functions
//

void SendVehicleStatusToPC();
void UpdateDesiredLocation(struct DesiredStateType* DesiredState);

//
// Sensor module functions
//

void SensorISR();
void SerialPortCISR();
void SerialPortDISR();
void WheelEncoderISR();
void TestMicromag();
void ResetMicromag();
void CalibrateMicromag();
void ReadMicromag(SINT16* i_x, SINT16* i_y);
void CalcVehicleHeadingUsingMicromag();
void CalcVehicleLatitudeAndLongitudeUsingGps(FLOAT32* latInDeg, FLOAT32* longInDeg);
void ConvertLatAndLongToNorthAndEast(FLOAT32 LatInDeg, FLOAT32 LongInDeg, FLOAT32* NorthingInMeters, FLOAT32* EastingInMeters);
void CalcVehicleVelocityUsingGps(UINT8* EWDir, FLOAT32* EWMagInMetersPerSecond, UINT8* NSDir, FLOAT32* NSMagInMetersPerSecond);
void CalcVehicleHeadingUsingGps(UINT8 EWDir, FLOAT32 EWMagInMetersPerSecond, UINT8 NSDir, FLOAT32 NSMagInMetersPerSecond, FLOAT32* HeadingAngleInDeg);
void CalcVehicleState(UINT8 SensorType, struct VehicleStateType* VehicleState);

//
// Comparator Module Functions
//

void CalcErrorSignal(struct VehicleStateType* VehicleState, struct DesiredStateType* DesiredState, struct ErrorSignalType* ErrorSignal);

//
// Control Mechanism Module Functions
//

UINT16 SmallestAngleShortestTurn(FLOAT32 currentHeadingInDeg, FLOAT32 desiredHeadingInDeg);
UINT16 inAngleThresh(FLOAT32 ia2, FLOAT32 ia1, FLOAT32 ithresh);
void CalcNextServoPositionAndMotorVelocity(struct VehicleStateType* VehicleState, struct ErrorSignalType* ErrorSignal, UINT8* ServoPosition, UINT8* MotorVelocity);

//
// Actuator Module Functions
//

void ActuateServo(UINT8 ServoPosition);
void ActuateMotor(UINT8 MotorVelocity);

///////////////////////////////////////////////////////////////////////////////
// Global Variables
///////////////////////////////////////////////////////////////////////////////

//
// Host Interface Module Data
//

// Port D (PC Receiver)
char RxPcFrame[FRAME_BUFFER_SIZE];
char RxPcFrameIndex;
char PcFrameFlag;
char PcFrameStartIndex;
char PreviousPcFrameStartIndex;
char PcFrameSize;

// Port D (PC Transmitter)
char TxPcBuffer[FRAME_BUFFER_SIZE];
char TxPcBufferCurrentPtr;
char TxPcBufferNewPtr;

//
// Sensor Module Data
//

// Port C (GPS Receiver)
char RxGpsFrame[FRAME_BUFFER_SIZE];
char RxGpsFrameIndex;
char GpsFrameFlag;
char GpsFrameStartIndex;
char PreviousGpsFrameStartIndex;
char GpsFrameSize;

UINT16 SensorISRCount;
UINT16 WheelEncoderISRCount;
SINT16 XRange;
SINT16 YRange;
SINT16 XOffset;
SINT16 YOffset;
FLOAT32 Heading;
UINT16 Quadrant;

//
// Comparator Module Data
//

//
// Control mechanism module functions
//

//
// Actuator Module Data
//

UINT8 CurrentServoPosition;

// Port C (SSC Transmitter)
char tx_ssc_buffer[FRAME_BUFFER_SIZE];
char tx_ssc_buffer_current_ptr;
char tx_ssc_buffer_new_ptr;

///////////////////////////////////////////////////////////////////////////////
// Function: Main
//
// Purpose:
//   To control the robot's overall behavior
//    
// Parameters:
//   none
//
// Return Value:
//   none
//
// Side Effects:
//   none
///////////////////////////////////////////////////////////////////////////////
main()
{
	UINT8 	byteBuffer;
	UINT16 	index;
	UINT16 	x;
	UINT8		checksum;
	struct 	VehicleStateType 	vehicleState;
	struct 	DesiredStateType	desiredState;
	struct	ErrorSignalType  	errorSignal;
	UINT8		servoPosition;
	UINT8		motorVelocity;

	// testing velocity calc
	UINT8 ewDir, nsDir;
	FLOAT32 ewMph, nsMph, heading;
	struct 	VehicleStateType 	vs;
	struct 	DesiredStateType	ds;
	struct	ErrorSignalType  	es;
		
	//
	// Initialize Local and Global Variables
	//
	
	// Port C
	GpsFrameFlag					= 0;
 	RxGpsFrameIndex				= 0;
	GpsFrameStartIndex			= 0;
 	PreviousGpsFrameStartIndex	= 0;
	GpsFrameSize					= 0;
 	tx_ssc_buffer_current_ptr	= 0;
 	tx_ssc_buffer_new_ptr		= 0;
 	
 	// Port D
	PcFrameFlag						= 0;
 	RxPcFrameIndex					= 0;
	PcFrameStartIndex				= 0;
 	PreviousPcFrameStartIndex	= 0;
	PcFrameSize						= 0;
 	TxPcBufferCurrentPtr			= 0;
 	TxPcBufferNewPtr				= 0;

	byteBuffer 						= 0;
	index 							= 0;
	SensorISRCount					= 0;
	WheelEncoderISRCount 		= 0;
	x									= 0;	
	Heading = 0.0;
		
	vehicleState.eastingInMeters			= 0;
	vehicleState.northingInMeters			= 0;
	vehicleState.gpsHeadingInDeg			= 0;
	vehicleState.compassHeadingInDeg		= 0;
	vehicleState.wheelEncoderSpeedInMph	= 0;
	
	// Green box on side of SpaceDev
	desiredState.eastingInMeters			= -497013.25;
	desiredState.northingInMeters			= 3644824.25;

	// Ryan's fire hydrant
	desiredState.eastingInMeters			= -472194.15625;
	desiredState.northingInMeters			= 3681010.25;

	// Q1
	desiredState.eastingInMeters			= -488643.0625;
	desiredState.northingInMeters			= -3627144.75;

	// Q2 MAZDA REV IT UP!
	desiredState.eastingInMeters			= -488437.09375;
	desiredState.northingInMeters			= -3627135.500;

	// Q2 MAZDA REV IT UP!
	desiredState.eastingInMeters			= -493692.8125;
	desiredState.northingInMeters			= -3626183.25;

	// Q2 TRASH CAN "END"
	desiredState.eastingInMeters			= -488697.96875;
	desiredState.northingInMeters			= -3627067.0;
						
	errorSignal.distanceInMeters			= 0;
	errorSignal.azimuthInDegrees			= 0;
	
	servoPosition											= 0;
	motorVelocity											= 0;

	InitializePorts();
		
	// Start Navigation
	printf("Starting navigation...\n");
	// Enable 100ms timer interrupt
	WrPortI(TBCSR, &TBCSRShadow, ENABLE_TIMER_B_ISR);
	// Enable SSCII/GPS interrupt
	WrPortI(SCCR, &SCCRShadow, ENABLE_SERIAL_INTERRUPT);
	ActuateMotor(FORWARD_MED);
		
	// Command Processor
	while(1)
	{
		//printf("Waiting for command...\n");
	
 		// PC Command Processor
 		if(PcFrameFlag)
 		{
 			printf("Received PC Frame: %d %d %d %d %d\n", RxPcFrame[0], RxPcFrame[1], RxPcFrame[2], RxPcFrame[12], RxPcFrame[13]);

			if(RxPcFrame[PC_UART_SYNC_BYTE_ELEMENT] == PC_UART_SYNC_BYTE)
			{				
				switch(RxPcFrame[PC_COMMAND_ELEMENT])
				{
					case DOWNLOAD_PATH:
						printf("Downloading path...\n");
						// Update desired location structure	
					break;

					case START_NAVIGATION:
						printf("Starting navigation...\n");
						// Enable 100ms timer interrupt
						WrPortI(TBCSR, &TBCSRShadow, ENABLE_TIMER_B_ISR);
						// Enable SSCII/GPS interrupt
						WrPortI(SCCR, &SCCRShadow, ENABLE_SERIAL_INTERRUPT);
						ActuateMotor(FORWARD_MED);
						ActuateServo(GO_STRAIGHT);
					break;

					case STOP_NAVIGATION:
						printf("Stop navigating...\n");								
						// Disable 100ms timer interrupt
						WrPortI(TBCSR, &TBCSRShadow, DISABLE_TIMER_B_ISR);
						ActuateMotor(STOP);
						//ActuateServo(L5);
						for(index = 0; index < 1000; index++);
						// Disable SSCII/GPS interrupt
						WrPortI(SCCR, &SCCRShadow, DISABLE_SERIAL_INTERRUPT);
					break;

					case ACTIVATE_LEDS:
						printf("Activating LEDs...\n");
						WrPortI(PADR, &PADRShadow, LED_ON);													
					break;

					case DEACTIVATE_LEDS:
						printf("Deactivating LEDs...\n");
						WrPortI(PADR, &PADRShadow, LED_OFF);
					break;														
				}
			}
 						
 			PcFrameFlag = 0;  			
	 	}

		// GPS Command Processor
		if(GpsFrameFlag)
		{
			//printf("Received GPS Frame: %c %c %c %c %c\n", RxGpsFrame[0], RxGpsFrame[1], RxGpsFrame[2], RxGpsFrame[12], RxGpsFrame[13]);  			
			printf("\n\nReceived GPS Frame\n");
			GpsFrameFlag = 0;

			// Verify GPS data is valid by checking Lat and Long Hemispheres
			if(RxGpsFrame[SENTENCE_START] == '@' && RxGpsFrame[LATITUDE_HEMI] == 'N' && RxGpsFrame[LONGITUDE_HEMI] == 'W' && RxGpsFrame[SENTENCE_END_BYTE_0] == 0x0D && RxGpsFrame[SENTENCE_END_BYTE_1] == 0x0A)
			{
				CalcVehicleState(USE_GPS, &vehicleState);
				CalcErrorSignal(&vehicleState, &desiredState, &errorSignal);
				CalcNextServoPositionAndMotorVelocity(&vehicleState, &errorSignal, &servoPosition, &motorVelocity);
				ActuateServo(servoPosition);
				ActuateMotor(motorVelocity);
			}
		}

		// Compass & Wheel Encoder Processor
		if(SensorISRCount >= COUNT_ONE_HUNDRED_MS)
		{			
			//printf("Processing Compass/Encoder Data...\n");
			SensorISRCount = 0;
			//CalcVehicleState(USE_COMPASS_AND_ENCODER, &vehicleState);
			// Set a desired location for the vehicle.
			//CalcErrorSignal(&vehicleState, &desiredState, &errorSignal);
			//CalcNextServoPositionAndMotorVelocity(&vehicleState, &errorSignal, &servoPosition, &motorVelocity);
			//ActuateServo(servoPosition);
			//ActuateMotor(motorVelocity);							
		}	 	
	}	
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// INITIALIZATION FUNCTIONS
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Function: InitializePorts
//
// Purpose:
//   To initialize the serial and digital ports.
//    
// Parameters:
//   none
//
// Return Value:
//   none
//
// Side Effects:
//   none
///////////////////////////////////////////////////////////////////////////////
void InitializePorts()
{
	//
	// Initialize SPI interface (Micromag)
	//

  SPIinit();
  //ResetMicromag();

   // Port E bits 0-7 configured as inputs (WheelEncoder)
	WrPortI(PEDDR, &PEDDRShadow, 0x00);

 	// Port D, bit0=output(SCLK), bit1=output(RESET), bit2=input(DRDY), bit3-7=output (Micromag)
	WrPortI(PDDDR, &PDDDRShadow, 0x03); 

	// Reset Micromag
	//ResetMicromag();
	//ResetMicromag();
	//ResetMicromag();

	//
	// Initialize LED port A
	//

	// Port A bits 0-7 configured as outputs (LEDs)
	WrPortI(SPCR, &SPCRShadow, 0x84);

	// Set port A outputs low (turns LEDs off)	
	WrPortI(PADR, &PADRShadow, 0x00);   
	
	//
	// Initialize Serial Port C ISR (PC2/PC3 => SSCII/GPSIN)
	//

 	// Set serial port C baud rate 
 	serCopen(SERIAL_BAUDRATE);

 	// Set serial port C parity 	
 	serCparity(PARAM_NOPARITY);			
 
 	// Set serial port C ISR
 	SetVectIntern(SERIAL_PORT_C_INT_ADDR, SerialPortCISR);
 
 	// Disable receive interrupt on serial port C, priority 1
	WrPortI(SCCR, &SCCRShadow, DISABLE_SERIAL_INTERRUPT);
 	
	//
	// Setup Serial Port D ISR (PC0/PC1 => PCDEBUG/PCIN)
	//
 	
 	// Set serial port C baud rate 
 	serDopen(SERIAL_BAUDRATE);

 	// Set serial port D parity 	
 	serDparity(PARAM_NOPARITY);			
 
 	// Set serial port D ISR, and enable it
 	SetVectIntern(SERIAL_PORT_D_INT_ADDR, SerialPortDISR);
 
 	// Enable receive interrupt on serial port D, priority 1
 	WrPortI(SDCR, &SDCRShadow, ENABLE_SERIAL_INTERRUPT); 	
   
	//
	// Setup internal timerB ISR for sensor module
	//

	SetVectIntern(0x0B, SensorISR);

	// clock timer B with (perclk/16 = 0x09, perclk/2 = 0x01) and set interrupt level to 1
	WrPortI(TBCR, &TBCRShadow, 0x09);	
												
	// Set timerB match registers

	// set initial match to 0!
	WrPortI(TBL1R, NULL, 0x00);

	// this causes interrupt to occur every time the counter counts to 2^10 this creates a 0.89 milisecond roll-over time.									
	WrPortI(TBM1R, NULL, 0x00);											
													
	// Enable SensorISR thus starting the control system!
	// enable timer B and B1 match interrupts				
	WrPortI(TBCSR, &TBCSRShadow, ENABLE_TIMER_B_ISR);										
							
	//
	// Setup wheel encoder external ISR for sensor module
	//
	
	SetVectExtern2000(1, WheelEncoderISR);

	// Note: External ISR 
	// This function call has changed with version 6.19
	// of Dynamic C.  The first parameter is now the priority level
	// of the external interrupt, and the second is the ISR itself.
	
	// Enable wheel encoder interrupts

	// enable external INT0 on PE4, rising edge
	WrPortI(I0CR, NULL, 0x2B);

	// enable external INT1 on PE5, rising edge
	WrPortI(I1CR, NULL, 0x2B);

	//
	//	Setup the Micromag Electric Compass
	//

	//ResetMicromag();
	//ResetMicromag();
	//ResetMicromag();
	
	//CalibrateMicromag();
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// COMPONENT TEST FUNCTIONS
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Function: TestMicromag
//
// Purpose:
//   To test the micromag electronic compass.
//
// Parameters:
//   none
//
// Return Value:
//   none
//
// Side Effects:
//   Must first call InitializePorts(). 
//
// Notes:
//   Read sensor Y then sensor X. Do not reverse CMD byte!
//   0x71 = Read X-axis sensor.
//   0x72 = Read Y-axis sensor.
//
// Notes:
//   1. If micromag does not respond. take it out of the circuit, ground all
//      pins together, and reinsert in circuit.
//
//   2. Ground "not connected" pins.
//
//   3. Pulse reset for 10usec.
//
//   4. Send command byte to micromag. Poll DRDY until it is logic high, then
//      clock in 16-bits of data.
//
///////////////////////////////////////////////////////////////////////////////
void TestMicromag()
{
	UINT16 index;

	UINT8 wait;
	
	UINT8 drdyData;
	
	UINT8 readXAxisCommandByte;
	UINT8 readYAxisCommandByte;
	
	SINT16 XAxisData;
	SINT16 YAxisData;

	index = 0;

	XAxisData = 0;
	YAxisData = 0;
	
	// Command Byte 
	readXAxisCommandByte = 0x71;
	readYAxisCommandByte = 0x72;
	
	//
	// Read Y Axis Sensor
	//

	YAxisData = 0;
	
	// Reset micromag
	ResetMicromag();
						
	// Write command byte, wait, then read 2 bytes of data
	SPIWrite(&readYAxisCommandByte, 1);

	// Wait for DRDY (maximum of 77ms)
	wait = 1;
	while(wait)
	{
		drdyData = RdPortI(0x60);

		if(drdyData & 0x04)
		{
			wait = 0;
			printf("DRDY arrived!\n");
		}
	}
		
	// Read sensor data
	SPIRead((SINT16*) &YAxisData, 2);
	
	// Print data received (note: data will not print properly if decimal %d type is used)
	printf("Y-Axis Sensor Data (DEC): %d\n", YAxisData);	

	//
	// Read X Axis Sensor
	//

	XAxisData = 0;

	// Reset micromag
	ResetMicromag();
				
	// Write command byte, wait, then read 2 bytes of data. 
	SPIWrite(&readXAxisCommandByte, 1);

	// Wait for DRDY (maximum of 77ms)
	wait = 1;
	while(wait)
	{
		drdyData = RdPortI(0x60);

		if(drdyData & 0x04)
		{
			wait = 0;
			printf("DRDY arrived!\n");
		}
	}
		
	// Read sensor data
	SPIRead((SINT16*) &XAxisData, 2);
	
	// Print data received (note: data will not print properly if decimal %d type is used)
	printf("X-Axis Sensor Data (DEC): %d\n\n", XAxisData);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// UTILITY FUNCTIONS
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Function: ReverseBits
//
// Purpose:
//   To reverse the bits in a UINT8 (MSB to LSB).
//    
// Parameters:
//   UINT8 b = input byte to be reversed
//
// Return Value:
//   UINT8 c = reversed bit version of b
//
// Side Effects:
//   none
///////////////////////////////////////////////////////////////////////////////
UINT8 ReverseBits(UINT8 b)
{
	UINT16 	i;
	UINT8 	outb;
	UINT8 	bitv;

	outb = 0x00;
	bitv = 0x1;

	for (i=1; i<=8; i++,bitv <<= 1)
	{
  		outb |= ((b&bitv)?(0x1<<(8-i)):0x00);
  	}

	return outb;
}

///////////////////////////////////////////////////////////////////////////////
// Function: Delay100ms
//
// Purpose: To provide a 100ms delay.
//   
//    
// Parameters:
//   none
//
// Return Value:
//   none
//
// Side Effects:
//   none
///////////////////////////////////////////////////////////////////////////////
void Delay100ms()
{
	UINT16 index;
	for(index = 0; index < 13888; index++);
}

///////////////////////////////////////////////////////////////////////////////
// Function: Delay10ms
//
// Purpose: To provide a 10ms delay.
//   
//    
// Parameters:
//   none
//
// Return Value:
//   none
//
// Side Effects:
//   none
///////////////////////////////////////////////////////////////////////////////
void Delay10ms()
{
	UINT16 index;
	for(index = 0; index < 1388; index++);		// 10ms
}

///////////////////////////////////////////////////////////////////////////////
// Function: BlinkLED
//
// Purpose:
//   To blink the LED(s)
//    
// Parameters:
//   none
//
// Return Value:
//   none
//
// Side Effects:
//   Must first call InitializePorts().
///////////////////////////////////////////////////////////////////////////////
void BlinkLED()
{
	// Turn on LEDs
	WrPortI(PADR, &PADRShadow, 0xff);
	Delay100ms();				
	
		
	// Turn off LEDs
	WrPortI(PADR, &PADRShadow, 0x00);
	Delay100ms();				
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// HOST INTERFACE MODULE FUNCTIONS
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Function: UpdateDesiredLocation()
//
// Purpose:
//    
// Parameters:
//   none
//
// Return Value:
//   none
//
// Side Effects:
//   Must first call InitializePorts().
///////////////////////////////////////////////////////////////////////////////
void UpdateDesiredLocation(struct DesiredStateType* DesiredState)
{
	
}

///////////////////////////////////////////////////////////////////////////////
// Function: SendVehicleStatusToPC()
//
// Purpose:
//   Transmits packet below:
//   [START_SYNC][MODE][LATITUDE][LONGITUDE][SPEED][DIRECTION][BATTERY_LIFE][END_SYNC]
//    
// Parameters:
//   none
//
// Return Value:
//   none
//
// Side Effects:
//   Must first call InitializePorts().
///////////////////////////////////////////////////////////////////////////////
void SendVehicleStatusToPC()
{
	UINT8 status;

	TxPcBuffer[TxPcBufferNewPtr++] = 'a';
	TxPcBuffer[TxPcBufferNewPtr++] = 'b';
	TxPcBuffer[TxPcBufferNewPtr++] = 'c';	

	// No need to implement circular buffer because pointer
	// is char and buffer is char so just keep incrementing
	// pointer and it will automatically roll over :)
	
	// read serial port D status register
 	status = RdPortI(SDSR);	

 	if(!(status&TRANSMITTER_BUSY))
 	{
 		// if the tx is idle, this will KICK START the transfer
 		WrPortI(SDDR, NULL, TxPcBuffer[TxPcBufferCurrentPtr++]);
 	}
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// SENSOR MODULE FUNCTIONS
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Function: SensorISR 
//
// Purpose:
//	  Interrupt routine for timerB which controls sensor polling
//    
// Parameters:
//   none
//
// Return Value:
//   none
//
// Side Effects:
//   Must first call InitializePorts().
///////////////////////////////////////////////////////////////////////////////
#asm
SensorISR::
	push	af							; save registers
	push	hl

	ioi	ld a, (TBCSR)			; load B1, B2 interrupt flags (clears flag); this
										; should be done as soon as possible in the ISR

	ld		hl, (SensorISRCount)
	inc	hl							; increment counter
	ld		(SensorISRCount), hl
			
	ld		a, 00h
	ioi	ld (TBL1R), a			; set up next B1 match (at timer=0000h)
	ioi	ld (TBM1R), a			; NOTE:  you _need_ to reload the match
										;	register after every interrupt!

done:
	pop	hl							; restore registers
	pop	af
	
	ipres								; restore interrupts
	ret								; return
#endasm

///////////////////////////////////////////////////////////////////////////////
// Function: SerialPortCISR()
//
// Purpose:
//	  Serial Port C Interrrupt Service Routine (Sensors)
//    
// serial port status register bit map
// SCSR bit 7 = 1 -> receive character ready
// SCSR bit 6 = 0 -> always in async mode
// SCSR bit 5 = 1 -> reciever overrun
// SCSR bit 4 = 0 -> always in async mode
// SCSR bit 3 = 1 -> TX buffer is full, will initiate an interrupt on 1->0 transition
// SCSR bit 2 = 1 -> TX busy busy, will initiate an interrupt when goes idle
// SCSR bit 1 = 0 -> always in async mode
// SCSR bit 1 = 0 -> always in async mode
//
// Parameters:
//   none
//
// Return Value:
//   none
//
// Side Effects:
//   Must first call InitializePorts().
///////////////////////////////////////////////////////////////////////////////
nodebug root interrupt void SerialPortCISR()
{
	// This ISR handles both TX and RX for serial port C

	UINT8 status;
	UINT8 serial_byte;

 	// read serial port C status register 
 	status = RdPortI(SCSR);	

 	// if a byte is available
 	if(status&RECEIVER_READY) 
	{	
		// read the incoming data byte
  		serial_byte = RdPortI(SCDR);		

  		// save data byte to the serial buffer and increment index
  		RxGpsFrame[RxGpsFrameIndex++]=serial_byte;		

  		// we have a frame delimiter (line feed for Garmin Text Out Frame)
     	if(serial_byte == GPS_FRAME_END)				
     	{
			GpsFrameSize = (RxGpsFrameIndex - PreviousGpsFrameStartIndex);		

      	if(GpsFrameSize >= GPS_FRAME_SIZE)
      	{
      		GpsFrameFlag = 1;
      	}
      	 
      	GpsFrameStartIndex = PreviousGpsFrameStartIndex;

      	// avoid wraparound math, this will prevent a wraparound of the longest theoretical frame 
      	if (RxGpsFrameIndex >= GPS_FRAME_SIZE) 
      	{
       		RxGpsFrameIndex 				= 0;	
       		PreviousGpsFrameStartIndex = 0;
      	}
      	else
      	{
       		PreviousGpsFrameStartIndex = RxGpsFrameIndex;
      	} 
   	}		  		
	}

 	if(!(status&TRANSMITTER_FULL))
 	{
		// If there are bytes in the tx_buffer that have not been written
		// to the serial port data register.
  		if(tx_ssc_buffer_new_ptr != tx_ssc_buffer_current_ptr)
  		{
			// Write next byte in tx_buffer to data register. Increment tx_buffer
  			WrPortI(SCDR, NULL, tx_ssc_buffer[tx_ssc_buffer_current_ptr++]);
  		}
  		else
  		{
  			// Clear the TX emptry ISR by writing to the status register (value does not matter)
  			WrPortI(SCSR, NULL, 0x00); 
  		}
 	}	
}

///////////////////////////////////////////////////////////////////////////////
// Function: SerialPortDISR()
//
// Purpose:
//	  Serial Port D Interrrupt Service Routine (PC Command Processor)
//    
// serial port status register bit map
// SDSR bit 7 = 1 -> receive character ready
// SDSR bit 6 = 0 -> always in async mode
// SDSR bit 5 = 1 -> reciever overrun
// SDSR bit 4 = 0 -> always in async mode
// SDSR bit 3 = 1 -> TX buffer is full, will initiate an interrupt on 1->0 transition
// SDSR bit 2 = 1 -> TX busy busy, will initiate an interrupt when goes idle
// SDSR bit 1 = 0 -> always in async mode
// SDSR bit 1 = 0 -> always in async mode
//
// Parameters:
//   none
//
// Return Value:
//   none
//
// Side Effects:
//   Must first call InitializePorts().
///////////////////////////////////////////////////////////////////////////////
nodebug root interrupt void SerialPortDISR()
{
	// This ISR handles both TX and RX for serial port D

	UINT8 status;
	UINT8 serial_byte;

 	// read serial port D status register 
 	status = RdPortI(SDSR);	

 	// if a byte is available
 	if(status&RECEIVER_READY) 
	{	
		// read the incoming data byte
  		serial_byte = RdPortI(SDDR);		

  		// save data byte to the serial buffer and increment index
  		RxPcFrame[RxPcFrameIndex++] = serial_byte;		

  		// we have a frame delimiter (line feed)
     	if(serial_byte == PC_FRAME_END)				
     	{
			PcFrameSize = (RxPcFrameIndex - PreviousPcFrameStartIndex);		

      	if(PcFrameSize >= PC_FRAME_SIZE)
      	{
      		PcFrameFlag = 1;
      	}
      	 
      	PcFrameStartIndex = PreviousPcFrameStartIndex;

      	// avoid wraparound math, this will prevent a wraparound of the longest theoretical frame 
      	if (RxPcFrameIndex >= PC_FRAME_SIZE) 
      	{
       		RxPcFrameIndex 				= 0;	
       		PreviousPcFrameStartIndex  = 0;
      	}
      	else
      	{
       		PreviousPcFrameStartIndex = RxPcFrameIndex;
      	} 
   	}		  		
	}

 	if(!(status&TRANSMITTER_FULL))
 	{
		// If there are bytes in the tx_buffer that have not been written
		// to the serial port data register.
  		if(TxPcBufferNewPtr != TxPcBufferCurrentPtr)
  		{
			// Write next byte in tx_buffer to data register. Increment tx_buffer
  			WrPortI(SDDR, NULL, TxPcBuffer[TxPcBufferCurrentPtr++]);
  		}
  		else
  		{
  			// Clear the TX emptry ISR by writing to the status register (value does not matter)
  			WrPortI(SDSR, NULL, 0x00); 
  		}
 	}	
}

///////////////////////////////////////////////////////////////////////////////
// Function: WheelEncoderISR
//
// Purpose:
//	  Interrupt routine for external interrupt 0
//    
// Parameters:
//   none
//
// Return Value:
//   none
//
// Side Effects:
//   Must first call InitializePorts().
///////////////////////////////////////////////////////////////////////////////
nodebug root interrupt void WheelEncoderISR()
{
	WheelEncoderISRCount++;
}

///////////////////////////////////////////////////////////////////////////////
// Function: CalibrateMicromag
//
// Purpose:
//   To calibrate the micromag electronic compass. Updates XRange, YRange,
//	  XOffset, and YOffset global variables used in calculating heading.	
//     cdsz
// Parameters:
//   none
//
// Return Value:
//   none
//
// Side Effects:
//   Must first call InitializePorts().
///////////////////////////////////////////////////////////////////////////////
void CalibrateMicromag()
{
	UINT16 index;

	SINT16 Xmax;
	SINT16 Xmin;
	SINT16 Ymax;
	SINT16 Ymin;
	SINT16 Xraw;
	SINT16 Yraw;

	Xmax = Ymax = -32768;
	Xmin = Ymin = 32767;

	Xraw = 0;
	Yraw = 0;

	printf("Calibrating Micromag...\n");
	
	for(index = 0; index < 25; index++)
	{
		ReadMicromag(&Xraw, &Yraw);

		if(Xraw > Xmax)
		{
			Xmax = Xraw;
		}

		if(Xraw < Xmin)
		{
			Xmin = Xraw;
		}

		if(Yraw > Ymax)
		{
			Ymax = Yraw;
		}

		if(Yraw < Ymin)
		{
			Ymin = Yraw;
		}		
	}

	XOffset = (Xmax + Xmin) >> 1;
	YOffset = (Ymax + Ymin) >> 1;
	XRange  = (Xmax - Xmin);
	YRange  = (Ymax - Ymin);
	
	printf("Calibration complete. XRange and YRange should be positive numbers!\n");
	printf("XOffset, YOffset, XRange, YRange: %d, %d, %d, %d\n", XOffset, YOffset, XRange, YRange);
}

///////////////////////////////////////////////////////////////////////////////
// Function: CalcVehicleHeadingUsingMicromag
//
// Purpose:
//   To calculate the robot heading based on the micromag electric compass.
//    
// Parameters:
//   none
//
// Return Value:
//   none
//
// Side Effects:
//   Must first call InitializePorts(). Must first call CalibrateMicromag().
//
// Note:
//   Quandrant = 1 => 0-90 degrees    = Where 90 degrees is North.
//   Quadrant  = 2 => 91-180 degrees  = Where 180 degrees is West.
//   Quadrant  = 3 => 181-270 degrees = Where 270 degrees is South.
//   Quadrant  = 4 => 271-359 degrees = Where 359 degrees is East; 
///////////////////////////////////////////////////////////////////////////////
void CalcVehicleHeadingUsingMicromag()
{
	SINT16 Xvalue;
	SINT16 Yvalue;
	SINT16 Xraw;
	SINT16 Yraw;
	FLOAT32 Angle;

	Xvalue = 0;
	Yvalue = 0;
	Xraw   = 0;
	Yraw   = 0;

	Angle		= 0.0;	
	Heading	= 0.0;
			
	ReadMicromag(&Xraw, &Yraw);

	Xvalue = Xraw - XOffset;
	Yvalue = Yraw - YOffset;

	if(XRange > YRange)
	{
		if(YRange == 0)
		{
			printf("Divide by zero 1.\n");
			printf("Xvalue, Yvalue: %d %d\n", Xvalue, Yvalue);			
			//exit(0);
			return;
		}
		else
		{		
			//Yvalue = (FLOAT32)((Yvalue * XRange) / YRange);
			Yvalue = ((Yvalue * XRange) / YRange);
		}
	}
	else
	{
		if(XRange == 0)
		{
			printf("Divide by zero 2.\n");
			printf("Xvalue, Yvalue: %d %d\n", Xvalue, Yvalue);
			//exit(0);
			return;
		}
		else
		{		
			Xvalue = ((Xvalue * YRange) / XRange);
		}
	}

	Angle = (atan2((FLOAT32)Yvalue, (FLOAT32)Xvalue) * RAD_TO_DEG);
	
	/*
	if(Xvalue == 0)
	{
		printf("Divide by zero 3.\n");
		printf("Xvalue, Yvalue: %d %d\n", Xvalue, Yvalue);		
		//exit(0);
		return;
	}
	else
	{
		Angle = (atan2((FLOAT32)Yvalue, (FLOAT32)Xvalue) * RAD_TO_DEG);
		//Angle = (atan((FLOAT32)(Yvalue/Xvalue)) * RAD_TO_DEG);
	}
	*/

	if(Xvalue >= 0 && Yvalue >= 0)
	{
		Heading  = Angle;
		Quadrant = 1;
	}
	else if(Xvalue < 0 && Yvalue >=0)
	{
		Heading = 180 - Angle;
		Quadrant = 2;
	}
	else if(Xvalue < 0 && Yvalue < 0)
	{
		Heading = 180 + Angle;
		Quadrant = 3;
	}
	else if(Xvalue >= 0 && Yvalue < 0)
	{
		Heading = 360 - Angle;
		//Heading = 270 - Angle;
		Quadrant = 4;
	}

	printf("Xraw, Yraw, Quadrant, Angle, Heading: %d, %d, %d, %f, %f\n", Xraw, Yraw, Quadrant, Angle, Heading);
}

///////////////////////////////////////////////////////////////////////////////
// Function: ReadMicromag
//
// Purpose:
//   To read the micromag electronic compass and update parameters passed.	
//    
// Parameters:
//   none
//
// Return Value:
//   none
//
// Side Effects:
//   Must first call InitializePorts().
//
// Notes:
//   Read sensor Y then sensor X. Do not reverse CMD byte.
//   0x71 = Read X-axis sensor.
//   0x72 = Read Y-axis sensor.
//
// Notes:
//   1. If micromag does not respond. take it out of the circuit, ground all
//      pins together, and reinsert in circuit. Piece of Shit.
//
//   2. Ground "not connected" pins.
//
//   3. Pulse reset for 10usec.
//
//   4. Send command byte to micromag. Poll DRDY until it is logic high, then
//      clock in 16-bits of data.
//
///////////////////////////////////////////////////////////////////////////////
void ReadMicromag(SINT16* i_x, SINT16* i_y)
{
	UINT8 wait;
	UINT8 drdyData;

	UINT8 readXAxisCommandByte;
	UINT8 readYAxisCommandByte;
	
	SINT16 XAxisData;
	SINT16 YAxisData;

	// Command Byte
	readXAxisCommandByte = 0x71;
	readYAxisCommandByte = 0x72;

	// Zero out data
	XAxisData = 0;
	YAxisData = 0;

	// Reset micromag
	ResetMicromag();
	
	// Write command byte, wait, then read 2 bytes of sensor data
	SPIWrite(&readYAxisCommandByte, 1);

	// Wait for DRDY (maximum of 77ms)
	wait = 1;
	while(wait)
	{
		drdyData = RdPortI(0x60);

		if(drdyData & 0x04)
		{
			wait = 0;
			//printf("DRDY arrived!\n");
		}
		
		//printf("waiting for DRDY 1...\n");
	}
		
	// Read the data in
	SPIRead((SINT16*) &YAxisData, 2);
	
	// Reset micromag
	ResetMicromag();
	
	// Write command byte, wait, then read 2 bytes of sensor data
	SPIWrite(&readXAxisCommandByte, 1);

	// Wait for DRDY (maximum of 77ms)
	wait = 1;
	while(wait)
	{
		drdyData = RdPortI(0x60);

		if(drdyData & 0x04)
		{
			wait = 0;
			//printf("DRDY arrived!\n");			
		}
		
		//printf("waiting for DRDY 2...\n");
	}
		
	// Read the data in
	SPIRead((SINT16*) &XAxisData, 2);

	// Update parameters
	*i_x = XAxisData;
	*i_y = YAxisData;

	printf("Micrmag Read: XAxisData = %d YAxisData = %d\n", *i_x, *i_y);
}

///////////////////////////////////////////////////////////////////////////////
// Function: ResetMicromag
//
// Purpose:
//   To reset the micromag electronic compass.
//    
// Parameters:
//   none
//
// Return Value:
//   none
//
// Side Effects:
//   Must first call InitializePorts().
///////////////////////////////////////////////////////////////////////////////
void ResetMicromag()
{
	WrPortI(PDDR, NULL, 0x00);	
	Delay10ms();
	WrPortI(PDDR, NULL, 0x02);
	Delay10ms();
	WrPortI(PDDR, NULL, 0x00);	
	Delay10ms();
}

///////////////////////////////////////////////////////////////////////////////
// Function: CalcVehicleLatitudeAndLongitudeUsingGps
//
// Tested? Yes. In Matlab as well as a Visual C++ program!
//
// Purpose:
//  
//    
// Parameters:
//   none
//
// Return Value:
//   none
//
// Side Effects:
//   Must be called after GPS unit data has been updated.
///////////////////////////////////////////////////////////////////////////////
void CalcVehicleLatitudeAndLongitudeUsingGps(FLOAT32* latInDeg, FLOAT32* longInDeg)
{
 	UINT32 index;
 	FLOAT32 latdd;
 	FLOAT32 latmm;
 	FLOAT32 latmmm;
 	FLOAT32 latitudeInDeg;
 	FLOAT32 latitudeInRad;
 	FLOAT32 longdd;
 	FLOAT32 longmm;
 	FLOAT32 longmmm;
 	FLOAT32 longitudeInDeg;
 	FLOAT32 longitudeInRad;
  	
	//
	// Calculate Latitude
	//
	// GPS Latitude position	7 bytes	WGS84 ddmmmmm, with an implied decimal after the 4th digit
	//
	// Given latitude position from GPS in 7 bytes. Need to assemble into 32-bit float.
	//
  	
 	latdd = 0.0;

 	for(index = LATITUDE_POS_BYTE_0; index < (LATITUDE_POS_BYTE_0 + 2); index++)
 	{
   	latdd = (latdd + (RxGpsFrame[index] & 0x0F) * pow(10, (1 - (index - LATITUDE_POS_BYTE_0))));
 	}

 	latmm = 0.0;  

 	for(index = LATITUDE_POS_BYTE_2; index < (LATITUDE_POS_BYTE_2 + 2); index++)
 	{
   	latmm = latmm + (RxGpsFrame[index] & 0x0F) * pow(10, (1 + (LATITUDE_POS_BYTE_2 - index)));
 	}

 	latmmm = 0.0;

 	for(index = LATITUDE_POS_BYTE_4; index < (LATITUDE_POS_BYTE_4 + 3); index++)
 	{
   	latmmm = latmmm + (RxGpsFrame[index] & 0x0F) * pow(10, (2 + (LATITUDE_POS_BYTE_4 - index)));
 	}

 	latitudeInDeg = 0.0;

 	latitudeInDeg = latdd + (latmm/60) + (latmmm/1000)/60;

	*latInDeg = latitudeInDeg;
  	  	
	//
	// Calculate Longitude
	//
 	// Longitude position	8 bytes	WGS84 dddmmmmm with an implied
	//                               decimal after the 5th digit
	//
	// Given longitude position from GPS in 8 bytes. Need to assemble into 32-bit float.
	//

 	longdd = 0.0;

 	for(index = LONGITUDE_POS_BYTE_0; index < (LONGITUDE_POS_BYTE_0 + 3); index++)
 	{
   	longdd = (longdd + (RxGpsFrame[index] & 0x0F) * pow(10, (2 - (index - LONGITUDE_POS_BYTE_0))));
 	}

 	longmm = 0.0;  

 	for(index = LONGITUDE_POS_BYTE_3; index < (LONGITUDE_POS_BYTE_3 + 2); index++)
 	{
   	longmm = longmm + (RxGpsFrame[index] & 0x0F) * pow(10, (1 + (LONGITUDE_POS_BYTE_3 - index)));
 	}

 	longmmm = 0.0;

 	for(index = LONGITUDE_POS_BYTE_5; index < (LONGITUDE_POS_BYTE_5 + 3); index++)
 	{
   	longmmm = longmmm + (RxGpsFrame[index] & 0x0F) * pow(10, (2 + (LONGITUDE_POS_BYTE_5 - index)));
 	}
  
 	longitudeInDeg = 0.0;

 	longitudeInDeg = longdd + (longmm/60) + (longmmm/1000)/60;

	*longInDeg = longitudeInDeg;

	printf("GPS Frame: latInDeg (dd.ddddd) =   %f\n", latitudeInDeg);
	printf("GPS Frame: longInDeg (ddd.ddddd) = %f\n", longitudeInDeg);
}

///////////////////////////////////////////////////////////////////////////////
// Function: ConvertLatAndLongToNorthAndEast()
//
// Purpose: To convert latitude and longitude to northing and easting.
//
// Tested? Yes. In Matlab and Visual C++ program!
//    
// Parameters:
//		LongInDeg = This calculation assumes that LatInDeg in North and
//								LongInDeg is West? In any case, must pass in (-LongInDeg).
//
// Return Value:
//   none
//
// Side Effects
//   Must be called after GPS unit data has been updated.
//
// Notes
//		This function works! It was developed in Matlab, then VC++.
///////////////////////////////////////////////////////////////////////////////
void ConvertLatAndLongToNorthAndEast(FLOAT32 LatInDeg, FLOAT32 LongInDeg, FLOAT32* NorthingInMeters, FLOAT32* EastingInMeters)
{
	FLOAT32 a;
	FLOAT32 eccSquared;
	FLOAT32 k0;
	FLOAT32 LongOrigin;
	FLOAT32 eccPrimeSquared;
	FLOAT32 N, T, C, A, M;
 	FLOAT32 LongTemp;
	FLOAT32 LatRad;
	FLOAT32 LongRad;
	FLOAT32 LongOriginRad;
  FLOAT32 FOURTHPI;
  FLOAT32 deg2rad;
  FLOAT32 rad2deg;
  FLOAT32 zoneNumber;
  FLOAT32 easting;
  FLOAT32 northing;

 	northing 					= 0.0;
 	easting						= 0.0;
 	zoneNumber        = 11;
 	FOURTHPI          = PI / 4;
 	deg2rad           = PI / 180;
 	rad2deg           = 180.0 / PI;
 	a                 = 6378137;
 	eccSquared        = 0.00669438;
 	k0                = 0.9996;
 	LongTemp          = (LongInDeg+180)-(UINT32)((LongInDeg+180)/360)*360-180; //Make sure the longitude is between -180.00 .. 179.9
 	LatRad            = LatInDeg*deg2rad;
	LongRad           = LongTemp*deg2rad;
	eccPrimeSquared   = (eccSquared)/(1-eccSquared);
	N                 = a/sqrt(1-eccSquared*sin(LatRad)*sin(LatRad));
	T                 = tan(LatRad)*tan(LatRad);
	C                 = eccPrimeSquared*cos(LatRad)*cos(LatRad);
	LongOrigin        = (zoneNumber - 1)*6 - 180 + 3;  //+3 puts origin in middle of zone
	LongOriginRad     = LongOrigin * deg2rad;
	A                 = cos(LatRad)*(LongRad-LongOriginRad);

	M = a*((1	- eccSquared/4		- 3*eccSquared*eccSquared/64	- 5*eccSquared*eccSquared*eccSquared/256)*LatRad 
				- (3*eccSquared/8	+ 3*eccSquared*eccSquared/32	+ 45*eccSquared*eccSquared*eccSquared/1024)*sin(2*LatRad)
									+ (15*eccSquared*eccSquared/256 + 45*eccSquared*eccSquared*eccSquared/1024)*sin(4*LatRad) 
									- (35*eccSquared*eccSquared*eccSquared/3072)*sin(6*LatRad));
	
 	easting = (FLOAT32)(k0*N*(A+(1-T+C)*A*A*A/6
					+ (5-18*T+T*T+72*C-58*eccPrimeSquared)*A*A*A*A*A/120)
					+ 500000.0);

	northing = (FLOAT32)(k0*(M+N*tan(LatRad)*(A*A/2+(5-T+9*C+4*C*C)*A*A*A*A/24
				 + (61-58*T+T*T+600*C-330*eccPrimeSquared)*A*A*A*A*A*A/720)));
	
 	if(LatInDeg < 0)
 	{
		northing += 10000000.0; //10000000 meter offset for southern hemisphere
		printf("10000000 meter offset for southern hemisphere used in Latitude calculation!\n");
 	}

	*NorthingInMeters = -northing;

	// negation because this algorithm calculates the westing not easting.
	*EastingInMeters  = -easting;

	printf("GPS Frame: UTMNorthing = %f\n", northing);
	printf("GPS Frame: UTMEasting =  %f\n", easting);
}

///////////////////////////////////////////////////////////////////////////////
// Function: CalcVehicleVelocityUsingGps
//
// Works? Yes. In Matlab, Visual C++ and Rabbit Debugger!
//
// Purpose: Used in conjuction with the ErrorSignal for the control module
//				to determine which way the robot should turn to correct it's
//				error. See robot heading diagram for more information.
//    
// Parameters:
//   none
//
// Return Value:
//   none
//
// Side Effects
//   Must be called after GPS unit data has been updated (GPSDataFlag set)
//   by interrupt.
///////////////////////////////////////////////////////////////////////////////
void CalcVehicleVelocityUsingGps(UINT8* EWDir, FLOAT32* EWMagInMetersPerSecond, UINT8* NSDir, FLOAT32* NSMagInMetersPerSecond)
{
  UINT32  index;
  FLOAT32 ewMagInMetersPerSecond;
  FLOAT32 nsMagInMetersPerSecond;
	FLOAT32 hundredsComponent;
	FLOAT32 tenthsComponent;

	index										= 0;
	nsMagInMetersPerSecond 	= 0;
	ewMagInMetersPerSecond 	= 0;
	hundredsComponent	= 0;
	tenthsComponent		= 0;
	
	// Calculate ewDir
	*EWDir = RxGpsFrame[EAST_WEST_VEL_DIR];

	// Calculate nsDir
	*NSDir = RxGpsFrame[NORTH_SOUTH_VEL_DIR];

	// Calculate NS magnitude in meters per second
	nsMagInMetersPerSecond = atoi(&RxGpsFrame[NORTH_SOUTH_VEL_MAG_BYTE_0]);
	nsMagInMetersPerSecond = (FLOAT32)(nsMagInMetersPerSecond / 10);
	*NSMagInMetersPerSecond = nsMagInMetersPerSecond;
		
	// Calculate EW magnitude in meters per second
	ewMagInMetersPerSecond = atoi(&RxGpsFrame[EAST_WEST_VEL_MAG_BYTE_0]);
	ewMagInMetersPerSecond = (FLOAT32)(ewMagInMetersPerSecond / 10);
	*EWMagInMetersPerSecond = ewMagInMetersPerSecond;
		
	printf("GPS Frame: NSDir = %c\n", *NSDir);
	printf("GPS Frame: EWDir = %c\n", *EWDir);
	printf("GPS Frame: NSMagInMeterPerSecond = %f\n", *NSMagInMetersPerSecond);
	printf("GPS Frame: EWMagInMeterPerSecond = %f\n", *EWMagInMetersPerSecond);
}

///////////////////////////////////////////////////////////////////////////////
// Function: CalcVehicleHeadingUsingGps
//
// Tested? Yes. On Ti92 and Rabbit Debugger!
//
// Purpose: Used in conjuction with the ErrorSignal for the control module
//				to determine which way the robot should turn to correct it's
//				error. See robot heading diagram for more information.
//    
// Parameters:
//   none
//
// Return Value:
//   none
//
// Side Effects
//   Must be called after GPS unit data has been updated (GPSDataFlag set)
//   by interrupt.
///////////////////////////////////////////////////////////////////////////////
void CalcVehicleHeadingUsingGps(UINT8 EWDir, FLOAT32 EWMagInMetersPerSecond, UINT8 NSDir, FLOAT32 NSMagInMetersPerSecond, FLOAT32* HeadingAngleInDeg)
{
	FLOAT32 headingAngleInDeg;

	headingAngleInDeg = 0;

	if(EWMagInMetersPerSecond == 0 && NSMagInMetersPerSecond == 0)
	{
		printf("Cannot calculate heading. Robot is not in motion.\n");
		*HeadingAngleInDeg = -1;
	}
	else
	{
			
		if(NSDir == 'N')
		{
			// case 1 (Northeast)
			if(EWDir == 'E')
			{
				headingAngleInDeg = (RAD_TO_DEG * atan2(EWMagInMetersPerSecond, NSMagInMetersPerSecond));				
			}
			// case 2 (Northwest)
			else
			{
				headingAngleInDeg = (360 - (RAD_TO_DEG * atan2(EWMagInMetersPerSecond, NSMagInMetersPerSecond)));				
			}			
		}
		else
		{
			// case 3 (Southeast)
			if(EWDir == 'E')
			{
				headingAngleInDeg = (180 - (RAD_TO_DEG * atan2(EWMagInMetersPerSecond, NSMagInMetersPerSecond)));				
			}
			// case 4 (Southwest)
			else
			{
				headingAngleInDeg = (270 - (RAD_TO_DEG * atan2(NSMagInMetersPerSecond, EWMagInMetersPerSecond)));				
			}			
		}

		*HeadingAngleInDeg = headingAngleInDeg;

		printf("GPS Frame: HeadingAngleInDeg = %f\n", *HeadingAngleInDeg);
	}
}

///////////////////////////////////////////////////////////////////////////////
// Function: CalcVehicleState
//
// Tested? Yes. Using Rabbit Debugger!
//
// Purpose: Used in conjuction with the ErrorSignal for the control module
//				to determine which way the robot should turn to correct it's
//				error.  
//
// Parameters:
//   none
//
// Return Value:
//   none
//
// Side Effects
//   Must be called after GPS unit data has been updated (GPSDataFlag set)
///////////////////////////////////////////////////////////////////////////////
void CalcVehicleState(UINT8 SensorType, struct VehicleStateType* VehicleState)
{
	UINT32  index;
	FLOAT32 latInDeg;
	FLOAT32 longInDeg;
	FLOAT32 northingInMeters;
	FLOAT32 eastingInMeters;
	UINT8 	EWDir;
	FLOAT32 EWMagInMph;
	UINT8		NSDir;
	FLOAT32 NSMagInMph;
	FLOAT32 headingInDeg;
		
	if(SensorType == USE_GPS)
	{
		// Calculate Vehicle State
		CalcVehicleLatitudeAndLongitudeUsingGps(&latInDeg, &longInDeg);
		ConvertLatAndLongToNorthAndEast(latInDeg, -longInDeg, &northingInMeters, &eastingInMeters);
		CalcVehicleVelocityUsingGps(&EWDir, &EWMagInMph, &NSDir, &NSMagInMph);
		CalcVehicleHeadingUsingGps(EWDir, EWMagInMph, NSDir, NSMagInMph, &headingInDeg);
		
		// Update Vehicle State
		VehicleState->eastingInMeters = eastingInMeters;
		VehicleState->northingInMeters = northingInMeters;		
		VehicleState->gpsHeadingInDeg = headingInDeg;
	}
	else if(SensorType == USE_COMPASS_AND_ENCODER)
	{
		// Calculate Vehicle State
		// Do nothing until I get the Micromag to work properly.
		CalcVehicleHeadingUsingMicromag();
		// Update VehicleState

		// Update Vehicle State
		// VehicleState->eastingInMeters = eastingInMeters;
		// VehicleState->northingInMeters = northingInMeters;		
	}

	if(WheelEncoderISRCount >= 1)
  {
   	WheelEncoderISRCount = 0;

   	/* Blink LED when wheel encoder counts
   	// Turn on LEDs
		WrPortI(PADR, &PADRShadow, 0xff);
		for(index = 0; index < 5000; index++);

		// Turn off LEDs
		WrPortI(PADR, &PADRShadow, 0x00);
		for(index = 0; index < 5000; index++);
		*/
	}

	// Send vehicle status to PC
	// SendVehicleStatusToPC();
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// COMPARATOR MODULE FUNCTIONS
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Function: CalculateErrorSignal()
//
// Tested? YES. With Rabbit Debugger!
//
// Purpose: Determines how far and what angle the vehicle is from it's
//				destination waypoint.
//    
// Parameters:
//   none
//
// Return Value:
//   
// Side Effects:
//   Must first call InitializePorts().
///////////////////////////////////////////////////////////////////////////////
void CalcErrorSignal(struct VehicleStateType* VehicleState, struct DesiredStateType* DesiredState, struct ErrorSignalType* ErrorSignal)
{
	FLOAT32 deltaEasting;
	FLOAT32 deltaNorthing;
	FLOAT32 de2;
	FLOAT32 dn2;

	de2 = 0;
	dn2 = 0;
	deltaEasting  = 0;
	deltaNorthing = 0;
	
	deltaEasting  = fabs((DesiredState->eastingInMeters - VehicleState->eastingInMeters));
	deltaNorthing = fabs((DesiredState->northingInMeters - VehicleState->northingInMeters));

	printf("desired easting and northing: %f %f\n", DesiredState->eastingInMeters, DesiredState->northingInMeters);
	printf("vehicle easting and northing: %f %f\n", VehicleState->eastingInMeters, VehicleState->northingInMeters);
	printf("deltaEasting: %f Delta Northing: %f\n", deltaEasting, deltaNorthing);
	
	// Calculate the distance between the current location and desired location
	de2 = pow(deltaEasting, 2);
	dn2 = pow(deltaNorthing, 2);
	ErrorSignal->distanceInMeters = sqrt((de2 + dn2));

	//
	// Using compass coordinate system. North is 0 degrees. South is 180 degrees. East is 90 degrees. West is 270 degrees.
	//

	if(VehicleState->gpsHeadingInDeg == -1)
	{
		printf("Cannot calculate angle offset in degrees. Valid GPS heading not available\n");
		ErrorSignal->azimuthInDegrees = -1;	
	}
	else
	{
	// Desired location is in Northeast (Quadrant 1)
	if((DesiredState->eastingInMeters >= VehicleState->eastingInMeters) && (DesiredState->northingInMeters >= VehicleState->northingInMeters))
	{
		ErrorSignal->azimuthInDegrees = (RAD_TO_DEG * atan2(deltaEasting,deltaNorthing));
	}
	// Desired location is in Southeast (Quadrant 4) 
	else if((DesiredState->eastingInMeters >= VehicleState->eastingInMeters) && (DesiredState->northingInMeters < VehicleState->northingInMeters))
	{
		ErrorSignal->azimuthInDegrees = (180 - (RAD_TO_DEG * atan2(deltaEasting,deltaNorthing)));
	}
	// Desired location is in Southwest (Quadrant 3)
	else if((DesiredState->eastingInMeters < VehicleState->eastingInMeters) && (DesiredState->northingInMeters < VehicleState->northingInMeters))
	{
		ErrorSignal->azimuthInDegrees = (270 - (RAD_TO_DEG * atan2(deltaNorthing, deltaEasting)));
	}
	// Desired location is in Northwest (Quadrant 2)
	else if((DesiredState->eastingInMeters < VehicleState->eastingInMeters) && (DesiredState->northingInMeters >= VehicleState->northingInMeters))
	{
		ErrorSignal->azimuthInDegrees = (360 - (RAD_TO_DEG * atan2(deltaEasting,deltaNorthing)));
	}
	
	printf("Angle Offset in Degrees =   %f\n", ErrorSignal->azimuthInDegrees);
	printf("Distance Offset in Meters = %f\n", ErrorSignal->distanceInMeters);
	}
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// CONTROL MODULE FUNCTIONS
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Function: CalcNextServoPositionAndMotorVelocity()
//
// Purpose: To calculate the servo's position and motor velocity.
//
// Parameters:
//   thresh = must be less then or equal to 360
//
// Return Value:
//	 bool = true = within angle. false = outside angle.
//
// Side Effects:
//   none. 
///////////////////////////////////////////////////////////////////////////////
UINT16 inAngleThresh(FLOAT32 ia2, FLOAT32 ia1, FLOAT32 ithresh)
{
	UINT16 amin;
	UINT16 amax;

	UINT16 a1;
	UINT16 a2;
	UINT16 thresh;

	a1 = (UINT16)ia1;
	a2 = (UINT16)ia2;
	thresh = (UINT16)ithresh;
	
	amin = 0;
	amax = 0;
	    
	amin = 0;
	amax = 0;
  
  amax = (a1 + thresh)%360;

  if(a1 < thresh)
  {
    amin = 360 - thresh;

    if(a2 > amin)
    {
      return 1;
    }
    else if(a2 < amax)
    {
      return 0;
    }
    else
    {
      return 0;
    }
  }
  else
  {
    amin = a1 - thresh;

    if((a2 > amin) && (a2 < amax))
    {
      return 1;
    }
    else
    {
      return 0;
    }
  }

	return 0;
}

///////////////////////////////////////////////////////////////////////////////
// Function: SmallestAngleShortestTurn()
//
// Tested? Yes. Using Visual C++.
//
// Purpose: To calculate the servo's position and motor velocity.
//
// Parameters:
//   none
//
// Return Value:
//
// Side Effects:
//   Must first call InitializePorts(). 
///////////////////////////////////////////////////////////////////////////////
UINT16 SmallestAngleShortestTurn(FLOAT32 currentHeadingInDeg, FLOAT32 desiredHeadingInDeg)
{
	FLOAT32 c;
	FLOAT32 d; 
  UINT16 turn;

	c = currentHeadingInDeg;
	d = desiredHeadingInDeg;
  
  if(c < 0 || c > 359 || d < 0 || d > 359)
  {
    printf("Error: INVALID CURRENT HEADING OR DESIRED HEADING.\n");
  }

  // C in First Quadrant
  if((c >= 0) && (c < 90))
  {
    if((d >= 0) && (d < 90))
    {
      if(c < d)
      {
        turn = TURN_LEFT;
      }
      else
      {
        turn = TURN_RIGHT;
      }
    }
    if((d >= 90) && (d < 180))
    {
      turn = TURN_LEFT;
    }
    if((d >= 180) && (d < 270))
    {
      if((d - c) < 180)
      {
        turn = TURN_LEFT;
      }
      else
      {
        turn = TURN_RIGHT;
      }
    }
    if((d >= 270) && (d < 360))
    {
      turn = TURN_RIGHT;
    }
  }

  // C in Second Quadrant
  if((c >= 90) && (c < 180))
  {
    if((d >= 0) && (d < 90))
    {
      turn = TURN_RIGHT;

    }
    if((d >= 90) && (d < 180))
    {
      if(c < d)
      {
        turn = TURN_LEFT;
      }
      else
      {
        turn = TURN_RIGHT;
      }
    }
    if((d >= 180) && (d < 270))
    {
      turn = TURN_LEFT;
    }
    if((d >= 270) && (d < 360))
    {
      if((d - c) < 180)
      {
        turn = TURN_LEFT;
      }
      else
      {
        turn = TURN_RIGHT;
      }
    }
  }

  // C in Third Quadrant
  if((c >= 180) && (c < 270))
  {
    if((d >= 0) && (d < 90))
    {
      if((d - c) < 180)
      {
        turn = TURN_LEFT;
      }
      else
      {
        turn = TURN_RIGHT;
      }
    }
    if((d >= 90) && (d < 180))
    {
      turn = TURN_RIGHT;
    }
    if((d >= 180) && (d < 270))
    {
      if(c < d)
      {
        turn = TURN_LEFT;
      }
      else
      {
        turn = TURN_RIGHT;
      }
    }
    if((d >= 270) && (d < 360))
    {
      turn = TURN_LEFT;
    }
  }

  // C in Fourth Quadrant
  if((c >= 270) && (c < 360))
  {
    if((d >= 0) && (d < 90))
    {
      turn = TURN_LEFT;
    }
    if((d >= 90) && (d < 180))
    {
      if((d - c) < 180)
      {
        turn = TURN_LEFT;
      }
      else
      {
        turn = TURN_RIGHT;
      }
    }
    if((d >= 180) && (d < 270))
    {
      turn = TURN_RIGHT;

    }
    if((d >= 270) && (d < 360))
    {
      if(c < d)
      {
        turn = TURN_LEFT;
      }
      else
      {
        turn = TURN_RIGHT;
      }
    }
  }
 
  if(turn == TURN_LEFT)
  {
    printf("Turn = LEFT\n");
  }
  else
  {
    printf("Turn = RIGHT\n");  
  }

	return turn;
}

///////////////////////////////////////////////////////////////////////////////
// Function: CalcNextServoPositionAndMotorVelocity()
//
// Purpose: To calculate the servo's position and motor velocity.
//
// Parameters:
//   none
//
// Return Value:
//
// Side Effects:
//   Must first call InitializePorts(). 
///////////////////////////////////////////////////////////////////////////////
void CalcNextServoPositionAndMotorVelocity(struct VehicleStateType* VehicleState, struct ErrorSignalType* ErrorSignal, UINT8* ServoPosition, UINT8* MotorVelocity)
{
	if(ErrorSignal->distanceInMeters < 3.0)
	{
		*MotorVelocity = STOP;
		printf("STOPPING\n");
	}
	else
	{
		// Maintain a slow, constant speed for testing
		*MotorVelocity = FORWARD_FAST;
	}

	if(ErrorSignal->azimuthInDegrees == -1)
	{
		// Go straight until GPS heading is available
		*ServoPosition = GO_STRAIGHT;
	}
	else
	{	
		// Within 2 degrees error threshold! Go straight
		if(inAngleThresh(VehicleState->gpsHeadingInDeg, ErrorSignal->azimuthInDegrees, 2.0))
		{
			//*ServoPosition = GO_STRAIGHT;	
			//printf("GOING STRAIGHT\n");
			printf("WITHIN ERROR THRESHOLD. NO ADJUSTMENT.\n");
		}
		else
		{
			if(SmallestAngleShortestTurn(VehicleState->gpsHeadingInDeg, ErrorSignal->azimuthInDegrees) == TURN_LEFT)
			{
				*ServoPosition = L1;
				printf("TURNING VEHICLE LEFT\n");
			}
			else
			{
				*ServoPosition = R1;
				printf("TURNING VEHICLE RIGHT\n");
			}
		}
	}
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// ACTUATOR MODULE FUNCTIONS
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Function: ActuateServo
//
// Purpose:
//   To rotate the servo to a specified degree.
//    
// Parameters:
//   UINT8 Degrees - Specifies location servo will move to.
//
// Return Value:
//   none
//
// Side Effects:
//   Must first call InitializePorts().
///////////////////////////////////////////////////////////////////////////////
void ActuateServo(UINT8 ServoPosition)
{
	UINT8 status;

	tx_ssc_buffer[tx_ssc_buffer_new_ptr++] = 255;
	tx_ssc_buffer[tx_ssc_buffer_new_ptr++] = 0;
	tx_ssc_buffer[tx_ssc_buffer_new_ptr++] = ServoPosition;	

	// No need to implement circular buffer because pointer
	// is char and buffer is char so just keep incrementing
	// pointer and it will automatically roll over :)
	
	// read serial port C status register
 	status = RdPortI(SCSR);	

 	if(!(status&TRANSMITTER_BUSY))
 	{
 		// if the tx is idle, this will KICK START the transfer
 		WrPortI(SCDR, NULL, tx_ssc_buffer[tx_ssc_buffer_current_ptr++]);
 	}	
}

///////////////////////////////////////////////////////////////////////////////
// Function: ActuateMotor
//
// Purpose:
//   Accelerates the motor to the specified velocity.
//    
// Parameters:
//   UINT8 Velocity - Specifies the velocity of the motor.
//
// Return Value:
//   none
//
// Side Effects:
//   Must first call InitializePorts().
///////////////////////////////////////////////////////////////////////////////
void ActuateMotor(UINT8 MotorVelocity)
{
	UINT8 status;

	tx_ssc_buffer[tx_ssc_buffer_new_ptr++] = 255;
	tx_ssc_buffer[tx_ssc_buffer_new_ptr++] = 1;
	tx_ssc_buffer[tx_ssc_buffer_new_ptr++] = MotorVelocity;	

	// No need to implement circular buffer because pointer
	// is char and buffer is char so just keep incrementing
	// pointer and it will automatically roll over :)
	
	// read serial port C status register
 	status = RdPortI(SCSR);	

 	if(!(status&TRANSMITTER_BUSY))
 	{
 		// if the tx is idle, this will KICK START the transfer
 		WrPortI(SCDR, NULL, tx_ssc_buffer[tx_ssc_buffer_current_ptr++]);
 	}	
}
