/*
 Name:		spikeencoder.ino
 Created:	12/16/2019 11:55:40 PM
 Author:	wonjo
*/
////////////////////////////////////////////////////////////////////////////
// Spikeling v1.1. By T Baden, Sussex Neuroscience, UK (www.badenlab.org) //
// 2017                                                                   //
// Izhikevich model taken from original paper (2003 IEEE)                 //
////////////////////////////////////////////////////////////////////////////
// Hardware-specific settings
// Swap these if the ESP32is used instead of Arduino Nano
// (for Spikeling 2.0, see GitHub/Manual)
//
//#include   "SettingsArduino.h"
// keep this project (shared in the onedrive). 
// 20210127	 Gyro: replace BNO055 with L3GH20 for faster rate / dual.

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
//#include <Adafruit_BNO055.h>
#include <Adafruit_L3GD20_U.h>
//#include <utility/imumaths.h>

#include <RH_RF95.h>

#include "swingdata.h" //file that stores swing angular position data


#if defined(ARDUINO_SAMD_ZERO)
// Required for Serial on Zero based boards
#define Serial SerialUSB
#endif


//#define IZHIKEVICH_NEURON_ENCODING true

//bool IZHIKEVICH_NEURON_ENCODING = true;

/* for feather32u4
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7
*/

//for feather m0  
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

/* for shield
#define RFM95_CS 10
#define RFM95_RST 9
#define RFM95_INT 7
*/

/* Feather 32u4 w/wing
#define RFM95_RST     11   // "A"
#define RFM95_CS      10   // "B"
#define RFM95_INT     2    // "SDA" (only SDA/SCL/RX/TX have IRQ!)
*/

/* Feather m0 w/wing
#define RFM95_RST     11   // "A"
#define RFM95_CS      10   // "B"
#define RFM95_INT     6    // "D"
*/
//
//#if defined(ESP8266)
//  /* for ESP w/featherwing */ 
//  #define RFM95_CS  2    // "E"
//  #define RFM95_RST 16   // "D"
//  #define RFM95_INT 15   // "B"
//
//#elif defined(ESP32)  
//  /* ESP32 feather w/wing */
//  #define RFM95_RST     27   // "A"
//  #define RFM95_CS      33   // "B"
//  #define RFM95_INT     12   //  next to A
//
//#elif defined(NRF52)  
//  /* nRF52832 feather w/wing */
//  #define RFM95_RST     7   // "A"
//  #define RFM95_CS      11   // "B"
//  #define RFM95_INT     31   // "C"
//  
//#elif defined(TEENSYDUINO)
//  /* Teensy 3.x w/wing */
//  #define RFM95_RST     9   // "A"
//  #define RFM95_CS      10   // "B"
//  #define RFM95_INT     4    // "C"
//#endif

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 403.5 //454 //403.5

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);


/// All messages sent and received by this RH_RF95 Driver conform to this packet format:
///
/// - LoRa mode:
/// - 8 symbol PREAMBLE
/// - Explicit header with header CRC (handled internally by the radio)
/// - 4 octets HEADER: (TO, FROM, ID, FLAGS)
/// - 0 to 251 octets DATA 
/// - CRC (handled internally by the radio)

// I believe minimum symbols per packet : 6(preamble) + 5 (header) + 1 (PL) = 12? (assuming crc = 0) 
// The header type is selected by the ImplictHeaderMode bit found within the RegSymbTimeoutMsb register.
// With SF = 6 selected, implicit header mode is the only mode of operation possible.
// Symbol rate = 500k / (2^6) = 7812.5
// Packets /sec =  7812.5/12 = 651  (Hz)?


volatile boolean spikeout = false;
volatile uint32_t tcc1_per = 60000; // fixed rate pulse out period (DIV8: 60000 is 100Hz, 30000 is 200 Hz). TCC limit: 24 bits. 
volatile uint32_t tcc1_idle_per = 900000;   //  EXP3. 
volatile uint16_t tcc1_pulsewidth = 1500;  // TCC1 pulseout thinkness = 200us now.



#define PIN_PA07			9  // PA07, for fixed pulse output. 
#define PIN_NEURON1			10 // PA18
#define PIN_NEURON2			5 // PA15
#define PIN_LED				13 // PA17 

#define PIN_PUSLEIN			12 // PA19
#define PIN_THIN_PULSEOUT	11 // PA16


#define IS_BNO055 false    //100Hz
#define IS_L3GD20 true    //1kHz
#define IS_MPU6050	false   // up to 8kHz gyro.?
#define IS_MPU6050_raw	false   // up to 8kHz gyro.?
//#define NO_RF_Wireless  true  //  False for live gyro / fixed rate.   True during swing data recording where you need faster rate & playback.


#if IS_BNO055
/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (50)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28), id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
#endif

#if IS_L3GD20
/* Assign a unique ID to this sensor at the same time */
Adafruit_L3GD20_Unified gyro1 = Adafruit_L3GD20_Unified(1);   // 1st
//Adafruit_L3GD20_Unified gyro2 = Adafruit_L3GD20_Unified(2);   // 2nd
#endif

#if IS_MPU6050
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
	#include "I2Cdev.h"
	#include "MPU6050.h"
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	#include "Wire.h"
#endif
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
//MPU6050 accelgyro;
MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t ax, ay, az;
int16_t gx, gy, gz;

// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO

#endif

#if IS_MPU6050_raw
#include <Wire.h>
const int MPU = 0x68; // MPU6050 I2C address
float GyroX, GyroY, GyroZ;
float gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float GyroErrorX, GyroErrorY, GyroErrorZ;
int c = 0;

#endif


#define FIXED_POINT_ARITH	false    // # fixed point vs floating point speed difference. 20210318 WJS 

///////////////////////////////////////////////////////////////////////////
// KEY PARAMETERS TO SET BY USER - IZH Neuron model ////////////////////////
///////////////////////////////////////////////////////////////////////////

int   nosound = 0;    // 0 default, click on spike + digi out port active. 1 switches both off
int   noled = 0;    // 0 default, 1 switches the LED off
int   AnalogInActive = 1;    // default = 1, PORT 3 setting: Is Analog In port in use? Note that this shares the dial with the Syn2 (PORT 2) dial

//float PD_Scaling = 0.5;  // the lower the more sensitive.                       Default = 0.5

//float Synapse_decay = 0.995;// speed of synaptic decay.The difference to 1 matters - the smaller the difference, the slower the decay. Default  = 0.995

#if FIXED_POINT_ARITH
typedef int16_t fixed_t;
#define FSCALE 320

fixed_t fracms = 10;  //10

// set up Neuron behaviour array parameters
//int   nModes = 5; // set this to number of entries in each array. Entries 1 define Mode 1, etc..

// Izhikevich model parameters - for some pre-tested behaviours from the original paper, see bottom of the script
fixed_t Array_a_inv[] = { 50,  50, 50, 50, 50, 5, 10 }; // time scale of recovery variable u. Smaller a gives slower recovery
fixed_t Array_b_inv[] = { 5, 4, 5, 4, -10, 4, 5 }; // recovery variable associated with u. greater b coules it more strongly (basically sensitivity)
fixed_t Array_c[] = { -65 * FSCALE,  -65 * FSCALE,  -50 * FSCALE,  -55 * FSCALE,  -55 * FSCALE,  -65 * FSCALE, -65 * FSCALE }; // after spike reset value
fixed_t Array_d[] = { 6 * FSCALE,  6 * FSCALE,  2 * FSCALE,  0.05 * FSCALE,  6 * FSCALE,   0 * FSCALE, 2 * FSCALE }; // after spike reset of recovery variable
//float Array_e[] = { 14.0,  0.5,  15.0,  0.6,    0,   0 };


int NeuronBehaviour = 0; // DONT USE 3 for fixed point!
// 0: tonic spiking
// 1: phasic spiking
// 2: tonic bursting
// 3: phasic bursting
// 4: Class 1  (x) 
// 5 : Class 2  (x)
// 6: fast spiking 


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MAIN PROGRAMME - ONLY CHANGE IF YOU KNOW WHAT YOU ARE DOING !                                                                       //
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////
// Setup variables required to drive the model
fixed_t I_total;           // Total input current to the model
fixed_t I_Synapse1, I_Synapse2;         // Total synaptic current of both synapses
fixed_t v, v2; // voltage in Iziekevich model
fixed_t u, u2; // recovery variable in Iziekevich model


// initialise state variables for different inputs
boolean spike = false;


//output_t Output; // output structure for plotting
String   OutputStr;

#if IS_L3GD20
fixed_t gyro_scale_orig = 2//30 ; // gyro values typically < 1000. , if 150,too easily saturate to max (~100Hz)
#elif ISMPU6050
int gyro_scale_orig = 1;
#endif


#else
float timestep_ms = 0.1;  // default 0.1. This is the "intended" refresh rate of the model.
							  // Note that it does not actually run this fast as the Arduino cannot execute the...
							  // ...full script at this rate.  Instead, it will run at 333-900 Hz, depending on settings (see top)

// set up Neuron behaviour array parameters
//int   nModes = 5; // set this to number of entries in each array. Entries 1 define Mode 1, etc..

// Izhikevich model parameters - for some pre-tested behaviours from the original paper, see bottom of the script
float Array_a[] = { 0.02, 0.02, 0.02, 0.02, 0.02, 0.2, 0.1}; // time scale of recovery variable u. Smaller a gives slower recovery
float Array_b[] = { 0.20, 0.25, 0.20, 0.25, -0.1, 0.26,0.2 }; // recovery variable associated with u. greater b coules it more strongly (basically sensitivity)
int   Array_c[] = { -65,  -65,  -50,  -55,  -55,  -65, -65 }; // after spike reset value
float Array_d[] = { 6.0,  6.0,  2.0,  0.05,  6.0,   0, 2  }; // after spike reset of recovery variable
//float Array_e[] = { 14.0,  0.5,  15.0,  0.6,    0,   0 };


int NeuronBehaviour = 4; // 0:8 for different modes, cycled by button
// 0: tonic spiking
// 1: phasic spiking
// 2: tonic bursting
// 3: phasic bursting
// 4: Class 1  (x) 
// 5 : Class 2  (x)
// 6: fast spiking 

// From Iziekevich.org - see also https://www.izhikevich.org/publications/figure1.pdf:
//      0.02      0.2     -65      6       14 ;...    % tonic spiking
//      0.02      0.25    -65      6       0.5 ;...   % phasic spiking
//      0.02      0.2     -50      2       15 ;...    % tonic bursting
//      0.02      0.25    -55     0.05     0.6 ;...   % phasic bursting
//      0.02      0.2     -55     4        10 ;...    % mixed mode
//      0.01      0.2     -65     8        30 ;...    % spike frequency adaptation
//      0.02      -0.1    -55     6        0  ;...    % Class 1
//      0.2       0.26    -65     0        0  ;...    % Class 2
//      0.02      0.2     -65     6        7  ;...    % spike latency
//      0.05      0.26    -60     0        0  ;...    % subthreshold oscillations
//      0.1       0.26    -60     -1       0  ;...    % resonator
//      0.02      -0.1    -55     6        0  ;...    % integrator
//      0.03      0.25    -60     4        0;...      % rebound spike
//      0.03      0.25    -52     0        0;...      % rebound burst
//      0.03      0.25    -60     4        0  ;...    % threshold variability
//      1         1.5     -60     0      -65  ;...    % bistability
//        1       0.2     -60     -21      0  ;...    % DAP
//      0.02      1       -55     4        0  ;...    % accomodation
//     -0.02      -1      -60     8        80 ;...    % inhibition-induced spiking
//     -0.026     -1      -45     0        80];       % inhibition-induced bursting
//     0.1		  0.2     -65	  2					% fast spiking 

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MAIN PROGRAMME - ONLY CHANGE IF YOU KNOW WHAT YOU ARE DOING !                                                                       //
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////
// Setup variables required to drive the model
float I_total;           // Total input current to the model
float I_Synapse1, I_Synapse2;         // Total synaptic current of both synapses
float Synapse1Ampl;      // Synapse 1 efficacy
float v, v2; // voltage in Iziekevich model
float u, u2; // recovery variable in Iziekevich model
float sensor_playback; // when playing back the sensor data to "simulate" the walking during quick sensory stim test.




// initialise state variables for different inputs
boolean spike = false;
int     buttonState = 0;
int     SpikeIn1State = 0;
int     Stimulator_Val = 0;

int DigiOutStep = 0;     // stimestep counter for stimulator mode
int Stim_State = 0;      // State of the internal stimulator


//output_t Output; // output structure for plotting
String   OutputStr;

#if IS_L3GD20
int gyro_scale_orig = 70;//30;//30 ; // gyro values typically < 1000. , if 150,too easily saturate to max (~100Hz)
#elif ISMPU6050
int gyro_scale = 1;
#endif

#endif 




//int startMicros = micros();

//serial button options 
String runmode = "live"; // live or playback.
String encoding_mode = "IZNeuron";  // IZNeuron or fixed rate.
String cadence_mode = "None"; // e.g. cad50, cad50_kick1st.
//String transmission_mode = "rf_wireless";  //  wireless mode

int  n = 0;

float playback_scale = 1.05;


//// display info for L3GD20 gyros.
//void displaySensorDetails(void)
//{
//	sensor_t sensor;
//	gyro1.getSensor(&sensor);
//	SerialUSB.println("------------------------------------");
//	SerialUSB.print("Sensor:       "); SerialUSB.println(sensor.name);
//	SerialUSB.print("Driver Ver:   "); SerialUSB.println(sensor.version);
//	SerialUSB.print("Unique ID:    "); SerialUSB.println(sensor.sensor_id);
//	SerialUSB.print("Max Value:    "); SerialUSB.print(sensor.max_value); SerialUSB.println(" rad/s");
//	SerialUSB.print("Min Value:    "); SerialUSB.print(sensor.min_value); SerialUSB.println(" rad/s");
//	SerialUSB.print("Resolution:   "); SerialUSB.print(sensor.resolution); SerialUSB.println(" rad/s");
//	SerialUSB.println("------------------------------------");
//	SerialUSB.println("");
//	delay(500);
//	/*gyro2.getSensor(&sensor);
//	SerialUSB.println("------------------------------------");
//	SerialUSB.print("Sensor:       "); SerialUSB.println(sensor.name);
//	SerialUSB.print("Driver Ver:   "); SerialUSB.println(sensor.version);
//	SerialUSB.print("Unique ID:    "); SerialUSB.println(sensor.sensor_id);
//	SerialUSB.print("Max Value:    "); SerialUSB.print(sensor.max_value); SerialUSB.println(" rad/s");
//	SerialUSB.print("Min Value:    "); SerialUSB.print(sensor.min_value); SerialUSB.println(" rad/s");
//	SerialUSB.print("Resolution:   "); SerialUSB.print(sensor.resolution); SerialUSB.println(" rad/s");
//	SerialUSB.println("------------------------------------");
//	SerialUSB.println("");
//	delay(500);*/
//
//}
//



////////////////////////////////////////////////////////////////////////////
// SETUP (this only runs once at when the Arduino is initialised) //////////
////////////////////////////////////////////////////////////////////////////


void setup() {

	//while (!SerialUSB) {
	//	delay(1);
	//}


	rf95_setup();  // setup radio head communication 

	//TCC common clock setting 
	GCLK->GENDIV.reg = GCLK_GENDIV_DIV(1) |          // Divide the 48MHz clock source by divisor 10: 48MHz/1=48MHz
		GCLK_GENDIV_ID(4);            // Select Generic Clock (GCLK) 4
	while (GCLK->STATUS.bit.SYNCBUSY);               // Wait for synchronization

	GCLK->GENCTRL.reg = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOWs
		GCLK_GENCTRL_GENEN |         // Enable GCLK4
		GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
		GCLK_GENCTRL_ID(4);          // Select GCLK4
	while (GCLK->STATUS.bit.SYNCBUSY);               // Wait for synchronization

	// Feed GCLK4 to TCC0 and TCC1
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TCC0 and TCC1
		GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
		GCLK_CLKCTRL_ID_TCC0_TCC1;   // Feed GCLK4 to TCC0 and TCC1


	while (GCLK->STATUS.bit.SYNCBUSY);                // Wait for synchronization  

	// gyroscope setup 

#if IS_BNO055
	// BN0055 Setting 
	/* Initialise the sensor */
	if (!bno.begin())
	{
		/* There was a problem detecting the BNO055 ... check your connections */
		Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
		while (1);
	}

	delay(500);

	/* Display some basic information on this sensor */
	displaySensorDetails();

	/* Optional: Display current status */
	displaySensorStatus();

	bno.setExtCrystalUse(true);
	//end of BNO055 setting 
#endif

#if IS_L3GD20
	SerialUSB.begin(115200);
	 /* Enable auto-ranging */
	gyro1.enableAutoRange(true);
	//gyro2.enableAutoRange(true);
	/* Initialise the sensor */
	if (!gyro1.begin(GYRO_RANGE_500DPS))
	{
		/* There was a problem detecting the L3GD20 ... check your connections */
		SerialUSB.println("Ooops, (gyro1) no L3GD20 detected ... Check your wiring!");
		while (1);
	}
	///* Initialise the sensor */
	//if (!gyro2.begin(GYRO_RANGE_500DPS))
	//{
	//	/* There was a problem detecting the L3GD20 ... check your connections */
	//	SerialUSB.println("Ooops, (gyro2) no L3GD20 detected ... Check your wiring!");
	//	while (1);
	//}
	/* Display some basic information on this sensor */
	//displaySensorDetails();

#endif
	
#if IS_MPU6050
	// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	Wire.begin();
	//Wire.setClock(200000); // 200kHz I2C clock. (max 400k in spec sheet but it gets 0's) Comment this line if having compilation difficulties

#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
	Fastwire::setup(400, true);
#endif
	SerialUSB.begin(38400);

	// initialize device
	SerialUSB.println("Initializing I2C devices...");
	accelgyro.initialize();

	// verify connection
	SerialUSB.println("Testing device connections...");
	SerialUSB.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
	// use the code below to change accel/gyro offset values

	//accelgyro.setXGyroOffset(220);
	//accelgyro.setYGyroOffset(76);
	//accelgyro.setZGyroOffset(-85);
	

  /*
  SerialUSB.println("Updating internal sensor offsets...");
  // -76	-2359	1688	0	0	0
  SerialUSB.print(accelgyro.getXAccelOffset()); SerialUSB.print("\t"); // -76
  SerialUSB.print(accelgyro.getYAccelOffset()); SerialUSB.print("\t"); // -2359
  SerialUSB.print(accelgyro.getZAccelOffset()); SerialUSB.print("\t"); // 1688
  SerialUSB.print(accelgyro.getXGyroOffset()); SerialUSB.print("\t"); // 0
  SerialUSB.print(accelgyro.getYGyroOffset()); SerialUSB.print("\t"); // 0
  SerialUSB.print(accelgyro.getZGyroOffset()); SerialUSB.print("\t"); // 0
  SerialUSB.print("\n");
  accelgyro.setXGyroOffset(220);
  accelgyro.setYGyroOffset(76);
  accelgyro.setZGyroOffset(-85);
  SerialUSB.print(accelgyro.getXAccelOffset()); SerialUSB.print("\t"); // -76
  SerialUSB.print(accelgyro.getYAccelOffset()); SerialUSB.print("\t"); // -2359
  SerialUSB.print(accelgyro.getZAccelOffset()); SerialUSB.print("\t"); // 1688
  SerialUSB.print(accelgyro.getXGyroOffset()); SerialUSB.print("\t"); // 0
  SerialUSB.print(accelgyro.getYGyroOffset()); SerialUSB.print("\t"); // 0
  SerialUSB.print(accelgyro.getZGyroOffset()); SerialUSB.print("\t"); // 0
  SerialUSB.print("\n");
  */
#endif

#if IS_MPU6050_raw
	SerialUSB.begin(38400);
	Wire.begin();
	Wire.setClock(200000); // Initialize comunication
	Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
	Wire.write(0x6B);                  // Talk to the regisster 6B
	Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
	Wire.endTransmission(true);        //end the transmission
	//calculate_IMU_error();
	delay(20);
#endif



	//initializeHardware(); // Set all the PINs
#if FIXED_POINT_ARITH
	v = -65 *FSCALE;
	u = 0;
	v2 = -65*FSCALE;
	u2 = 0;
#else
	v = -65;
	u = 0;
	v2 = -65;
	u2 = 0;
#endif

	// setting for IZ NEURON AND IS_GYRO
	// DISABLE FIXED RATE OUTPUT  (control mux to disable PWM and enable digitalWrite
	PORT->Group[g_APinDescription[PIN_NEURON1].ulPort].PMUX[g_APinDescription[PIN_NEURON1].ulPin >> 1].bit.PMUXE = PORT_PMUX_PMUXE_B_Val; //PA18
	PORT->Group[g_APinDescription[PIN_NEURON2].ulPort].PMUX[g_APinDescription[PIN_NEURON2].ulPin >> 1].bit.PMUXO = PORT_PMUX_PMUXO_B_Val; //PA15

	// IZ NEURON ENCODING
	

	pinMode(PIN_LED, OUTPUT);
	pinMode(PIN_PUSLEIN, INPUT); // 
	attachInterrupt(PIN_PUSLEIN, interrupt_pulsegen_pulsein, RISING); // not used?

	pinMode(PIN_PA07, OUTPUT);
	pinMode(PIN_THIN_PULSEOUT, OUTPUT); // 
	oneshot_TCC0_thinPulse_setup();  // 
	fixed_rate_pulse_out_setup();

}


int16_t packetnum = 0;  // packet counter, we increment per xmission


void threshold_based_state_decoder() {


}

// classification using a hidden Markov model. 
void continuous_HMM_state_decoder() {


}

void rf_send() {
	//SerialUSB.println("Sending to rf95_server");
 // Send a message to rf95_server


 // ** rf95.send() only when there is a spike.  **//
	if (spikeout) {
		digitalWrite(PIN_LED, HIGH);
		spikeout = false;
		//noInterrupts();
		char radiopacket = '1';
		//itoa(packetnum++, radiopacket, 10);
		SerialUSB.print("Sending..... "); SerialUSB.println(radiopacket);
		//radiopacket[0] = 0;


		rf95.send((uint8_t*)radiopacket, 1); 
		digitalWrite(PIN_LED, LOW);
		//interrupts();
	}
	// working version 20210320 - sends a number incremented
	//if (spikeout) {
	//	digitalWrite(PIN_LED, HIGH);
	//	spikeout = false;
	//	//noInterrupts();
	//	char radiopacket[4] = " ";
	//	itoa(packetnum++, radiopacket, 10);
	//	SerialUSB.print("Sending... "); SerialUSB.println(radiopacket);
	//	radiopacket[2] = 0;


	//	rf95.send((uint8_t*)radiopacket, 3);
	//	digitalWrite(PIN_LED, LOW);
	//	//interrupts();
	//}

	//  SerialUSB.println("Waiting for packet to complete..."); delay(10);
	//  rf95.waitPacketSent();
	//  // Now wait for a reply
	//  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
	//  uint8_t len = sizeof(buf);
	//
	//  SerialUSB.println("Waiting for reply..."); delay(10);
	//  if (rf95.waitAvailableTimeout(1000))
	//  { 
	//    // Should be a reply message for us now   
	//    if (rf95.recv(buf, &len))
	//   {
	//      SerialUSB.print("Got reply: ");
	//      SerialUSB.println((char*)buf);
	//      SerialUSB.print("RSSI: ");
	//      SerialUSB.println(rf95.lastRssi(), DEC);    
	//    }
	//    else
	//    {
	//      SerialUSB.println("Receive failed");
	//    }
	//  }
	//  else
	//  {
	//    SerialUSB.println("No reply, is there a listener around?");
	//  }
	//  delay(1000);

}

unsigned int time_us;
unsigned int playback_starttime;
sensors_event_t event;  // L3GD30
int gn = 0; // counter (gyro n)
int gyro_scale;
bool wired_mode = false; // default is wireless. 
bool low_freq_during_idle = true; // default: idle state (stationary state) doesn't generate feedback. 

void loop() {
	time_us = micros();

	gyro_scale = gyro_scale_orig;

	// check system time in microseconds
	//unsigned long currentMicros = micros() - startMicros;

	if (!wired_mode) {
		rf_send();
	}

	
	// serial connection 
	char command;
	char commands[4];

	int ua = SerialUSB.available();

	if (ua > 0)
	{
		//std::vector <char> buffer(ua); 
		command = SerialUSB.read();


		//int rlen = SerialUSB.readBytes(commands, 4); //
		////SerialUSB.println(commands);
		//if (rlen > 1) { //this is a frequency setting. 
		//	for (int i = 0; i < rlen; i++)
		//		SerialUSB.print(commands[i]);
		//	SerialUSB.println();
		//	commands[rlen] = '\0';
		//	
		//	setPulseFrequency(commands);
		//}
		//else {// one letter commands


		// transmission mode
		if (command == 'V') {
			// IZ Neuron encoding 
			wired_mode = false;
			SerialUSB.println(F("Transmission mode: Rf wireless"));
		}
		else if (command == 'W') {
			// fixed rate encoding
			wired_mode = true;
			SerialUSB.println(F("Transmissoin mode: wired"));
		}


			// encoding mode setting 
		if (command == 'A') {
			// IZ Neuron encoding 
			encoding_mode = "IZNeuron";
			SerialUSB.println(F("Encoding mode: IZN"));
		}
		else if (command == 'B') {
			// fixed rate encoding
			encoding_mode = "fixedrate";
			SerialUSB.println(F("Encoding mode:fixedrate"));
		}
		else {
			SerialUSB.println(F("Unknown encoding mode"));
		}

		// run mode setting 
		if (command == 'X') {
			// live, realtime mode
			runmode = "live";
			SerialUSB.println(F("Run mode: live"));
		}
		else if (command == 'Y') {
			// playback mode
			runmode = "playback";
			SerialUSB.println(F("Run mode: playback"));
		}
		else {
			SerialUSB.println(F("Unknown runmode"));
		}


		// idle definitione
		if (command == 'T') {
			low_freq_during_idle = false;
			SerialUSB.println(F("Idle: silent"));
		}
		else if (command == 'U') {
			// fixed rate encoding
			low_freq_during_idle = true;
			SerialUSB.println(F("Idle: low freq"));
		}




		if (runmode == "playback") {
			switch (command) {
			case 'C':
				cadence_mode = "cad50";
				playback_starttime = micros();
				gyro_scale *= playback_scale; // playback can have high frequency
				break;

			case 'D':    // swing2
				cadence_mode = "cad60";
				playback_starttime = micros();
				gyro_scale *= playback_scale; // playback can have high frequency
				break;

			case 'E':    // swing3
				cadence_mode = "cad90";
				playback_starttime = micros();
				gyro_scale *= playback_scale; // playback can have high frequency
				break;

			case 'F':    // swing4
				cadence_mode = "cad50_k1"; // cad50_kick1st 
				playback_starttime = micros();
				gyro_scale *= playback_scale; // playback can have high frequency
				break;

			case 'G':   // swing5
				cadence_mode = "cad60_k1"; //// cad60_kick1st
				playback_starttime = micros();
				gyro_scale *= playback_scale; // playback can have high frequency
				break;
			case 'H':   // swing6
				cadence_mode = "cad50_k2"; // cad50_kick2nd 
				playback_starttime = micros();
				gyro_scale *= playback_scale; // playback can have high frequency
				break;
			case 'I':   // swing7
				cadence_mode = "cad60_k2"; // cad60_kick2nd 
				playback_starttime = micros();
				gyro_scale *= playback_scale; // playback can have high frequency
				break;
			case 'J':   // swing8
				cadence_mode = "cad50_tr"; // cad50_trip 
				playback_starttime = micros();
				gyro_scale *= playback_scale; // playback can have high frequency
				break;
			case 'K':   // swing9
				cadence_mode = "cad60_tr"; // cad60_Trip 
				playback_starttime = micros();
				gyro_scale *= playback_scale; // playback can have high frequency
				break;
			default:
				SerialUSB.println(F("Cadence mode not defined")); //
				break;
			}
		}

		//if (command == 'A') {  // gyro
		//	runmode = "gyro";
		//}
		//else if (command == 'B') { // fixed rate
		//	runmode = "fixedrate";
		//}
		//else if (command == 'C') { // swing 1
		//	runmode = "cad50";
		//	playback_starttime = micros();
		//	gyro_scale *= 1; // playback can have high frequency
		//}
		//else if (command == 'D') {   // swing2
		//	runmode = "cad60";  
		//	playback_starttime = micros();
		//	gyro_scale *= 1; // playback can have high frequency

		//}
		//else if (command == 'E') {   // swing3
		//	runmode = "cad90";
		//	playback_starttime = micros();
		//	gyro_scale *= 1; // playback can have high frequency

		//}
		//else if (command == 'F') {   // swing4
		//	runmode = "cad50_k1"; // cad50_kick1st 
		//	playback_starttime = micros();
		//	gyro_scale *= 1; // playback can have high frequency

		//}
		//else if (command == 'G') {   // swing5
		//	runmode = "cad60_k1"; //// cad60_kick1st
		//	playback_starttime = micros();
		//	gyro_scale *= 1; // playback can have high frequency

		//}
		//else if (command == 'H') {   // swing6
		//	runmode = "cad50_k2"; // cad50_kick2nd 
		//	playback_starttime = micros();
		//	gyro_scale *= 1; // playback can have high frequency

		//}
		//else if (command == 'I') {   // swing7
		//	runmode = "cad60_k2"; // cad60_kick2nd 
		//	playback_starttime = micros();
		//	gyro_scale *= 1; // playback can have high frequency

		//}
		//else if (command == 'J') {   // swing8
		//	runmode = "cad50_tr"; // cad50_trip 
		//	playback_starttime = micros();
		//	gyro_scale *= 1; // playback can have high frequency

		//}
		//else if (command == 'K') {   // swing9
		//	runmode = "cad60_tr"; // cad60_Trip 
		//	playback_starttime = micros();
		//	gyro_scale *= 1; // playback can have high frequency

		//}
		//else { //
		//	SerialUSB.println("Wrong serial command"); //


		//}
		
	}
	//SerialUSB.println(size_of_array); //


	if (runmode == "live") { // live or playback mode. 

#if IS_BNO055

		/* Get a new sensor event */
		sensors_event_t event;
		bno.getEvent(&event);
		imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

		/* Display the floating point data */
		//SerialUSB.print("X: ");
		//SerialUSB.print(event.orientation.x, 4);
		//SerialUSB.print(gyro.x());
		//SerialUSB.print("\tY: ");
		//SerialUSB.print(event.orientation.y, 4);
		//SerialUSB.print(gyro.y());
		//SerialUSB.print("\tZ: ");
		//SerialUSB.print(event.orientation.z, 4);
		//SerialUSB.print(gyro.z());

		I_Synapse1 = int(gyro.y() * gyro_scale);
#endif
#if IS_L3GD20

		/* Get a new sensor event */
		gyro1.getEvent(&event);// takes most of the delay (almost 1ms!)  Must try MPU6050? 8Khz? 




	//sensors_event_t event2;
	//gyro2.getEvent(&event2);

	/* Display the results (speed is measured in rad/s) */
	//SerialUSB.print("X1: "); SerialUSB.print(event.gyro.x); SerialUSB.print("  ");
	//SerialUSB.print("Y1: "); SerialUSB.print(event.gyro.y); SerialUSB.print("  ");
	//SerialUSB.print("Z1: "); SerialUSB.print(event.gyro.z); SerialUSB.print("  ");

	//SerialUSB.print("X2: "); SerialUSB.print(event2.gyro.x); SerialUSB.print("  ");
	//SerialUSB.print("Y2: "); SerialUSB.print(event2.gyro.y); SerialUSB.print("  ");
	//SerialUSB.print("Z2: "); SerialUSB.print(event2.gyro.z); SerialUSB.print("  ");
	//SerialUSB.println("rad/s ");
	 // L3GD30 adjustment
		//SerialUSB.println(event.gyro.z); // this slows by 300us - but need this in swing data recording.
		//SerialUSB.print(",");
		
		I_Synapse1 = event.gyro.z * gyro_scale  ; // cheap
#endif

#if IS_MPU6050
		//accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
		accelgyro.getRotation(&gx, &gy, &gz);
		//I_Synapse1 = int(gz * gyro_scale); // cheap
		I_Synapse1 = gz * gyro_scale; // cheap
#endif

#if IS_MPU6050_raw
		Wire.beginTransmission(MPU);
		Wire.write(0x45); // Gyro data first register address 0x43
		Wire.endTransmission(false);
		Wire.requestFrom(MPU, 2, true); // Read 4 registers total, each axis value is stored in 2 registers
		//GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
	   // GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
		GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
		// Correct the outputs with the calculated error values
		//GyroX = GyroX + 0.56; // GyroErrorX ~(-0.56)
		//GyroY = GyroY - 2; // GyroErrorY ~(2)
		GyroZ = GyroZ + 0.79; // GyroErrorZ ~ (-0.8)


		I_Synapse1 = GyroZ * gyro_scale; // cheap
		SerialUSB.println(I_Synapse1);

#endif

#ifdef OUTPUT_READABLE_ACCELGYRO
// display tab-separated accel/gyro x/y/z values
		//SerialUSB.print("a/g:\t");
		//SerialUSB.print(ax); SerialUSB.print("\t");
		//SerialUSB.print(ay); SerialUSB.print("\t");
		//SerialUSB.print(az); SerialUSB.print("\t");
		//SerialUSB.print(gx); SerialUSB.print("\t");
		//SerialUSB.print(gy); SerialUSB.print("\t");
		SerialUSB.println(gz);
#endif


		//SerialUSB.print(I_Synapse1);
		//SerialUSB.print(",");
	} // end of realtime mode
	else if (runmode == "playback") {
		gyro_scale *= 2; // increase the rate in the playback mode when the bottle neck from RFM is removed.
						// instead of max 130Hz if live mode, this can go up to 300 Hz (or more). 

		if (cadence_mode == "stop") { // do nothing 
			I_Synapse1 = 0;
		}
		else if (cadence_mode == "cad50") { // swing 1
			// load one value per loop.

			sensor_playback = cad50_1swing_max2p5[n];
			SerialUSB.println(sensor_playback);
			I_Synapse1 = sensor_playback * gyro_scale;
			if (n == sizeof(cad50_1swing_max2p5) / sizeof(int) - 1) {
				cadence_mode = "stop";
				n = 0;
				SerialUSB.print("Elapsed time: ");
				SerialUSB.println(micros() - playback_starttime);
			}
			n = n + 1;

		}
		else if (cadence_mode == "cad60") { // swing 2
		   // load one value per loop.

			sensor_playback = cad60_1swing_max4p2[n];
			SerialUSB.println(sensor_playback);
			I_Synapse1 = sensor_playback * gyro_scale;
			if (n == sizeof(cad60_1swing_max4p2) / sizeof(int) - 1) {
				cadence_mode = "stop";
				n = 0;
				SerialUSB.print("Elapsed time: ");
				SerialUSB.println(micros() - playback_starttime);
			}
			n = n + 1;
		}
		else if (cadence_mode == "cad90") { // swing 3
	// load one value per loop.

			sensor_playback = cad90_1swing_max5p3[n];
			SerialUSB.println(sensor_playback);
			I_Synapse1 = sensor_playback * gyro_scale;
			if (n == sizeof(cad90_1swing_max5p3) / sizeof(int) - 1) {
				cadence_mode = "stop";
				n = 0;
				SerialUSB.print("Elapsed time: ");
				SerialUSB.println(micros() - playback_starttime);
			}
			n = n + 1;
		}
		else if (cadence_mode == "cad50_k1") { // swing 4
			// load one value per loop.

			sensor_playback = cad50_kick1st_1swing_max3p7[n];
			SerialUSB.println(sensor_playback);
			I_Synapse1 = sensor_playback * gyro_scale;
			if (n == sizeof(cad50_kick1st_1swing_max3p7) / sizeof(int) - 1) {
				cadence_mode = "stop";
				n = 0;
				SerialUSB.print("Elapsed time: ");
				SerialUSB.println(micros() - playback_starttime);
			}
			n = n + 1;
		}
		else if (cadence_mode == "cad60_k1") { // swing 5
			// load one value per loop.

			sensor_playback = cad60_kick1st_1swing_max4p6[n];
			SerialUSB.println(sensor_playback);
			I_Synapse1 = sensor_playback * gyro_scale;
			if (n == sizeof(cad60_kick1st_1swing_max4p6) / sizeof(int) - 1) {
				cadence_mode = "stop";
				n = 0;
				SerialUSB.print("Elapsed time: ");
				SerialUSB.println(micros() - playback_starttime);
			}
			n = n + 1;
		}
		else if (cadence_mode == "cad50_k2") { // swing 6
			// load one value per loop.

			sensor_playback = cad50_kick2nd_1swing_max6p1[n];
			SerialUSB.println(sensor_playback);
			I_Synapse1 = sensor_playback * gyro_scale;
			if (n == sizeof(cad50_kick2nd_1swing_max6p1) / sizeof(int) - 1) {
				cadence_mode = "stop";
				n = 0;
				SerialUSB.print("Elapsed time: ");
				SerialUSB.println(micros() - playback_starttime);
			}
			n = n + 1;
		}
		else if (cadence_mode == "cad60_k2") { // swing 7
			// load one value per loop.

			sensor_playback = cad60_kick2nd_1swing_max7p3[n];
			SerialUSB.println(sensor_playback);
			I_Synapse1 = sensor_playback * gyro_scale;
			if (n == sizeof(cad60_kick2nd_1swing_max7p3) / sizeof(int) - 1) {
				cadence_mode = "stop";
				n = 0;
				SerialUSB.print("Elapsed time: ");
				SerialUSB.println(micros() - playback_starttime);
			}
			n = n + 1;
		}
		else if (cadence_mode == "cad50_tr") { // swing 8
			// load one value per loop.

			sensor_playback = cad50_trip_1swing_max3p8[n];
			SerialUSB.println(sensor_playback);
			I_Synapse1 = sensor_playback * gyro_scale;
			if (n == sizeof(cad50_trip_1swing_max3p8) / sizeof(int) - 1) {
				cadence_mode = "stop";
				n = 0;
				SerialUSB.print("Elapsed time: ");
				SerialUSB.println(micros() - playback_starttime);
			}
			n = n + 1;
		}
		else if (cadence_mode == "cad60_tr") { // swing 9
			// load one value per loop.

			sensor_playback = cad60_trip_1swing_max3p6[n];
			SerialUSB.println(sensor_playback);
			I_Synapse1 = sensor_playback * gyro_scale;
			if (n == sizeof(cad60_trip_1swing_max3p6) / sizeof(int) - 1) {
				cadence_mode = "stop";
				n = 0;
				SerialUSB.print("Elapsed time: ");
				SerialUSB.println(micros() - playback_starttime);
			}
			n = n + 1;
		}
		else {
			SerialUSB.println("cadence_mode not found");
			SerialUSB.print("Elapsed time: ");
			SerialUSB.println(micros() - playback_starttime);

		}
	}
	//I_Synapse2 = int(event2.gyro.y * gyro_scale);

	/*SerialUSB.print("I1: "); SerialUSB.print(I_Synapse1); SerialUSB.print("  ");
	SerialUSB.print("I2: "); SerialUSB.print(I_Synapse2); SerialUSB.print("  ");
	SerialUSB.println(" AU ");*/
//#endif

	I_Synapse1 = (I_Synapse1 > 0) ? I_Synapse1 : 0;  // positive flextion only.

	if (encoding_mode == "IZNeuron") { // the same for live and playback mode. 

		//SerialUSB.println(I_Synapse1);

		//I_Synapse2 = (I_Synapse2 > 0) ? I_Synapse2 : 0;  // positive flextion only.
	/*	if (I_Synapse1 > 100) {
			SerialUSB.print(" I_Synapse1: ");
			SerialUSB.println(I_Synapse1);
		}*/

		// Decay all synaptic current towards zero
		//I_Synapse1 *= Synapse_decay;

		// compute Izhikevich model
		// ****** first gyro **********  

#if FIXED_POINT_ARITH
		fixed_t I_total = I_Synapse1 * FSCALE; // Add up all current sources
		v = v + ((v * v) / 25 / FSCALE + 5 * v + 140 * FSCALE - u + I_total) / fracms;  //>>7 approximating /100, >>3 for /10  (time step adjust)
																						//v = v + ((v * v) / 25 / FSCALE + 5 * v + 140 * FSCALE - u + I_total) >> 3; // / fracms;  //>>7 approximating /100, >>3 for /10  (time step adjust)

		u = u + ((v / Array_b_inv[NeuronBehaviour] - u) / Array_a_inv[NeuronBehaviour]) / fracms; //>>13 for approximating / 10^4 from Array_a, Array_b adjustment) , >>3 for /10
		//u = u + ((v / Array_b_inv[NeuronBehaviour] - u) / Array_a_inv[NeuronBehaviour]) >> 3; //>>13 for approximating / 10^4 from Array_a, Array_b adjustment) , >>3 for /10

//		SerialUSB.print( ((v* v) >> 13 + 5 * v + (1 << 15 + 1 << 13 + 1 << 12) - u + I_total) >> 3);
//		SerialUSB.print(", ");
	//	SerialUSB.println(((v / Array_b_inv[NeuronBehaviour] - u) / Array_a_inv[NeuronBehaviour]) >> 3);

		if (v >= 30.0 * FSCALE) {
			v = Array_c[NeuronBehaviour];
			u += Array_d[NeuronBehaviour];
			//digitalWrite(PIN_NEURON1, HIGH);
			//SerialUSB.println("\tspike 1");
			spikeout = true;

			////**  rf_send()
			//digitalWrite(PIN_LED, HIGH);
			////spikeout = false;
			////noInterrupts();
			//char radiopacket = 1;
			////itoa(packetnum++, radiopacket, 10);
			//SerialUSB.print("Sending.... "); SerialUSB.println(radiopacket);
			////radiopacket[3] = 0;

			//digitalWrite(PIN_LED, LOW);
			//rf95.send((uint8_t*)radiopacket, 1);
			//
			////interrupts();
			//// end of rf_send


			// the following works for thin pulse out.  
			//TCC0->CTRLBSET.reg = TCC_CTRLBSET_CMD_RETRIGGER;         // Retrigger the timer's One/Multi-Shot pulse
			//while (TCC0->SYNCBUSY.bit.CTRLB);                        // Wait for synchronization
		}

		else {
			//digitalWrite(PIN_NEURON1, LOW);
		}


		//if (v <= -90*FSCALE) { v = -90 *FSCALE; } // prevent from analog out (below) going into overdrive - but also means that it will flatline at -90. Change the "90" in this line and the one below if want to
		//int AnalogOutValue = (v + 90) * 2;
	}
	else { // fixed rate mode

// the following works for thin pulse out.
		SerialUSB.print("I_Synapse1:");
		SerialUSB.println(I_Synapse1);
		if (I_Synapse1 > 1 * FSCALE) {
			spikeout = true;
		}
		//delay(5); // delay(10): 95-100H. delay(20): 45-50Hz 
	}
#else
		// IZ Neuron calculation -  keep this computation at all time even in the EXP3 mode for loop-delay consistency.
		float I_total = I_Synapse1; // Add up all current sources
		v = v + timestep_ms * (0.04 * v * v + 5 * v + 140 - u + I_total);
		u = u + timestep_ms * (Array_a[NeuronBehaviour] * (Array_b[NeuronBehaviour] * v - u));

		if (wired_mode) {
			digitalWrite(PIN_LED, LOW);
		}

		 if (v >= 30.0) {  // typical IZ Neuron 
			v = Array_c[NeuronBehaviour];
			u += Array_d[NeuronBehaviour];
			//digitalWrite(PIN_NEURON1, HIGH);
			//SerialUSB.println("\tspike 1");
			spikeout = true;
	
			if (wired_mode) {
				digitalWrite(PIN_LED, HIGH);
			}

			// the following works for thin pulse out.  
			TCC0->CTRLBSET.reg = TCC_CTRLBSET_CMD_RETRIGGER;         // Retrigger the timer's One/Multi-Shot pulse
			while (TCC0->SYNCBUSY.bit.CTRLB);                        // Wait for synchronization
		}

		else {
			//digitalWrite(PIN_NEURON1, LOW);
		}

		if (v <= -90) { v = -90; } // prevent from analog out (below) going into overdrive - but also means that it will flatline at -90. Change the "90" in this line and the one below if want to
		//int AnalogOutValue = (v + 90) * 2;


	}
	else { // fixed rate mode.  
		
		// the following works for thin pulse out. 
		//if (I_Synapse1 > 50) {

		//if (event.gyro.z > 1.0) { // if swing speed above certain threshold... fire constant rate.
		SerialUSB.println(I_Synapse1);
		if (I_Synapse1 >  0.4* gyro_scale) {// low enough threshold. 
			spikeout = true;
			// HERE, if playback: turn on the fixed_rate_pulse_out UNTIL the playback ended. If live, on until stop commanded.
			TCC1->PER.reg = tcc1_per;//    f = 48M / 8 / 60000 =  100Hz                       
			while (TCC1->SYNCBUSY.bit.PER);                  // Wait for synchronization

			if (runmode == "live") { // fixed rate in live mode requires controlling the frequency with delay. // in playback mode, wired speed can be as high as 1KHz if needed.
				delay(11); // change the delay to vary the frequency < 130hz in live mode. e.g. 7ms: 118Hz. 8ms: 106Hz, 9ms: 96Hz  10ms: 88Hz,  11ms: 80Hz. 13ms: 70Hz,  15ms: 61Hz.  18ms: 50Hz. 
			}

			TCC1->CTRLA.bit.ENABLE = 1;                     // enable the TCC1 counter
			while (TCC1->SYNCBUSY.bit.ENABLE);              // Wait for synchronizatio
			
		
			if (wired_mode) {
				// the following works for thin pulse out.  
				//TCC0->CTRLBSET.reg = TCC_CTRLBSET_CMD_RETRIGGER;         // Retrigger the timer's One/Multi-Shot pulse
				//while (TCC0->SYNCBUSY.bit.CTRLB);                        // Wait for synchronization
				digitalWrite(PIN_LED, HIGH);
				spikeout = false;
			}
		}
		else {
			if (low_freq_during_idle) { //low frequency stimulation during idle for EXP3. Make a button?
				spikeout = true;
				if (runmode == "live") { 
					delay(48); // change the delay to vary the frequency < 130hz in live mode. e.g. 7ms: 118Hz. 8ms: 106Hz,9ms: 96Hz 10ms: 88Hz,  11ms: 80Hz. 13ms: 70Hz, 15ms: 61Hz.  19ms: 50Hz. 32ms:30Hz, 48ms: 20Hz.   96ms: 10.2z
				}

				TCC1->PER.reg = tcc1_idle_per;  //  f = 48M / 8 / 300000 = 20Hz                          
				while (TCC1->SYNCBUSY.bit.PER);                  // Wait for synchronization

				TCC1->CTRLA.bit.ENABLE = 1;                     // enable the TCC1 counter
				while (TCC1->SYNCBUSY.bit.ENABLE);              // Wait for synchronizatio
			}
			else {  // no spiking during idle (original) 
				TCC1->CTRLA.bit.ENABLE = 0;                     // Disable the TCC1 counter
				while (TCC1->SYNCBUSY.bit.ENABLE);              // Wait for synchronizatio

				if (wired_mode) {
					digitalWrite(PIN_LED, LOW);
				}
			}
			
		}


		//if (runmode == "live")
		//	delay(12); // delay(10): 95-100H. delay(12) ~ 75Hz. works in live mode) 
		//else // playback
		//	delay(1); // ok so this work to generate 75Hz but overall playaback slowed down too much. 
	}
	
#endif
	//second_gyro();

	//// ****** second gyro ******* 
	//I_total = I_Synapse2; // Add up all current sources

	//v2 = v2 + timestep_ms * (0.04 * v2 * v2 + 5 * v2 + 140 - u2 + I_total);
	//u2 = u2 + timestep_ms * (Array_a[NeuronBehaviour] * (Array_b[NeuronBehaviour] * v2 - u2));
	//if (v2 >= 30.0) {
	//	v2 = Array_c[NeuronBehaviour];
	//	u2 += Array_d[NeuronBehaviour];
	//	digitalWrite(PIN_NEURON2, HIGH);
	//	//SerialUSB.println("\tspike 2");
	//	spikeout = true;

	//	// the following works for thin pulse out.  
	//	TCC0->CTRLBSET.reg = TCC_CTRLBSET_CMD_RETRIGGER;         // Retrigger the timer's One/Multi-Shot pulse
	//	while (TCC0->SYNCBUSY.bit.CTRLB);                        // Wait for synchronization
	//}

	//else {
	//	digitalWrite(PIN_NEURON2, LOW);
	//}

	//if (v2 <= -90) { v2 = -90.0; } // prevent from analog out (below) going into overdrive - but also means that it will flatline at -90. Change the "90" in this line and the one below if want to




	//if (v > -30.0) {   // check if there has been a spike for digi out routine (below)

	//	// trigger audio click and Digi out 5V pulse if there has been a spike
	//
	//	digitalWrite(PIN_NEURON, HIGH);
	//	SerialUSB.println("\tspike");
	//	//if (nosound == 0) {
	//	//	
	//	//}
	//	spike = false;
	//}
	//else {
	//	digitalWrite(PIN_NEURON, LOW);
	//}


	/* Wait the specified delay before requesting next data  (sensor) */
	if (runmode == "playback") {
		delay(2); // control this for playing pre-recorded swing data. 
	}
	

	
	//else { // FIXED RATE.
	//	//SerialUSB.println("FIXED RATE RESUMED");
	//	PORT->Group[g_APinDescription[PIN_NEURON].ulPort].PMUX[g_APinDescription[PIN_NEURON].ulPin >> 1].bit.PMUXE = PORT_PMUX_PMUXE_F_Val; //PA12, TCC0-6


	//}
	//SerialUSB.print("delay ");
	//SerialUSB.println(micros() - time_us); //prints time since program started

}// end of loop()


//void TCC1_Handler()           // ISR TCC1 overflow callback function
//{
//	// Check for overflow (OVF) interrupt
//	if (TCC1->INTFLAG.bit.OVF)// && TCC1->INTENSET.bit.OVF)
//	{
//		TCC1->INTFLAG.bit.OVF = 1;         // Clear the OVF interrupt flag
//	}
//	// Check for match counter 0 (MC0) interrupt
//	if (TCC1->INTFLAG.bit.MC1 && TCC1->INTENSET.bit.MC1)
//	{
//		//    if (spikeout){
//		//      spikeout = false;
//		//    }
//		spikeout = true; // only on for one cycle
//
//		REG_TCC1_INTFLAG = TCC_INTFLAG_MC1;         // Clear the MC0 interrupt flag
//		TCC1->INTFLAG.bit.MC1 = 1;   // Clear the MC0 interrupt flag
//	}
//}
//
//void setPulseFrequency(commands) {
//	
//
//}
//


void rf95_setup() {


	pinMode(RFM95_RST, OUTPUT);
	digitalWrite(RFM95_RST, HIGH);


	delay(100);

	SerialUSB.println("Feather LoRa TX Test!");

	// manual reset
	digitalWrite(RFM95_RST, LOW);
	delay(10);
	digitalWrite(RFM95_RST, HIGH);
	delay(10);

	while (!rf95.init()) {
		SerialUSB.println("LoRa radio init failed");
		SerialUSB.println("Uncomment '#define SerialUSB_DEBUG' in RH_RF95.cpp for detailed debug info");
		while (1);
	}
	SerialUSB.println("LoRa radio init OK!");

	// Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
	if (!rf95.setFrequency(RF95_FREQ)) {
		SerialUSB.println("setFrequency failed");
		while (1);
	}
	SerialUSB.print("Set Freq to: "); SerialUSB.println(RF95_FREQ);

	// FSK/OOK mode available??

	  //** setting for the fastest Lora *****////
	  //* Set SpreadingFactor = 6 in RegModemConfig2 
	  //* The header must be set to Implicit mode 
	  //  (With SF = 6 selected, implicit header mode is the only mode of operation possible.)
	  //* Write bits 2-0 of register address 0x31 to value "0b101" 
	  //* Write register address 0x37 to value 0x0C



	  //Signal Bandwidth

	  //Continuous Reception Operating Mode

	  //rf95.setPayloadCRC(0);// turn off CRC?
	//  rf95.setPreambleLength(6); // Default is 8,  setting to 6 doesn't not seem to increase rate
	//  //TODO:  how to set header to be implicit mode ((0x1D), bit 0=>1)
	//  rf95.spiWrite(0x31, 0b101); /// following recommenation preamble length = 6  (not for Lora?)
	//  rf95.spiWrite(0x37, 0x0C);/// following recommenation preamble length = 6  (not for Lora?_

	delay(10);


	// Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

	// The default transmitter power is 13dBm, using PA_BOOST.
	// If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
	// you can set transmitter powers from 5 to 23 dBm:
	rf95.setTxPower(20, false);

	// setting method 1 (Sets all the registers required to configur, e.g. BW, SF) 
	// option: https://github.com/Yveaux/RadioHead/blob/master/RadioHead/RH_RF95.cpp#L502
  //  const RH_RF95::ModemConfig myProfile =  { 
  //    RH_RF95_BW_500KHZ | RH_RF95_CODING_RATE_4_5,
  //    RH_RF95_SPREADING_FACTOR_128CPS }; //RH_RF95_SPREADING_FACTOR_64CPS the fastest but receiver can't handle?
  //  rf95.setModemRegisters(&myProfile);
  //  //RH_RF95_IMPLICIT_HEADER_MODE_ON
  //  delay(10);


  ////** set FSK mode (doens;t work)
  //// Set sleep mode, so we can also set LORA mode:
  //    rf95.spiWrite(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_SLEEP | RH_RF95_MODE_FSTX);
  //    delay(10); // Wait for sleep mode to take over from say, CAD
  //    // Check we are in sleep mode, with LORA set
  //    if (rf95.spiRead(RH_RF95_REG_01_OP_MODE) != (RH_RF95_MODE_SLEEP | RH_RF95_MODE_FSTX))
  //    {
  //      SerialUSB.println(rf95.spiRead(RH_RF95_REG_01_OP_MODE), HEX);
  //    }




  //// Set up FIFO // not faster if on
  //    // We configure so that we can use the entire 256 byte FIFO for either receive
  //    // or transmit, but not both at the same time
  //  rf95.spiWrite(RH_RF95_REG_0E_FIFO_TX_BASE_ADDR, 0);
  //  rf95.spiWrite(RH_RF95_REG_0F_FIFO_RX_BASE_ADDR, 0);
  //  delay(10);

	// setting method 2 (set by pre-defined cnonfigs: easier than 1 but only 4 choices?)
	//rf95.setModemConfig(RH_RF95::Bw500Cr45Sf128); ///
		//Bw125Cr45Sf128 = 0,    ///< Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on. Default medium range
		//Bw500Cr45Sf128,            ///< Bw = 500 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on. Fast+short range
		//Bw31_25Cr48Sf512,    ///< Bw = 31.25 kHz, Cr = 4/8, Sf = 512chips/symbol, CRC on. Slow+long range
	  //  Bw125Cr48Sf4096,           ///< Bw = 125 kHz, Cr = 4/8, Sf = 4096chips/symbol, CRC on. Slow+long range


	//https://hackaday.io/project/27791/instructions
	//Example: RH_RF95::ModemConfig myconfig = { 0x28,   0x94,    0x04 }; // Bw15_6Cr48Sf512 
	// will give you
	// 15.6kHz bandwidth, 4/8 coding rate, spreading factor  9 (512), low data rate optimization off, AGC on

	//{ 0x72, 0x74, 0x00}, // Bw125Cr45Sf128 (the chip default)
	//{ 0x92,   0x74,    0x00 }, // Bw500Cr45Sf128
	//{ 0x48,   0x94,    0x00 }, // Bw31_25Cr48Sf512
	//{ 0x78,   0xc4,    0x00 }, // Bw125Cr48Sf4096

	//FASTER: minimize coding rate (CR), minimize the spreading rate (SF), Maximize bandwidth. 
	// RFM97: spreading factor 6-12. 
	// FSK/OOK modem vs LoRa modem. -> FSK/GFSK/OOK modes are not (yet) supported.

	//Spreading Factor 6
	//Special requirement for SF6 -> 0x61
	//The header must be set to Implicit mode 
	// Write bits 2-0 of register address 0x31 to value "0b101" 
	//Write register address 0x37 to value 0x0C
	// Doesn't work with implicit mode. 

	RH_RF95::ModemConfig myconfig = { 0x92,   0x70,    0x00 };
	// CR:2->e, doesn't make it faster. 2->1 doesn't received (implicit header mode): 92 is final
	// SF:7->6 doesn't receive. 7->8 (pulse rate halves).  -: 70 is final. 
	// 00: no CRC, final.

	rf95.setModemRegisters(&myconfig);
	//rf95.spiWrite(0x31, 0b101); /// following recommenation preamble length = 6  (why doesn't this work)- - (
	//rf95.spiWrite(0x37, 0x0C);/// following recommenation preamble length = 6  (why doesn't this work)
	////It's been a while, but if I remember correctly the device I tested with would only use implicit header mode with low data rate enabled. 

	rf95.setPreambleLength(6); // between 6 and 65536.

	//rf95.printRegisters();
	  /// to the Serial device if RH_HAVE_SERIAL is defined for the current platform


}

// One shot pulse (spike kernal)
void interrupt_pulsegen_pulsein()
{
	if (TCC0->STATUS.bit.STOP)                                 // Check if the previous pulse is complete
	{
		//one shot is irrelevant? 
		
		// count restarts...
		//TCC0->CTRLBSET.reg = TCC_CTRLBSET_CMD_RETRIGGER;         // Retrigger the timer's One/Multi-Shot pulse
		//while (TCC0->SYNCBUSY.bit.CTRLB);                        // Wait for synchronization
	}

}


void oneshot_TCC0_thinPulse_setup() { // ONE_SHOT 
	// Neuron ENCODING

	GCLK->GENDIV.reg = GCLK_GENDIV_DIV(1) |          // Divide the 48MHz clock source by divisor 10: 48MHz/1=48MHz
		GCLK_GENDIV_ID(4);            // Select Generic Clock (GCLK) 4
	while (GCLK->STATUS.bit.SYNCBUSY);               // Wait for synchronization

	GCLK->GENCTRL.reg = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOWs
		GCLK_GENCTRL_GENEN |         // Enable GCLK4
		GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
		GCLK_GENCTRL_ID(4);          // Select GCLK4
	while (GCLK->STATUS.bit.SYNCBUSY);               // Wait for synchronization

	//*** Using direct port and pin numbers *******
	// TO test muxing in samd21. (may discard, works.)

	PORT->Group[g_APinDescription[PIN_THIN_PULSEOUT].ulPort].PINCFG[g_APinDescription[PIN_THIN_PULSEOUT].ulPin].bit.PMUXEN = 1;

	// Connect the TCC timers to the port outputs - port pins are paired odd PMUO and even PMUXE
	// F & E specify the timers: TCC0, TCC1 and TCC2
	PORT->Group[g_APinDescription[PIN_THIN_PULSEOUT].ulPort].PMUX[g_APinDescription[PIN_THIN_PULSEOUT].ulPin >> 1].bit.PMUXE = PORT_PMUX_PMUXE_F_Val; //PA16, TCC0-WO6 (CC[2])


// Feed GCLK4 to TCC0 and TCC1
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TCC0 and TCC1
		GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
		GCLK_CLKCTRL_ID_TCC0_TCC1;   // Feed GCLK4 to TCC0 and TCC1
	while (GCLK->STATUS.bit.SYNCBUSY);                // Wait for synchronization  

  // TCC0 setting 
	TCC0->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;          // Single slope PWM operation     
	while (TCC0->SYNCBUSY.bit.WAVE);                 // Wait for synchronization

	TCC0->PER.reg = 10000;//                          z
	while (TCC0->SYNCBUSY.bit.PER);                  // Wait for synchronization
	TCC0->CC[2].reg = 5000;
	while (TCC0->SYNCBUSY.bit.CC2);

	//REG_TCC0_INTFLAG |= TCC_INTFLAG_OVF | TCC_INTFLAG_MC0 | TCC_INTFLAG_MC1;              // Clear the overflow interrupt flag
	//REG_TCC0_INTENSET = TCC_INTENSET_OVF | TCC_INTENSET_MC0 | TCC_INTENSET_MC1;            // Enable TCC1 overflow interrupt
	//while (TCC0->SYNCBUSY.bit.WAVE);                // Wait for synchronization

	TCC0->CTRLBSET.reg |= TCC_CTRLBSET_ONESHOT;// | TCC_CTRLBSET_DIR;  // Is this right way to do down-counting and one shot mode together? I concur. 
	//last_cmd = TCC_CTRLBSET_DIR;
	while (TCC0->SYNCBUSY.bit.CTRLB);                 // Wait for synchronization

	TCC0->DRVCTRL.reg |= TCC_DRVCTRL_NRE6;



	TCC0->CTRLA.reg |= TCC_CTRLA_PRESCALER_DIV8 |    // Divide GCLhaveK4 by 1 (was 8)
		TCC_CTRLA_PRESCSYNC_RESYNC |   // set to retrigger timer on GCLK with prescaler reset
		TCC_CTRLA_ENABLE;             // Enable the TCC0 output
	while (TCC0->SYNCBUSY.bit.ENABLE);               // Wait for synchronization

	TCC0->CTRLA.bit.ENABLE = 1;                     // Disable the TCC1 counter
	while (TCC0->SYNCBUSY.bit.ENABLE);              // Wait for synchronization



	SerialUSB.println("TCC0 for oneshot setting completed.");
}


void fixed_rate_pulse_out_setup() {
	// FIXED RATE ENCODING

	//GCLK->GENDIV.reg = GCLK_GENDIV_DIV(1) |          // Divide the 48MHz clock source by divisor 10: 48MHz/1=48MHz
	//	GCLK_GENDIV_ID(4);            // Select Generic Clock (GCLK) 4
	//while (GCLK->STATUS.bit.SYNCBUSY);               // Wait for synchronization

	//GCLK->GENCTRL.reg = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOWs
	//	GCLK_GENCTRL_GENEN |         // Enable GCLK4
	//	GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
	//	GCLK_GENCTRL_ID(4);          // Select GCLK4
	//while (GCLK->STATUS.bit.SYNCBUSY);               // Wait for synchronization

	//*** Using direct port and pin numbers *******
	// TO test muxing in samd21. (may discard, works.)

	PORT->Group[g_APinDescription[PIN_PA07].ulPort].PINCFG[g_APinDescription[PIN_PA07].ulPin].bit.PMUXEN = 1;

	// Connect the TCC timers to the port outputs - port pins are paired odd PMUO and even PMUXE
	// F & E specify the timers: TCC0, TCC1 and TCC2
	PORT->Group[g_APinDescription[PIN_PA07].ulPort].PMUX[g_APinDescription[PIN_PA07].ulPin >> 1].bit.PMUXO = PORT_PMUX_PMUXO_E_Val; //PA07, TCC1/WO[1]


//// Feed GCLK4 to TCC0 and TCC1
//	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TCC0 and TCC1
//		GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
//		GCLK_CLKCTRL_ID_TCC0_TCC1;   // Feed GCLK4 to TCC0 and TCC1
//	while (GCLK->STATUS.bit.SYNCBUSY);                // Wait for synchronization  

  // TCC1 setting 
	TCC1->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;          // Single slope PWM operation     
	while (TCC1->SYNCBUSY.bit.WAVE);                 // Wait for synchronization

	TCC1->PER.reg = tcc1_per;//                          z
	while (TCC1->SYNCBUSY.bit.PER);                  // Wait for synchronization
	TCC1->CC[1].reg =tcc1_pulsewidth ;
	while (TCC1->SYNCBUSY.bit.CC1);

	//REG_TCC1_INTFLAG |= TCC_INTFLAG_OVF | TCC_INTFLAG_MC0 | TCC_INTFLAG_MC1;              // Clear the overflow interrupt flag
	//REG_TCC1_INTENSET = TCC_INTENSET_OVF | TCC_INTENSET_MC0 | TCC_INTENSET_MC1;            // Enable TCC1 overflow interrupt
	//while (TCC1->SYNCBUSY.bit.WAVE);                // Wait for synchronization

	TCC1->CTRLA.reg |= TCC_CTRLA_PRESCALER_DIV8 |    // Divide GCLhaveK4 by 1 (was 8)
		TCC_CTRLA_PRESCSYNC_RESYNC |   // set to retrigger timer on GCLK with prescaler reset
		TCC_CTRLA_ENABLE;             // Enable the TCC0 output
	while (TCC1->SYNCBUSY.bit.ENABLE);               // Wait for synchronization

	TCC1->CTRLA.bit.ENABLE = 0;                     // Disable the TCC1 counter
	while (TCC1->SYNCBUSY.bit.ENABLE);              // Wait for synchronization

	// added 20200709 because otherwise it never enters TCC1_Hander
	NVIC_SetPriority(TCC1_IRQn, 0);    // Set the Nested Vector Interrupt Controller (NVIC) priority for TCC to 0 (highest)
	NVIC_EnableIRQ(TCC1_IRQn);         // Connect TCC to Nested Vector Interrupt Controller (NVIC)

	SerialUSB.println("TCC1 for pulseout setting completed.");
}



#if IS_BNO055

/**************************************************************************/
/*
	Display some basic info about the sensor status
*/
/**************************************************************************/
void displaySensorStatus(void)
{
	/* Get the system status values (mostly for debugging purposes) */
	uint8_t system_status, self_test_results, system_error;
	system_status = self_test_results = system_error = 0;
	bno.getSystemStatus(&system_status, &self_test_results, &system_error);

	/* Display the results in the Serial Monitor */
	SerialUSB.println("");
	SerialUSB.print("System Status: 0x");
	SerialUSB.println(system_status, HEX);
	SerialUSB.print("Self Test:     0x");
	SerialUSB.println(self_test_results, HEX);
	SerialUSB.print("System Error:  0x");
	SerialUSB.println(system_error, HEX);
	SerialUSB.println("");
	delay(500);
}



/*
	Displays some basic information on this sensor from the unified
	sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
	sensor_t sensor;
	bno.getSensor(&sensor);
	SerialUSB.println("------------------------------------");
	SerialUSB.print("Sensor:       "); SerialUSB.println(sensor.name);
	SerialUSB.print("Driver Ver:   "); SerialUSB.println(sensor.version);
	SerialUSB.print("Unique ID:    "); SerialUSB.println(sensor.sensor_id);
	SerialUSB.print("Max Value:    "); SerialUSB.print(sensor.max_value); SerialUSB.println(" xxx");
	SerialUSB.print("Min Value:    "); SerialUSB.print(sensor.min_value); SerialUSB.println(" xxx");
	SerialUSB.print("Resolution:   "); SerialUSB.print(sensor.resolution); SerialUSB.println(" xxx");
	SerialUSB.println("------------------------------------");
	SerialUSB.println("");
	delay(500);
}



/**************************************************************************/
/*
	Display sensor calibration status
*/
/**************************************************************************/
void displayCalStatus(void)
{
	/* Get the four calibration values (0..3) */
	/* Any sensor data reporting 0 should be ignored, */
	/* 3 means 'fully calibrated" */
	uint8_t system, gyro, accel, mag;
	system = gyro = accel = mag = 0;
	bno.getCalibration(&system, &gyro, &accel, &mag);

	/* The data should be ignored until the system calibration is > 0 */
	SerialUSB.print("\t");
	if (!system)
	{
		SerialUSB.print("! ");
	}

	/* Display the individual values */
	SerialUSB.print("Sys:");
	SerialUSB.print(system, DEC);
	SerialUSB.print(" G:");
	SerialUSB.print(gyro, DEC);
	SerialUSB.print(" A:");
	SerialUSB.print(accel, DEC);
	SerialUSB.print(" M:");
	SerialUSB.print(mag, DEC);
}


#endif



////////////////////////////////////////////////////////////////////////////////////////////////







/// Put the things that is depracated
//
//// UI control 
////int ledPin = 13;
//String inputString = "";         // a string to hold incoming data
//String value = "";
//boolean stringComplete = false;  // whether the string is complete
//String commandString = "";
//boolean isConnected = false;
//// end of UI control

//
//void getCommand() {
//	if (inputString.length() > 0) {
//		commandString = inputString.substring(1, 5);
//		//value = inputString.substring(6,inputString.length()-2);
//		value = inputString.substring(5, inputString.length() - 1);
//
//	}
//}
//
//void serialEvent() {
//	while (SerialUSB.available() > 0) {
//		// get the new byte:
//		char inChar = (char)SerialUSB.read();
//		// add it to the inputString:
//		inputString += inChar;
//		// if the incoming character is a newline, set a flag
//		// so the main loop can do something about it:
//		if (inChar == '\n') {
//			stringComplete = true;
//		}
//	}
//}


// in the loop ()
// temporary horrible design
//serialEvent();
//if (stringComplete) {
//	stringComplete = false;
//	getCommand();
//
//	if (commandString.equals("CONN")) {
//	}
//	else if (commandString.equals("STOP")) {
//	}
//	else if (commandString.equals("TYPE")) {
//		if (value.equals("IZ Neuron")) {
//			IZHIKEVICH_NEURON_ENCODING = true;
//		}
//		else if (value.equals("Fixed Rate")) {
//			IZHIKEVICH_NEURON_ENCODING = false;
//		}
//		else {
//			IZHIKEVICH_NEURON_ENCODING = false;
//		}
//	}
//	else if (commandString.equals("VALC")) {
//		// PER.reg max = 0xFFFF (65535) 
//		int period = 750000 / value.toInt();//3750<- 200Hz , 48M(base)/64(div) = 750000  
//		TCC0->PER.reg = period;                  // Set the frequency of the PWM on TCC0 to 10kHz
//		while (TCC0->SYNCBUSY.bit.PER);                  // Wait for synchronization
//
//	}
//	inputString = "";
//}
//// end of temporary horrible design

