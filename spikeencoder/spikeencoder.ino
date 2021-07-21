/*
 Name:		 spikeencoder.ino
 Created:	 12/16/2019 11:55:40 PM
 big change: 4/23/2021.  integrate spikeencoder and RFM_relay into a single solution. IZN neuron needs to run in the previously RFM_relay 
		     to overcome the wireless pulse rate cap (130Hz) wiht the RFM.  We send the leg kinematic instead thru RFM and generate spikes

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

// *******   PINS   ********
// PIN_PULSEOUT_TO_CWU  10  // PA18 (11, PA16 is damaged in arduino) 
// 



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



#define I2C_SPIKEENCODER_ADDR	20 // slave address seen by core 1. Make sure the address matches

#define PIN_FR			9  // PA07, for fixed rate pulse output. (TODO: make sure PW are the same)
//#define PIN_THIN_PULSEOUT	11 // PA16   // IZN output  (pin broken in RFM_relay)
#define PIN_FR_IN			6   //PA20    (attachIntterupt for PIN_PA07)

#define PIN_LED				13 // PA17 
#define PIN_PULSEOUT_TO_CWU  10// PA18 (11, PA16 is damaged in arduino) 

#define PIN_INTERRUPT_C1	5 // PA15   Interrupt pin to call I2C request from the master (C1)  
#define PIN_DAC				14 // PA02 (A0)



//#define NO_RF_Wireless  true  //  False for live gyro / fixed rate.   True during swing data recording where you need faster rate & playback.


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


//** PARAMS  **//
// gyro_scale_orig is the same as in the gryo_sender.  The purpose is to scale it y-range to 0-255 (from the gyro raw data which is <10)
int gyro_scale_orig = 30;//was 40 (SP1) but change to match gyro_sender (that doesn't clip) 30;//30 ; // gyro values typically < 1000. , if 150,too easily saturate to max (~100Hz)
int gyro_scale_high_spike_train = 40;  // This parameter to map the gyro to higher spike rate domain ( >300Hz for high values) 
int gyro_live_factor = 2;    //1: usually upto ~160Hz,  max ~220 Hz. 2: usually upto ~300Hz, max ~400HZ   3: usually up to ~400Hz, max ~600Hz determines the frequency range during live. 

int repeated_data = 0;  // detect a repeated gyrodata that is too long to put put it to zero in case of RFM connection lost.


//TCC 
volatile boolean spikeout = false;
volatile uint32_t tcc1_per = 30000;//60000; // fixed rate pulse out period (DIV8: 60000 is 100Hz, 30000 is 200 Hz). TCC limit: 24 bits. 
volatile uint32_t tcc1_idle_per = 170000;   //  EXP3.  35.30Hz   
volatile uint16_t tcc1_pulsewidth = 1500;  // TCC1 pulseout thinkness = 200us now.
volatile uint32_t tcc1_50hz_per = 120000; // 50Hz mode for exp3

// exp3
//bool low_freq_during_idle = true; // default: idle state (stationary state) doesn't generate feedback. 
String exp3_mode = "MODE1_idle_silent";

#endif 

//serial button options 
String runmode = "live"; // live or playback.
String encoding_mode = "IZNeuron";  // IZNeuron or fixed rate.
String cadence_mode = "None"; // e.g. cad50, cad50_kick1st.

uint8_t gyrodata = 0; // wireless gyro data 8 bit
uint8_t gyro_source = 0;  // 0: from left leg, 1: from right leg.
uint8_t gyrodata_prev = 0;
uint8_t gyro_source_prev = 0;

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

	//***  I2C slave connected to CORE1 
	Wire.begin(I2C_SPIKEENCODER_ADDR); // set itself as slave with address 
	Wire.onRequest(requestEvent);	//register event

	pinMode(PIN_INTERRUPT_C1, OUTPUT);
	digitalWrite(PIN_INTERRUPT_C1, LOW);


	SerialUSB.begin(9600);

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


	// to share the one output pin to CWU between IZN and FR mode. 
	pinMode(PIN_FR_IN, INPUT_PULLUP); //  will be uses as EIC. PA07 -> PA20
	attachInterrupt(PIN_FR_IN, interrupt_pulsegen_FR, CHANGE);


	// DAC setting (A0 pin)
	analogWriteResolution(10); //10 is max. 
	//analogWrite(A0, setDAC(I_Synapse1) );
	//analogWrite(A0,  0 ); // 0 to 1023?



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

	//// setting for IZ NEURON AND IS_GYRO
	//// DISABLE FIXED RATE OUTPUT  (control mux to disable PWM and enable digitalWrite
	//PORT->Group[g_APinDescription[PIN_NEURON1].ulPort].PMUX[g_APinDescription[PIN_NEURON1].ulPin >> 1].bit.PMUXE = PORT_PMUX_PMUXE_B_Val; //PA18
	//PORT->Group[g_APinDescription[PIN_NEURON2].ulPort].PMUX[g_APinDescription[PIN_NEURON2].ulPin >> 1].bit.PMUXO = PORT_PMUX_PMUXO_B_Val; //PA15

	// IZ NEURON ENCODING
	

	pinMode(PIN_LED, OUTPUT);

	pinMode(PIN_FR, OUTPUT);


	oneshot_TCC0_thinPulse_setup();  // 
	fixed_rate_pulse_out_setup();

}


int16_t packetnum = 0;  // packet counter, we increment per xmission

//
//
//void rf_send() {
//	//SerialUSB.println("Sending to rf95_server");
// // Send a message to rf95_server
//
//
// // ** rf95.send() only when there is a spike.  **//
//	if (spikeout) {
//		digitalWrite(PIN_LED, HIGH);
//		spikeout = false;
//		//noInterrupts();
//		char radiopacket = '1';
//		//itoa(packetnum++, radiopacket, 10);
//		SerialUSB.print("Sending..... "); SerialUSB.println(radiopacket);
//		//radiopacket[0] = 0;
//
//
//		rf95.send((uint8_t*)radiopacket, 1); 
//		digitalWrite(PIN_LED, LOW);
//		//interrupts();
//	}
//	// working version 20210320 - sends a number incremented
//	//if (spikeout) {
//	//	digitalWrite(PIN_LED, HIGH);
//	//	spikeout = false;
//	//	//noInterrupts();
//	//	char radiopacket[4] = " ";
//	//	itoa(packetnum++, radiopacket, 10);
//	//	SerialUSB.print("Sending... "); SerialUSB.println(radiopacket);
//	//	radiopacket[2] = 0;
//
//
//	//	rf95.send((uint8_t*)radiopacket, 3);
//	//	digitalWrite(PIN_LED, LOW);
//	//	//interrupts();
//	//}
//
//	//  SerialUSB.println("Waiting for packet to complete..."); delay(10);
//	//  rf95.waitPacketSent();
//	//  // Now wait for a reply
//	//  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
//	//  uint8_t len = sizeof(buf);
//	//
//	//  SerialUSB.println("Waiting for reply..."); delay(10);
//	//  if (rf95.waitAvailableTimeout(1000))
//	//  { 
//	//    // Should be a reply message for us now   
//	//    if (rf95.recv(buf, &len))
//	//   {
//	//      SerialUSB.print("Got reply: ");
//	//      SerialUSB.println((char*)buf);
//	//      SerialUSB.print("RSSI: ");
//	//      SerialUSB.println(rf95.lastRssi(), DEC);    
//	//    }
//	//    else
//	//    {
//	//      SerialUSB.println("Receive failed");
//	//    }
//	//  }
//	//  else
//	//  {
//	//    SerialUSB.println("No reply, is there a listener around?");
//	//  }
//	//  delay(1000);
//
//}

int setDAC(float I) { 
	return  (int)(I /600*1023);  // assume I is 0 to 600 (in case of playback),   
}

void rf_receive() {



	if (rf95.available())
	{
		//SerialUSB.println("rf95 available");
		// Should be a message for us now
		uint8_t buf[3];
		uint8_t len = sizeof(buf);


		if (rf95.recv(buf, &len))
		{

			//RH_RF95::printBuffer("Received: ", buf, len);// this one has a problem.
			packetnum++;

			//SerialUSB.print("Got: ");
			//      SerialUSB.print(packetnum);
				 // SerialUSB.print(" ");
			//SerialUSB.println((char*)buf);
			//SerialUSB.print("RSSI: ");
			//RSSI: receiver signal strength indicator. This number will range from about -15 to about -100. The larger the number (-15 being the highest you'll likely see) the stronger the signal.
			//SerialUSB.println(rf95.lastRssi(), DEC);

			////* trigger one-shot mode pulse * // 20210312 
		

			gyro_source = buf[0];
			gyrodata = buf[1];
			//SerialUSB.print("Gyro source: ");
			SerialUSB.print("gyro_source at rf_received: ");
			SerialUSB.println(gyro_source);

			// I2C send to Core 1 the gyro_source. 
			if (gyro_source != 0 & gyro_source != gyro_source_prev) {  // send to bit interrupt to core1 only when source changed 
				digitalWrite(PIN_INTERRUPT_C1, HIGH);
				delayMicroseconds(200);
				//delayMicroseconds(20000);
				digitalWrite(PIN_INTERRUPT_C1, LOW);
				SerialUSB.println("Interruption bit sent to Core 1");
			}



			if (gyrodata > 15) {
				digitalWrite(PIN_LED, HIGH);
			}
			else {
				digitalWrite(PIN_LED, LOW);
			}

			//digitalWrite(PIN_PULSEOUT_TO_CWU, HIGH);
			//delay(1);

			// Send a reply
	  //      uint8_t data[] = "And hello back to you";
	  //      rf95.send(data, sizeof(data));
	  //      rf95.waitPacketSent();
	  //      SerialUSB.println("Sent a reply");
		}
		else
		{

			SerialUSB.println("Receive failed");

		}
	}

	// condition for RF signal lost (battery out or other reason)
	if (gyrodata > 10) {
		if (gyrodata == gyrodata_prev) {
			repeated_data += 1;
			//SerialUSB.print("SAME DATA: ");
			//SerialUSB.println(repeated_data);
			if (repeated_data == 100) { // set current input to neuron to zero.
				gyrodata = 0;
				repeated_data = 0;
				digitalWrite(PIN_LED, LOW);
			}
		}
		else {
			repeated_data = 0;
		}
	}
	gyrodata_prev = gyrodata;
	gyro_source_prev = gyro_source;
}

unsigned int time_us;
unsigned int playback_starttime;
sensors_event_t event;  // L3GD30
int gn = 0; // counter (gyro n)
int gyro_scale;


void loop() {
	time_us = micros();
	gyro_scale = gyro_scale_orig;
	


	


	//gyro_scale *= gyro_scale_high_spike_train; // map to highter pulse frequency. 

	// check system time in microseconds
	//unsigned long currentMicros = micros() - startMicros;

	// serial connection 
	char command;
	char commands[4];

	int ua = SerialUSB.available();

	if (ua > 0)
	{
		//std::vector <char> buffer(ua); 

		//if the first byte = A: read 1 byte (command), B: read 2 bytes (uint16_t)
		// AA, AB, AX... (same except the first char) 
		// BCC, B5V, ....  (send the upper byte first)  
		
		//e.g.  uint16_t number = 5703;               // 0001 0110 0100 0111
		//		uint16_t mask = B11111111;          // 0000 0000 1111 1111
		//		uint8_t first_half = number >> 8;   // >>>> >>>> 0001 0110
		//		uint8_t sencond_half = number & mask; // ____ ____ 0100 0111
		
		command = SerialUSB.read(); // read the first byte. 


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

		// encoding mode setting 
		if (command == 'A') {
			// IZ Neuron encoding 
			encoding_mode = "IZNeuron";

			// turn off the FR mode if it was on.
			TCC1->CTRLA.bit.ENABLE = 0;                     // Disable the TCC1 counter
			while (TCC1->SYNCBUSY.bit.ENABLE);              // Wait for synchronization

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
			cadence_mode = "stop";
			SerialUSB.println(F("Run mode: playback"));
		}
		else {
			SerialUSB.println(F("Unknown runmode"));
		}


		// idle definitione
		if (command == 'T') {
			exp3_mode = "MODE1_idle_silent";// low_freq_during_idle = false;
			SerialUSB.println(F("MODE1_Idle: silent"));
		}
		else if (command == 'U') {
			// fixed rate encoding
			exp3_mode = "MODE2_idle_lowfreq_stim";  //low_freq_during_idle = true;
			SerialUSB.println(F("MODE2_Idle: low freq"));
		}
		else if (command == 'V') {
			/*encoding_mode = "fixedrate";  */
			exp3_mode = "MODE3_idle_run_both_50hz";
			SerialUSB.println(F("MODE3_Idle = run = 50Hz"));
		}
		else if (command == 'W') {
			/*encoding_mode = "fixedrate";  */
			exp3_mode = "MODE4_idle_200_run_300Hz";
			SerialUSB.println(F("MODE3_Idle = run = 50Hz"));
		}




		if (runmode == "playback") {
			switch (command) {
			case 'C':
				cadence_mode = "cad50";
				n = 0;
				playback_starttime = micros();
				break;

			case 'D':    // swing2
				cadence_mode = "cad60";
				n = 0;
				playback_starttime = micros();
				break;

			case 'E':    // swing3
				cadence_mode = "cad90";
				n = 0;
				playback_starttime = micros();
				break;

			case 'F':    // swing4
				cadence_mode = "cad50_k1"; // cad50_kick1st 
				n = 0;
				playback_starttime = micros();
				break;

			case 'G':   // swing5
				cadence_mode = "cad60_k1"; //// cad60_kick1st
				n = 0;
				playback_starttime = micros();
				break;
			case 'H':   // swing6
				cadence_mode = "cad50_k2"; // cad50_kick2nd 
				n = 0;
				playback_starttime = micros();
				break;
			case 'I':   // swing7
				cadence_mode = "cad60_k2"; // cad60_kick2nd 
				n = 0;
				playback_starttime = micros();
				break;
			case 'J':   // swing8
				cadence_mode = "cad50_tr"; // cad50_trip 
				n = 0;
				playback_starttime = micros();
				break;
			case 'K':   // swing9
				cadence_mode = "cad60_tr"; // cad60_Trip 
				n = 0;
				playback_starttime = micros();
				break;
			default:
				SerialUSB.println(F("Cadence mode not defined")); //
				break;
			}
		}
	}
	//SerialUSB.println(size_of_array); //


	if (runmode == "live") { // live or playback mode. 
		rf_receive(); // gyrodata gets updated...
		//SerialUSB.print("Source: ");
		//SerialUSB.print(gyro_source);
		//SerialUSB.print(" reception test: ");
		//SerialUSB.println(gyrodata, DEC);


		I_Synapse1 = (float) gyrodata  * gyro_live_factor; //  this gyrodata is uint8_t

		//SerialUSB.println(I_Synapse1);
		//SerialUSB.print(",");
	} // end of realtime mode
	else if (runmode == "playback") {
		int upsample_factor = 4; // since 1khz gyro is downsampled to 130Hz due to RFM,  upsample back.

		//gyro_scale *= 2; // increase the rate in the playback mode when the bottle neck from RFM is removed.
		//				// instead of max 130Hz if live mode, this can go up to 300 Hz (or more). 
		if (cadence_mode == "stop") { // do nothing 
			n = 0;


		}
		else if (cadence_mode == "cad50") { // swing 1
			// load one value per loop.
			int m = floor(n / upsample_factor);
			sensor_playback = bellshape_swing_level1[m];
			sensor_playback = sensor_playback * 0.6; // 1 * 0.8 = 0.8
			if (n == sizeof(bellshape_swing_level1)* upsample_factor / sizeof(int) - 1) {
				cadence_mode = "stop";
				n = 0;
				SerialUSB.print("Elapsed time: ");
				SerialUSB.println(micros() - playback_starttime);
			}
			n = n + 1;
		}
		else if (cadence_mode == "cad60") { // swing 2
		   // load one value per loop.
			int m = floor(n / upsample_factor);
			sensor_playback = bellshape_swing_level2[m];
			sensor_playback = sensor_playback * 0.5;  // 2 * 0.65 = 1.3

			//I_Synapse1 = sensor_playback * gyro_scale;
			if (n == sizeof(bellshape_swing_level2)* upsample_factor / sizeof(int) - 1) {
				cadence_mode = "stop";
				n = 0;
				SerialUSB.print("Elapsed time: ");
				SerialUSB.println(micros() - playback_starttime);
			}
			n = n + 1;
		}
		else if (cadence_mode == "cad90") { // swing 3
	// load one value per loop.
			int m = floor(n / upsample_factor);
			sensor_playback = bellshape_swing_level3[m];
			sensor_playback = sensor_playback * 1.5;   //3*1.8 = 5.4

			//I_Synapse1 = sensor_playback * gyro_scale;
			if (n == sizeof(bellshape_swing_level3)* upsample_factor / sizeof(int) - 1) {
				cadence_mode = "stop";
				n = 0;
				SerialUSB.print("Elapsed time: ");
				SerialUSB.println(micros() - playback_starttime);
			}
			n = n + 1;
		}
		else if (cadence_mode == "cad50_k1") { // swing 4
			// load one value per loop.
			int m = floor(n / upsample_factor);
			sensor_playback = cad50_kick1st_1swing_max3p7[m];
			//I_Synapse1 = sensor_playback * gyro_scale;
			if (n == sizeof(cad50_kick1st_1swing_max3p7)* upsample_factor / sizeof(int) - 1) {
				cadence_mode = "stop";
				n = 0;
				SerialUSB.print("Elapsed time: ");
				SerialUSB.println(micros() - playback_starttime);
			}
			n = n + 1;
		}
		else if (cadence_mode == "cad60_k1") { // swing 5
			// load one value per loop.
			int m = floor(n / upsample_factor);

			sensor_playback = cad60_kick1st_1swing_max4p6[m];
			//I_Synapse1 = sensor_playback * gyro_scale;
			if (n == sizeof(cad60_kick1st_1swing_max4p6)* upsample_factor / sizeof(int) - 1) {
				cadence_mode = "stop";
				n = 0;
				SerialUSB.print("Elapsed time: ");
				SerialUSB.println(micros() - playback_starttime);
			}
			n = n + 1;
		}
		else if (cadence_mode == "cad50_k2") { // swing 6
			// load one value per loop.
			int m = floor(n / upsample_factor);

			sensor_playback = cad50_kick2nd_1swing_max6p1[m];
			sensor_playback = sensor_playback * 1.4;

			//I_Synapse1 = sensor_playback * gyro_scale;
			if (n == sizeof(cad50_kick2nd_1swing_max6p1)* upsample_factor / sizeof(int) - 1) {
				cadence_mode = "stop";
				n = 0;
				SerialUSB.print("Elapsed time: ");
				SerialUSB.println(micros() - playback_starttime);
			}
			n = n + 1;
		}
		else if (cadence_mode == "cad60_k2") { // swing 7
			// load one value per loop.
			int m = floor(n / upsample_factor);

			sensor_playback = cad60_kick2nd_1swing_max7p3[m];
			sensor_playback = sensor_playback * 1.4;

			//I_Synapse1 = sensor_playback * gyro_scale;
			if (n == sizeof(cad60_kick2nd_1swing_max7p3)* upsample_factor / sizeof(int) - 1) {
				cadence_mode = "stop";
				n = 0;
				SerialUSB.print("Elapsed time: ");
				SerialUSB.println(micros() - playback_starttime);
			}
			n = n + 1;
		}
		else if (cadence_mode == "cad50_tr") { // swing 8
			// load one value per loop.
			int m = floor(n / upsample_factor);

			sensor_playback = cad50_trip_1swing_max3p8[m];
			sensor_playback = sensor_playback + 0.28; // manual calibraion

			sensor_playback = sensor_playback * 1.2;

			//I_Synapse1 = sensor_playback * gyro_scale;
			if (n == sizeof(cad50_trip_1swing_max3p8)* upsample_factor / sizeof(int) - 1) {
				cadence_mode = "stop";
				n = 0;
				SerialUSB.print("Elapsed time: ");
				SerialUSB.println(micros() - playback_starttime);
			}
			n = n + 1;
		}
		else if (cadence_mode == "cad60_tr") { // swing 9
			// load one value per loop.
			int m = floor(n / upsample_factor);

			sensor_playback = cad60_trip_1swing_max3p6[m];
			sensor_playback = sensor_playback + 0.28; // manual calibraion
			sensor_playback = sensor_playback * 1.5;

			//I_Synapse1 = sensor_playback * gyro_scale;
			if (n == sizeof(cad60_trip_1swing_max3p6) * upsample_factor / sizeof(int) - 1) {
				cadence_mode = "stop";
				n = 0;
				SerialUSB.print("Elapsed time: ");
				SerialUSB.println(micros() - playback_starttime);
			}
			n = n + 1;
		}
		else {
			SerialUSB.println("cadence_mode not found");
			/*SerialUSB.print("Elapsed time: ");
			SerialUSB.println(micros() - playback_starttime);*/

		}

		gyro_scale *= playback_scale;

		if (cadence_mode == "stop") { // do nothing 
			I_Synapse1 = 0;
			SerialUSB.println(I_Synapse1);
		}
		else {
			// from here the same process as in the gyro_sender but the gyro data is float (recorded in MArch 2021)
			float pos_gyro_z = sensor_playback + 0.28; // manual calibraion
			pos_gyro_z = (pos_gyro_z > 0) ? pos_gyro_z : 0; // need to calubrate
			//gyrodata = (pos_gyro_z * gyro_scale); // TODO:  convert to uint8_t.  event.gyro.z ranges from -10 to 10 ish.  offset: -0.56.
			//gyrodata = (gyrodata > 255) ? 255 : gyrodata; // make sure the range is 8 bit.
			I_Synapse1 = (pos_gyro_z * gyro_scale) * 1;// gyro_scale_high_spike_train; // for higher spike rate
			SerialUSB.println(I_Synapse1);
		}



		// playback additional slow down 5/6/2021
		delayMicroseconds(500); //check the frequency. 

	} // end of playback


	I_Synapse1 = (I_Synapse1 > 0) ? I_Synapse1 : 0;  // positive flextion only.


	analogWrite(A0, setDAC(I_Synapse1));


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

		//if (wired_mode) {
		//	digitalWrite(PIN_LED, LOW);
		//}

		 if (v >= 30.0) {  // typical IZ Neuron 
			v = Array_c[NeuronBehaviour];
			u += Array_d[NeuronBehaviour];
			//digitalWrite(PIN_NEURON1, HIGH);
			//SerialUSB.println("\tspike 1");
			//spikeout = true;
	
			//if (wired_mode) {
			//	digitalWrite(PIN_LED, HIGH);
			//}

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
		//SerialUSB.print("FR Mode: ");
		//SerialUSB.println(I_Synapse1);
		delayMicroseconds(70); // to match the playback speed between IZN and FR.
		float FR_threshold = 0.3;

		if (exp3_mode == "MODE3_idle_run_both_50hz") {
			configureTCC1_rate(50);
		}
		else if (exp3_mode == "MODE2_idle_lowfreq_stim") {
			if (I_Synapse1 > FR_threshold* gyro_scale) { //e.g. 0.3 * 30 = 9
				configureTCC1_rate(200);
			}
			else {
				configureTCC1_rate(35);
			}
		}
		else if (exp3_mode == "MODE4_idle_200_run_300Hz") {
			if (I_Synapse1 > FR_threshold* gyro_scale) {
				configureTCC1_rate(300);
			}
			else {
				configureTCC1_rate(200);
			}
		}
		else { // MODE1 (idle silent)
			if (I_Synapse1 > FR_threshold* gyro_scale) {
				configureTCC1_rate(200);
			}
			else {
				TCC1->CTRLA.bit.ENABLE = 0;                     // Disable the TCC1 counter
				while (TCC1->SYNCBUSY.bit.ENABLE);              // Wait for synchronizatio
			}
		}

	} // end of fixed rate mode
	
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
	//if (runmode == "playback") {
	//	delay(3); // control this for playing pre-recorded swing data. 
	//}
	

	
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


// configure TCC1 pulse frequency 

void configureTCC1_rate(int pulse_frequency) {
	int temp_tcc1_per = 48000000 / 8 / pulse_frequency;

	TCC1->PERB.reg = temp_tcc1_per;  //  f = 48M / 8 / 150000 = 40Hz                          
	while (TCC1->SYNCBUSY.bit.PERB);                  // Wait for synchronization

	TCC1->CTRLA.bit.ENABLE = 1;                     // enable the TCC1 counter
	while (TCC1->SYNCBUSY.bit.ENABLE);              // Wait for synchronizatio
}

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

	// FROM: RFM_relay
	PORT->Group[g_APinDescription[PIN_PULSEOUT_TO_CWU].ulPort].PINCFG[g_APinDescription[PIN_PULSEOUT_TO_CWU].ulPin].bit.PMUXEN = 1;

	// Connect the TCC timers to the port outputs - port pins are paired odd PMUO and even PMUXE
	// F & E specify the timers: TCC0, TCC1 and TCC2
	PORT->Group[g_APinDescription[PIN_PULSEOUT_TO_CWU].ulPort].PMUX[g_APinDescription[PIN_PULSEOUT_TO_CWU].ulPin >> 1].bit.PMUXE = PORT_PMUX_PMUXE_F_Val; //PA18, TCC0-WO2 (CC[2]),  (PA16, TCC0-WO6 (CC[2]))
	// END of 'FROM: RFM_relay'



	//***  pin 11 (PA16) pin broken for this RFM_relay feather board. *** //
	//PORT->Group[g_APinDescription[PIN_THIN_PULSEOUT].ulPort].PINCFG[g_APinDescription[PIN_THIN_PULSEOUT].ulPin].bit.PMUXEN = 1;

	//// Connect the TCC timers to the port outputs - port pins are paired odd PMUO and even PMUXE
	//// F & E specify the timers: TCC0, TCC1 and TCC2
	//PORT->Group[g_APinDescription[PIN_THIN_PULSEOUT].ulPort].PMUX[g_APinDescription[PIN_THIN_PULSEOUT].ulPin >> 1].bit.PMUXE = PORT_PMUX_PMUXE_F_Val; //PA16, TCC0-WO6 (CC[2])


// Feed GCLK4 to TCC0 and TCC1
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TCC0 and TCC1
		GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
		GCLK_CLKCTRL_ID_TCC0_TCC1;   // Feed GCLK4 to TCC0 and TCC1
	while (GCLK->STATUS.bit.SYNCBUSY);                // Wait for synchronization  

  // TCC0 setting 
	TCC0->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;          // Single slope PWM operation     
	while (TCC0->SYNCBUSY.bit.WAVE);                 // Wait for synchronization

	TCC0->PERB.reg = 30000;//                          z
	while (TCC0->SYNCBUSY.bit.PERB);                  // Wait for synchronization
	TCC0->CC[2].reg = 1500;
	while (TCC0->SYNCBUSY.bit.CC2);

	//REG_TCC0_INTFLAG |= TCC_INTFLAG_OVF | TCC_INTFLAG_MC0 | TCC_INTFLAG_MC1;              // Clear the overflow interrupt flag
	//REG_TCC0_INTENSET = TCC_INTENSET_OVF | TCC_INTENSET_MC0 | TCC_INTENSET_MC1;            // Enable TCC1 overflow interrupt
	//while (TCC0->SYNCBUSY.bit.WAVE);                // Wait for synchronization

	TCC0->CTRLBSET.reg |= TCC_CTRLBSET_ONESHOT;// | TCC_CTRLBSET_DIR;  // Is this right way to do down-counting and one shot mode together? I concur. 
	//last_cmd = TCC_CTRLBSET_DIR;
	while (TCC0->SYNCBUSY.bit.CTRLB);                 // Wait for synchronization

	//TCC0->DRVCTRL.reg |= TCC_DRVCTRL_NRE6;
	TCC0->DRVCTRL.reg |= TCC_DRVCTRL_NRE2; // for PA18 (pin 10) 



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

	PORT->Group[g_APinDescription[PIN_FR].ulPort].PINCFG[g_APinDescription[PIN_FR].ulPin].bit.PMUXEN = 1;

	// Connect the TCC timers to the port outputs - port pins are paired odd PMUO and even PMUXE
	// F & E specify the timers: TCC0, TCC1 and TCC2
	PORT->Group[g_APinDescription[PIN_FR].ulPort].PMUX[g_APinDescription[PIN_FR].ulPin >> 1].bit.PMUXO = PORT_PMUX_PMUXO_E_Val; //PA07, TCC1/WO[1]


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



// Fixed Rate (FR)  EIC trigger
void interrupt_pulsegen_FR()
{
	// the following works for thin pulse out.  
	TCC0->CTRLBSET.reg = TCC_CTRLBSET_CMD_RETRIGGER;         // Retrigger the timer's One/Multi-Shot pulse
	while (TCC0->SYNCBUSY.bit.CTRLB);                        // Wait for synchronization
}



// I2C receiver
void requestEvent() {
	byte gaitstate;
	//gaitstate2bits = gyro_source & 0x03; //use first two bits (b00000-gyro_source) , gyrp_source 1: left leg, 2:right leg, 0: no movement  
	uint8_t current_level = 5;  // 0, 1, 2, 3, 4, 5, 6, 7 (3 bits, 8 levels)
	gaitstate = ((current_level & 0x07) << 2) + (gyro_source & 0x03) & 0xff;

	Wire.write(gaitstate); // respond with message of 1 bytes

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

