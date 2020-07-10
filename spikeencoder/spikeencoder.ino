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

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <RH_RF95.h>


#if defined(ARDUINO_SAMD_ZERO)
// Required for Serial on Zero based boards
#define Serial SerialUSB
#endif

#define GYRO_DEVICE true
#define IZHIKEVICH_NEURON_ENCODING true

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
#define RF95_FREQ 403.5

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
#define PIN_PA07  9
#define PIN_NEURON  10	// PA18 
#define PIN_PUSLEIN  12// PA19
#define PIN_THIN_PULSEOUT  11// PA16


#if GYRO_DEVICE
/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (50)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

#endif
///////////////////////////////////////////////////////////////////////////
// KEY PARAMETERS TO SET BY USER  /////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

int   nosound = 0;    // 0 default, click on spike + digi out port active. 1 switches both off
int   noled = 0;    // 0 default, 1 switches the LED off
int   AnalogInActive = 1;    // default = 1, PORT 3 setting: Is Analog In port in use? Note that this shares the dial with the Syn2 (PORT 2) dial

float PD_Scaling = 0.5;  // the lower the more sensitive.                       Default = 0.5

float Synapse_decay = 0.995;// speed of synaptic decay.The difference to 1 matters - the smaller the difference, the slower the decay. Default  = 0.995
float timestep_ms = 0.1;  // default 0.1. This is the "intended" refresh rate of the model.
							  // Note that it does not actually run this fast as the Arduino cannot execute the...
							  // ...full script at this rate.  Instead, it will run at 333-900 Hz, depending on settings (see top)

// set up Neuron behaviour array parameters
int   nModes = 5; // set this to number of entries in each array. Entries 1 define Mode 1, etc..

  // Izhikevich model parameters - for some pre-tested behaviours from the original paper, see bottom of the script
float Array_a[] = { 0.02,  0.02, 0.02, 0.02, 0.02 }; // time scale of recovery variable u. Smaller a gives slower recovery
float Array_b[] = { 0.20,  0.20, 0.25, 0.20, -0.1 }; // recovery variable associated with u. greater b coules it more strongly (basically sensitivity)
int   Array_c[] = { -65,   -50,  -55,  -55,  -55 }; // after spike reset value
float Array_d[] = { 6.0,   2.0, 0.05,  4.0,  6.0 }; // after spike reset of recovery variable

int NeuronBehaviour = 0; // 0:8 for different modes, cycled by button


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MAIN PROGRAMME - ONLY CHANGE IF YOU KNOW WHAT YOU ARE DOING !                                                                       //
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////
// Setup variables required to drive the model
float I_total;           // Total input current to the model
float I_Synapse;         // Total synaptic current of both synapses
float Synapse1Ampl;      // Synapse 1 efficacy

// initialise state variables for different inputs
boolean spike = false;
int     buttonState = 0;
int     SpikeIn1State = 0;
int     Stimulator_Val = 0;

int DigiOutStep = 0;     // stimestep counter for stimulator mode
int Stim_State = 0;      // State of the internal stimulator
float v; // voltage in Iziekevich model
float u; // recovery variable in Iziekevich model

//output_t Output; // output structure for plotting
String   OutputStr;

int gyro_scale = 4; // gyro values typically < 1000. 

//int startMicros = micros();

////////////////////////////////////////////////////////////////////////////
// SETUP (this only runs once at when the Arduino is initialised) //////////
////////////////////////////////////////////////////////////////////////////


void setup() {
	SerialUSB.begin(115200);
	while (!SerialUSB) {
		delay(1);
	}

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

#if GYRO_DEVICE
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

	
	//initializeHardware(); // Set all the PINs
	v = -65;
	u = 0;

	// setting for IZ NEURON AND GYRO_DEVICE
	// DISABLE FIXED RATE OUTPUT  (control mux to disable PWM and enable digitalWrite
	PORT->Group[g_APinDescription[PIN_NEURON].ulPort].PMUX[g_APinDescription[PIN_NEURON].ulPin >> 1].bit.PMUXE = PORT_PMUX_PMUXE_B_Val; //

	// IZ NEURON ENCODING
	


	pinMode(PIN_PUSLEIN, INPUT); // 
	attachInterrupt(PIN_PUSLEIN, interrupt_pulsegen_pulsein, RISING);

	//pinMode(PIN_PA07, OUTPUT);
	pinMode(PIN_THIN_PULSEOUT, OUTPUT); // 
	oneshot_TCC0_thinPulse_setup();  // 
	//fixed_rate_pulse_out_setup();

}


int16_t packetnum = 0;  // packet counter, we increment per xmission


void rf_send() {
	//SerialUSB.println("Sending to rf95_server");
 // Send a message to rf95_server


 // ** rf95.send() only when there is a spike.  **//
	if (spikeout) {
		//noInterrupts();
		char radiopacket[3] = " ";
		itoa(packetnum++, radiopacket, 10);
		SerialUSB.print("Sending... "); SerialUSB.println(radiopacket);
		radiopacket[2] = 0;


		rf95.send((uint8_t*)radiopacket, 3);
		spikeout = false;
		//interrupts();
	}

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

void loop() {

	// check system time in microseconds
	//unsigned long currentMicros = micros() - startMicros;

	rf_send();

	// IZ NEURON ENCODING
	if (IZHIKEVICH_NEURON_ENCODING) {
	
#if GYRO_DEVICE

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


		I_Synapse = int(gyro.y() * gyro_scale);
		I_Synapse = (I_Synapse > 0) ? I_Synapse : 0;  // positive flextion only.
		SerialUSB.print(" I_Synapse: ");
		SerialUSB.println(I_Synapse);

		// Decay all synaptic current towards zero
		//I_Synapse *= Synapse_decay;

		// compute Izhikevich model
		float I_total = I_Synapse; // Add up all current sources
		v = v + timestep_ms * (0.04 * v * v + 5 * v + 140 - u + I_total);
		u = u + timestep_ms * (Array_a[NeuronBehaviour] * (Array_b[NeuronBehaviour] * v - u));
		if (v >= 30.0) { 
			v = Array_c[NeuronBehaviour]; 
			u += Array_d[NeuronBehaviour]; 
			digitalWrite(PIN_NEURON, HIGH);
			SerialUSB.println("\tspike");
			spikeout = true;

			// the following works for thin pulse out.  
			TCC0->CTRLBSET.reg = TCC_CTRLBSET_CMD_RETRIGGER;         // Retrigger the timer's One/Multi-Shot pulse
			while (TCC0->SYNCBUSY.bit.CTRLB);                        // Wait for synchronization
		}

		else {
			digitalWrite(PIN_NEURON, LOW);
		}

		if (v <= -90) { v = -90.0; } // prevent from analog out (below) going into overdrive - but also means that it will flatline at -90. Change the "90" in this line and the one below if want to
		//int AnalogOutValue = (v + 90) * 2;


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


		/* Wait the specified delay before requesting nex data  (sensor) */
		delay(2);
#endif
	}
	//else { // FIXED RATE.
	//	//SerialUSB.println("FIXED RATE RESUMED");
	//	PORT->Group[g_APinDescription[PIN_NEURON].ulPort].PMUX[g_APinDescription[PIN_NEURON].ulPin >> 1].bit.PMUXE = PORT_PMUX_PMUXE_F_Val; //PA12, TCC0-6


	//}
  
}


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
	rf95.setTxPower(13, false);

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
	rf95.setModemConfig(RH_RF95::Bw500Cr45Sf128); ///
		//Bw125Cr45Sf128 = 0,    ///< Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on. Default medium range
		//Bw500Cr45Sf128,            ///< Bw = 500 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on. Fast+short range
		//Bw31_25Cr48Sf512,    ///< Bw = 31.25 kHz, Cr = 4/8, Sf = 512chips/symbol, CRC on. Slow+long range
	  //  Bw125Cr48Sf4096,           ///< Bw = 125 kHz, Cr = 4/8, Sf = 4096chips/symbol, CRC on. Slow+long range


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

	TCC1->PER.reg = 30000;//                          z
	while (TCC1->SYNCBUSY.bit.PER);                  // Wait for synchronization
	TCC1->CC[1].reg = 3000;
	while (TCC1->SYNCBUSY.bit.CC1);

	REG_TCC1_INTFLAG |= TCC_INTFLAG_OVF | TCC_INTFLAG_MC0 | TCC_INTFLAG_MC1;              // Clear the overflow interrupt flag
	REG_TCC1_INTENSET = TCC_INTENSET_OVF | TCC_INTENSET_MC0 | TCC_INTENSET_MC1;            // Enable TCC1 overflow interrupt
	while (TCC1->SYNCBUSY.bit.WAVE);                // Wait for synchronization

	TCC1->CTRLA.reg |= TCC_CTRLA_PRESCALER_DIV8 |    // Divide GCLhaveK4 by 1 (was 8)
		TCC_CTRLA_PRESCSYNC_RESYNC |   // set to retrigger timer on GCLK with prescaler reset
		TCC_CTRLA_ENABLE;             // Enable the TCC0 output
	while (TCC1->SYNCBUSY.bit.ENABLE);               // Wait for synchronization

	TCC1->CTRLA.bit.ENABLE = 1;                     // Disable the TCC1 counter
	while (TCC1->SYNCBUSY.bit.ENABLE);              // Wait for synchronization

	// added 20200709 because otherwise it never enters TCC1_Hander
	NVIC_SetPriority(TCC1_IRQn, 0);    // Set the Nested Vector Interrupt Controller (NVIC) priority for TCC to 0 (highest)
	NVIC_EnableIRQ(TCC1_IRQn);         // Connect TCC to Nested Vector Interrupt Controller (NVIC)

	SerialUSB.println("TCC1 for pulseout setting completed.");
}



#if GYRO_DEVICE

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


/**************************************************************************/
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