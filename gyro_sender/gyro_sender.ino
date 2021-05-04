/*
 Name:		gyro_sender.ino
 Created:	4/23/2021 10:19:41 AM
 big change: 4/23/2021.  integrate spikeencoder and RFM_relay into a single solution. IZN neuron needs to run in the previously RFM_relay
			 to overcome the wireless pulse rate cap (130Hz) wiht the RFM.  We send the leg kinematic instead thru RFM and generate spikes here.

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


#if defined(ARDUINO_SAMD_ZERO)
// Required for Serial on Zero based boards
#define Serial SerialUSB
#endif

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



#if IS_L3GD20
int gyro_scale_orig = 30;   //40 is too high that it clips.  (0-255) 30;//30 ; // gyro values typically < 1000. , if 150,too easily saturate to max (~100Hz)
#elif ISMPU6050
int gyro_scale = 1;
#endif



int  n = 0;


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


uint8_t gyrodata = 0;  // 8 bit encoded gyro data... 
String device_mode = "on";
boolean forward_swing = false;
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

	
	pinMode(PIN_LED, OUTPUT);

}


int16_t packetnum = 0;  // packet counter, we increment per xmission



void rf_send() {
	//SerialUSB.println("Sending to rf95_server");
 // Send a message to rf95_server


 // ** rf95.send() only when there is a spike.  **// {
	
	if (gyrodata > 15) {
		digitalWrite(PIN_LED, HIGH);
	}
	//noInterrupts();
	uint8_t radiopacket[2];
	radiopacket[0]= gyrodata;
	radiopacket[1] = '\0';

	//char buf[1];
	//sprintf(buf, "%d", gyrodata);
	//byte sendLen = strlen(buf);


	//itoa(packetnum++, radiopacket, 10);
	SerialUSB.print("Sending..... "); SerialUSB.println(radiopacket[0]);
	//radiopacket[len] = '\0';
	//radiopacket[0] = 0;


	rf95.send((uint8_t*)radiopacket, 2);

	
	digitalWrite(PIN_LED, LOW);

	//interrupts();
	
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


void loop() {
	time_us = micros();

	gyro_scale = gyro_scale_orig;

	// check system time in microseconds
	//unsigned long currentMicros = micros() - startMicros;


	rf_send();



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

		// device mode setting 
		if (command == 'X') {
			device_mode = "on";
			SerialUSB.println(F("Device mode: on"));
		}
		else if (command == 'Y') {
			device_mode = "off";
			SerialUSB.println(F("Device mode: off"));
		}
		else {
			SerialUSB.println(F("Unknown runmode"));
		}

	}
	//SerialUSB.println(size_of_array); //


	if (device_mode == "on") { // transmit to spikeencoder_RFM_relay

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
	

		float pos_gyro_z = event.gyro.z + 0.28; // manual calibraion
		pos_gyro_z = (pos_gyro_z > 0) ? pos_gyro_z : 0; // need to calubrate
		//SerialUSB.print("(pos)out_of_gyro: ");
		//SerialUSB.println(pos_gyro_z);
		gyrodata = (uint8_t) (pos_gyro_z * gyro_scale); // TODO:  convert to uint8_t.  event.gyro.z ranges from -10 to 10 ish.  offset: -0.56.
		// so the rationale for type casting to uint8_t was to make sure the range is 0-255. But this causes the gyro data to clip ! scale it here.  

		//gyrodata = (gyrodata > 255) ? 255 : gyrodata; // make sure the range is 8 bit.
		//SerialUSB.print("uint8 converted: ");
		//SerialUSB.println(gyrodata);

#endif

#if IS_MPU6050
		//accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
		accelgyro.getRotation(&gx, &gy, &gz);
		//I_Synapse1 = int(gz * gyro_scale); // cheap
		gyrodata = gz * gyro_scale; // cheap
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


		gyrodata = GyroZ * gyro_scale; // cheap
		SerialUSB.println(gyrodata);

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
	else if (device_mode == "off") {
		gyrodata = 0; // TODO: just turn off the sending to save power?

	}else {

		SerialUSB.print("Elapsed time: ");
		SerialUSB.println(micros() - playback_starttime);

		
	}

	

	forward_swing = gyrodata > 0;

	gyrodata = forward_swing ? gyrodata : 0;  // positive flextion only.


	//SerialUSB.print("delay ");
	//SerialUSB.println(micros() - time_us); //prints time since program started

}// end of loop()



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

