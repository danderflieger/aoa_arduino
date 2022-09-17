/*********** Licensing, Information and Warnings - READ THIS PLEASE ****************

  AngleOfAttackSensor_1.0
  Author: Dan DeFord

  MIT License

  Copyright (c) 2021 danderflieger.com
  Copyright (c) 2020 Seeed Technology
            (AS5600 library https://github.com/Seeed-Studio/Seeed_Arduino_AS5600)

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.


* ************************************ WARNING: ***************************************            
* This software is free to use, but you agree to use it at your own risk.             *
* By using it you agree that any harm sustained to yourself or others is YOUR         *    
* responsibility as Pilot in Command of your aircraft. If you are not able to accept  *            
* that risk, do NOT use this software or the device for which it's written.           *  
                                                                                      *
* It is important to understand your own aircraft and know it's capabilities and      *            
* limitations before relying on any homemade device like that for which this code     *        
* was developed. It is simply another device that can improve situational awareness   *          
* in the cockpit, but you should NOT rely on it as a primary flight instrument.       *      
* *************************************************************************************

  The code below will connect an Arduino Nano 3.3 BLE or Arduino Nano 3.3 IoT board to a 
  rotational sensors that will provide angle of attack information using a windvane on a 
  free-rotating shaft (e.g. using roller bearings). The body of the sensor is designed
  to be 3D printed (e.g. additive manufacturing). That data is then reported via a Bluetooth 
  connection to a nearby Android device running an app that is designed to subscribe to 
  the sensor data and display the data graphically.

  Electronic Hardware Requirements
  This code was developed for specific electronic devices that are easily obtained on the internet.

  What you will need (I'm fairly confident that these URLs will stay active until the 
  chips is no longer available):
  
    - 1x Aduino Nano 3.3 BLE (https://store-usa.arduino.cc/collections/boards/products/arduino-nano-33-ble)
   OR
    - 1x Arduino Nano 3.3 IOT board (https://store-usa.arduino.cc/collections/boards/products/arduino-nano-33-iot)
      
  
  
    - 1x AS5600 Magnetic Rotation Encoder.
       I had a fairly difficult time finding these anywhere other than Aliexpress. I'm
       not confident AT ALL that these will remain available at the links below. There are
       several variants of the AS5600 on printed circuit boards of varying sizes. The ones you
       want are white and roughly square shaped, measuring roughly 23mm x 23mm with the actual
       sensor chip in the middle of the board with mounting holes in each corner - and I suggest
       purchasing them with an included magnet - it takes a special magnet (diametrically magnetized
       around the circumfrence rather than on each end) to trigger the sensor. I'm adding several
       links in case any of the sellers on aliexpress decide to stop selling them:
       https://www.aliexpress.com/item/1005001636565407.html
       https://www.aliexpress.com/item/1005001709088354.html
       https://www.aliexpress.com/item/4001240626350.html
  
    - Though not necessary, I suggest several colors of solid-core, pre-tinned wire to make your AoA
       device more durable, easier to solder, and easier to distinguish which parts are connected to
       each other. Here's a good option (and what I used during my prototyping):
       https://www.amazon.com/gp/product/B07TX6BX47/

  Note:
  Obviously, beyond electronics there are also several other mechanical parts needed to make a
  functional AoA sensor. Go check out my website (https://www.DanDerFlieger.com) for info on parts as 
  well as a single place where you can download STL files for 3D printing. The STL files are designed 
  around the parts list  I've mentioned.
  
  Also note: I will not be changing my 3D models because someone wants to try some other sensor or 
  bearings or shafts or whatever. Perhaps, like me, you are an EAA member and have access to a 
  low-cost Solidworks installation for personal use. Feel free to design your own 3D printable 
  parts and run with it.

  Also, I have no interest in becoming a distributor of parts (purchased or printed). The parts I purchased on
  Aliexpress had a fairly long lead time. I used them because I couldn't find the parts on Amazon or any other
  retailer with faster shipping times. If someone prefers to purchase them in bulk and then sell them on eBay or
  Amazon, feel free. I don't have the time or interest to do so.

  In terms of 3D printing, I have an inexpensive 3D printer that I paid about $250USD for in 2018 or so. It's
  the Creality Ender 3 and it works pretty well for printing parts in PLA or PETG plastic. If you want something more
  durable, I suggest printing your parts with PETG or ABS (my preference is PETG). I have not had good luck with ABS
  on the Ender 3, so you'll probably need a better printer if you want to print the parts in ABS.

***********************************************************/



/************** Arduino code starts here ****************
  Note: the AS5600 library can be downloaded from Seeed-Studio's github repository
    Go to https://github.com/Seeed-Studio/Seeed_Arduino_AS5600 and click on the green
    button that says "Code," then click "Download ZIP" This file will need to be imported
    into your Arduino IDE (either installed or online). If you opt to use the browser-based IDE,
    you'll still need to install the Agent software on your computer to connect to the Arduino
    device. I've done both options and both work.
**/

/** Add the required code libraries **/
#include <ArduinoBLE.h> // library for utilizing Bluetooth Low Energy 
#include <Wire.h>       // library required for I2C communications
#include <AS5600.h>     // library for the AS5600 magnetic rotational encoder
#include <Arduino_LSM6DS3.h>


/** 
 Instantiate the Serial connection for troubleshooting - information can
 appear on the Arduino Serial Monitor if you have the Arduino hardware connected 
**/
#define SERIAL Serial
#define SYS_VOL   5


/** create a few public variables that will be used later on **/

// defining an object to communicate with the rotational sensors
AMS_5600 ams5600; 

// defining a keep a copy of the last readings so the arduino doesn't 
// waste cycles retransmitting something it has already transmitted
float lastAnglePitch  = 0.0; 
float lastTurnRate = 0.0;
float lastSlipSkid = 0.0;


// set time intervals for blinking light signals (e.g. there is an LED mounted
// on the outside of the AoA sensor, and depending on how fast the LED blinks,
// you can tell which state it's in)
unsigned long connectedHighBlinkInterval  = 100;  // the amount of time the LED stays ON when connected to device
unsigned long connectedLowBlinkInterval   = 2500; // the amount of time the LED is off when connected to device
unsigned long powerHighBlinkInterval      = 500; // the amount of time the LED is ON when powered on, but not connected
unsigned long powerLowBlinkInterval       = 1000; // the amount of time the LED is off when powered on, but not connected

/** 
 *  I noticed that if I don't limit how often a message is pushed, 
 *  the amount of data can sometimes become overwhelming and the 
 *  indicator app may freeze. Increasing this number will extend 
 *  the amount of time between messages sent from the Arduino to 
 *  the phone (note, you must also change a corresponding value in
 *  the Android app). 
 *  
 *  Every 150ms should be a good compromise between 
 *  sending too much data and keeping the data as close to live as 
 *  possible. I also added a smoothing algorythm below that averages the
 *  readings gathered between the messages that it sends every 150ms.
 */ 
unsigned long messageSendInterval         = 250; 

float smoothReadings = 0.0;
float smoothTurnRate = 0.0;
float smoothSlipSkidReading = 0.0;
int smoothReadingsCount = 0;




/*** Prepare the LED Pin and set it to LOW (off) ***/
const int ledPin = 2; // 8; // LED_BUILTIN; //8;
int ledState = LOW;

/**************** millis() *************************
  The topic of millis() is often a bit confusing for new Arduino devleopers
  Essentially, you can use the millis() function to get a value that corresponds to
  the number of milliseconds (1/1000 of a second) since the Arduino was powered on.
  The value we're creating here (lastTimerReading) will hold a value that
  will be used to decide whether or not to turn the LED on and for how long, depending
  on the current state of the application. For example, if the Arduino has been
  running for 12000 milliseconds, we might set this "lastTimerReading" to 12000 before
  looping through the code again. If the difference between the duration the Arduino has
  been powered on and the "lastTimerReading" meets a specific threshold, we will turn the
  LED on or off or possibly reset the "lastTimerReading" variable to a new value.
  
  There is a very lengthy discussion on what an "unsigned long" variable is below. Read
  it if you are interested. Look for a section that says Signed vs. Unsigned Longs around 
  line 330 of this file 
****************************************************/

unsigned long lastBlinkTimerMillis;
unsigned int lastMessageTimerMillis;



/******** Bluetooth Services and Characteristics **************
* Next we'll create a Bluetooth Service and Characteristic (of type "String"). A Service is basically a
* server that you can connect to. A Characteristic is basically a datum you can subscribe to once you are
* connected to a Bluetooth Service. In this case, we are creating a service named "angleService" and a
* Characteristic named "angleCharacteristic".
* The Characteristic is being enabled with the ability to read the value and be subscribed to (notice the
* BLERead | BLENotify below). The 16 denotes the length of the String value we will pass to the Android device. 
* 
* Also notice the UIDs we're using to identify each object (each of the first portions are slightly different):
* 00000001 and 00000002, respectively. If ever I decide to add other Characteristics (for example, AHRS data,
* barometric data, temperature data, etc.), they will likely be 00000003 through whatever number of other
* Characteristics I decide to add.
**/
BLEService             angleService           ("00000001-627E-47E5-A3FC-DDABD97AA966");
//BLEFloatCharacteristic angleCharacteristic    ("00000002-627E-47E5-A3FC-DDABD97AA966", BLERead | BLENotify );
//BLEFloatCharacteristic turnRateCharacteristic ("00000003-627E-47E5-A3FC-DDABD97AA966", BLERead | BLENotify );
//BLEFloatCharacteristic slipSkidCharacteristic ("00000004-627E-47E5-A3FC-DDABD97AA966", BLERead | BLENotify );
//BLEFloatCharacteristic angleCharacteristic    ("00000002-627E-47E5-A3FC-DDABD97AA966", BLERead | BLENotify );
//BLEFloatCharacteristic turnRateCharacteristic ("00000003-627E-47E5-A3FC-DDABD97AA966", BLERead | BLENotify );
//BLEFloatCharacteristic slipSkidCharacteristic ("00000004-627E-47E5-A3FC-DDABD97AA966", BLERead | BLENotify );

BLEStringCharacteristic readingCharacteristic ( "00000002-627E-47E5-A3FC-DDABD97AA966", BLERead | BLENotify , 50);

/*********************************** 
  This is where the work begins. Arduinos have two required functions - setup() and loop(). setup() runs
  once when the Arduino fires up. loop() runs over and over until it is powered off. We will use the
  setup() function to ... obviously ... set up some things.
************************************/
void setup() {


  /***** Serial Interface **********
   * You've heard of a USB device, right? USB is an acronym for Universal Serial Bus. It's used to 
   * move data between a device and a computer in most cases. The Arduino also supports passing data
   * over a serial connection. Here we are setting up our serial interface using a baudrate of 115200. 
   * Baud rate is basically an agreed upon speed to communicate between two devices. Specifically,
   * the Arduino Nano (and many other Arduino boards) have a USB connector on the board. It can be 
   * used to power the Arduino, as well as pass data to and from it via Serial Rx (receive) and Tx 
   * (transmit). We can use our Arduino IDE to monitor data received from the Arduino device (look below
   * for lines that say SERIAL.println() - data within the paretheses will show up in the Serial monitor)
   *********************************/
  SERIAL.begin(115200);
  

  /***** LED Light Control ************
  Here we are telling our LED pin (Pin 8) to be used as an OUTPUT device - or in other words, this line of 
  code tells the Arduino that we want to be able to use Pin 8 (named ledPin) as output - in this case, 
  we'll connect an LED to it and turn it on and off using code. 
  *************************************/
  pinMode(ledPin, OUTPUT);

  delay(250); // wait a quarter second and let things warm up 
  
  /*** Wire library *****
   * Firing up the Wire library, which gives us access to the I2C bus - the connection 
   * we're using for our rotary encoder
  ***********************/
  Wire.begin(); 

  
  /***** Bluetooth Low Energy (BLE) Mode *****
    * Once the sensors are found to be working, fire up the Bluetooth module on the Arduino 
    * Notice the ! in front of the BLE.begin() statement. This means that if the opposite is 
    * true, do something. In this case, we're say that if the BLE object does NOT "begin", run 
    * the code between the brackets { }
  *******************************************/
  if (!BLE.begin()) {
    
    /*** While loops ***
     * The small code block below is what we call a while loop. It will run while whatever is in 
     * the parentheses is true. In this case, it's a little confusing. What we're doing is telling 
     * our code to stop here and toss an error out to the Serial monitor over and over every second
     * because, for some reason (probably a hardware issue) the BLE object didn't "begin" 
     * If the BLE object DOES start, this while loop will never run.
    *******************/
    while (1) {
      SERIAL.println("BLE Start Failed!");
      delay(1000);
    }
  }
  /** Provided the BLE object begins properly, pop out a message and start setting up our BLE connection **/
  SERIAL.println("BLE Started");

  
  if (!IMU.begin()) {
    Serial.println("Failed to initiate IMU!");
  }
  SERIAL.println("IMU Started");

  
  /** Set name for connection **/
  BLE.setLocalName("DanDerFlieger");  
  
  /** Set Manufacturer Data - in this case I'm setting the manufName to "DanDerFlieger" **/
  //byte manufName[3] = { 0x44, 0x61, 0x6e, 0x44, 0x65, 0x72, 0x46, 0x6c, 0x69, 0x65, 0x67, 0x65, 0x72 };
  byte manufName[3] = { 0x44, 0x44, 0x72 };
  
  /** Setting the Manuf. Name using the byte array above, and telling our code that it's 13 bytes long **/
  BLE.setManufacturerData(manufName, 13); 


  /** Add the Bluetooth Service and Characteristic defined above **/
  //  BLE.setAdvertisedService(messageService);
  //  messageService.addCharacteristic(messageCharacteristic);
  //  BLE.addService(messageService);
  //  messageCharacteristic.setValue("Connection Established");
  
  //angleService.addCharacteristic(angleCharacteristic);
  //angleService.addCharacteristic(turnRateCharacteristic);
  //angleService.addCharacteristic(slipSkidCharacteristic);
  //angleService.addCharacteristic(messageCharacteristic);

  angleService.addCharacteristic(readingCharacteristic);
  BLE.setAdvertisedService(angleService);
  BLE.addService(angleService);

 /** Start advertising the Bluetooth service(s) **/
  BLE.advertise();  
  delay(1200); /** give the Bluetooth hardware a second to start up **/
  
  /** display that something is happening on the Serial monitor **/
  Serial.print("Peripheral device MAC: "); Serial.println(BLE.address());
  Serial.println("Waiting for connections...\n");

}



/********* Main LOOP of the application *******************
 * Start the main loop of the code. This section repeats over and over as long as  the Arduino is powered on
**********************************************************/

void loop() {


  /** Create a new BLEDevice object (named "central" here) **/
  BLEDevice central = BLE.central();

  /** As long as "central" starts up correctly (basically, is your Android device is connected to
   * the Arduino via Bluetooth), start doing stuff 
  **/
  if (central) {

    /** Display some connection info Serial Monitor **/
    SERIAL.print("Connected to central MAC: "); SERIAL.println(central.address());

    /** light up the LED when a connection is made **/
    digitalWrite(ledPin, HIGH);
    
    /** Signed vs. Unsigned Longs
     *  
     * Next, we'll figure out how many milliseconds it's been since the Arduino was powered on using the "millis()" function below
     * and put that value in an "unsigned long" variable. 
     * 
     * A signed long is an integer that can range from -2,147,483,648 to 2,147,483,647. An unsigned long (which we're using here) can hold
     * values between 0 and 4,294,967,294. Notice that's double the upper range of the signed long. 
     * 
     * Here's some info you probably either don't care about or already know, but I'll tell you anway:
     * 
     * In computer science, everything is broken into 0s and 1s. One "bit" is either a 1 or a 0. A byte is a string of 8 bits.
     * (for example, 00000001 is the same as the decimal number 1 to us, but 00000010 is equal to 2, and 00000011 is equal to 3). 
     * 
     * A "long" (sometimes called a "signed long") uses 4 bytes to hold its data and one of the bits (let's say the first because I can't remember 
     * which) is used as the "sign" for positive or negative. So the decimal number -1 would translate into 00000000 00000000 00000000 00000001 in 
     * binary (where the first bit (0) in the list means it's negative number and the other 31 bits are the numeric value). 
     * 
     * A positive 1 might translate into 10000000 00000000 00000000 00000001, for example; this time, where the very first bit (1) denotes a positve number.
     *
     * However:
     * Since we're using an "unsigned" long, the first bit is actually part of an always-positive number rather than being wasted as a -/+ sign 
     * (which we would never use since the millis() function only reports positive numbers), so we can use the first bit to effectively double the 
     * highest number we can hold in that "unsigned long" variable. 
     * 
     * The highest value of milliseconds we can hold in the unsigned long will be 4,294,967,294 milliseconds. The Arduino would have to
     * be powered on a looooong time (no pun intended) to reach the top of an "unsigned long" variable: It would have to be powered on constantly for 
     * almost 50 days to do so. A "long" is the largest integer type that Arduino supports (though we could probably do some binary
     * trickery to make a larger variable). But the likelihood of someone leaving their AoA sensor on for over 50 continuous days is very unlikely, 
     * so we'll simply use an unsigned long value instead to make this easy. :)
     **/
    unsigned long currentMillis = millis();

    /*
       Now that the Bluetooth connection is established, check to see if the magnet 
       is in the correct position to get a good reading 
    */
    if (ams5600.detectMagnet() == 0 ) {

      /* if it's not, keep trying to read the sensor until you get a good reading */
      while (1) {

        if (ams5600.detectMagnet() == 1 ) {

          SERIAL.print("Beginning Current Pitch Magnitude: ");
          SERIAL.println(ams5600.getMagnitude());

//          float x, y, z;
//          if(IMU.gyroscopeAvailable()) {
//            IMU.readGyroscope(x, y, z);
//            SERIAL.print("Turn Rate: ");
//            SERIAL.print(x);
//            SERIAL.print("\t");
//            SERIAL.print(y);
//            SERIAL.print("\t");
//            SERIAL.println(z);
//            
//          }
//          
          /* this kicks the code out of the while loop once the magnet is found */
          break; 

        }
        else {

          SERIAL.println("Can not detect Pitch magnet");
          //messageCharacteristic.setValue("Check Pitch Magnet");

        }

        delay(1500);
      }

    }


    /* Enter a loop that lasts as long as there is a device connected to the Arduino's BLE server */
    while (central.connected()) {

      /* Here we grab a new millis() value to compare to the lastTimerReading above to decide if we
          want to continue reading the data or wait a little longer
      */
      unsigned long timerMillis = millis();

      /* Check to see if the time difference between the current millis() value is
          greater than 250ms (e.g. has it been more than a quarter of a second since
          the data was passed over Bluetooth?) - if not, wait a little longer. If so,
          go ahead and update the values and pass them over Bluetooth to the Android device
      */
      if (timerMillis - lastBlinkTimerMillis < connectedHighBlinkInterval) {
        digitalWrite(ledPin, HIGH);
      } else if (timerMillis - lastBlinkTimerMillis < connectedLowBlinkInterval) {
        digitalWrite(ledPin, LOW);
      } else {
        lastBlinkTimerMillis = timerMillis;
      }

      /* 
        Because the AS5600 reports angles from 0 to ~365 and we want to see positive
        and negative values, let's subtract 180 from the value reported by the AS5600.
        This will give us a value of somewhere between -180 and +180 degrees instead. 
        That way 0 degrees will be close to level flight (as long as we assemble the 
        AoA sensor with the windvane at close to level flight when read at the AS5600).
      */
      float rawAnglePitch = convertRawAngleToDegrees(ams5600.getRawAngle()) - 180;
      float gX, gY, gZ, aX, aY, aZ;
      
      
      IMU.readGyroscope(gX, gY, gZ);
      float turnRate = gZ;

      IMU.readAcceleration(aX, aY, aZ);
      float slipSkid = aY;

      /* 
        Check to see if the current value is the same as it was the last time through.
        We don't want to send the same value to the indicator for no reason - there's 
        nothing to change, so save the energy and just iterate through the loop again 
        instead.
      */
      
      //if (rawAnglePitch != lastAnglePitch) { // || turnRate != lastTurnRate || slipSkid != lastSlipSkid) {
        /* if so, push the update out over the Bluetooth connection */
        float pitchAngle = rawAnglePitch;

        if (timerMillis - lastMessageTimerMillis < messageSendInterval) {
          // Keep tally of the readings and the number of times the code has run
          // since the last readings were sent. 
          smoothReadings += pitchAngle;
          smoothTurnRate += turnRate;
          smoothSlipSkidReading += slipSkid;
          smoothReadingsCount++;
          
        } else {

          // Once we reach our send threshold, average the values using the 
          // tally for each divided by the number of times the code ran since 
          // the last message was sent
          pitchAngle = smoothReadings / smoothReadingsCount;
          turnRate = smoothTurnRate / smoothReadingsCount;
          slipSkid = smoothSlipSkidReading / smoothReadingsCount;
          
          /* reset the last readings with the current ones */
          lastAnglePitch = pitchAngle;
          lastTurnRate = turnRate;
          lastSlipSkid = slipSkid;
          

          /* update the Bluetooth Characteristic - any devices subscribed to the Characteristic
             should see their data updated since we configured it to BLENotify */
//          SERIAL.print( turnRateCharacteristic  .setValue(turnRate) );
//          SERIAL.print( angleCharacteristic     .setValue(pitchAngle) );
//          SERIAL.println( slipSkidCharacteristic  .setValue(slipSkid) );

          String readings = String(pitchAngle) + "," + String(turnRate) + "," + String(slipSkid);
          SERIAL.println(readings);
          readingCharacteristic.setValue(readings);
          //char readingBytes [readings.length()+1];
          //readings.toCharArray(readingBytes);
//          readings.toCharArray(readingBytes, readings.length());
//          readingCharacteristic.setValue(readingBytes);

          //SERIAL.println (readingBytes);
          
  
          //SERIAL.println( readingCharacteristic .setValue(readingCharacteristic));


//          String output = String(pitchAngle) + String(",") + String(turnRate) + String(",") + String(slipSkid);
//
//          SERIAL.println(output);
          
  
          /* write the data out to the Serial line to view for debugging. */
//          SERIAL.print(pitchAngle);
//          SERIAL.print(',);
//          SERIAL.print(turnRate);
//          SERIAL.print(',');
//          SERIAL.println(slipSkid);

          // reset the counters in preparation for the next message to be sent
          lastMessageTimerMillis = timerMillis;
          smoothReadings = 0.0;
          smoothTurnRate = 0.0;
          smoothSlipSkidReading = 0.0;
          smoothReadingsCount = 0;
          
        }
      //}
    }

    /* 
      Since the while loop above only runs until there are no more devices connected,
       if we have arrived here, there are no more connections. So we will turn off the
       onboard LED and write a disconnect message out to the Serial interface for debugging. 
    */
    digitalWrite(LED_BUILTIN, LOW);
    SERIAL.print("Disconnected from central");

  }
  
  
  /** As long as there's nothing connected, change the LED blinks to a different pattern **/
  //unsigned long currentMillis = millis();
  unsigned long timerMillis = millis();

  /* Check to see if the time difference between the current millis() value is
      greater than "powerHighBlinkInterval" defined at the top of the program.
      Then turn the LED on or off at the specified interval.
  */
  if (timerMillis - lastBlinkTimerMillis < powerHighBlinkInterval) {
    digitalWrite(ledPin, HIGH);
  } else if (timerMillis - lastBlinkTimerMillis < powerLowBlinkInterval) {
    digitalWrite(ledPin, LOW);
  } else {
    lastBlinkTimerMillis = timerMillis;
  }

}

/********************************************************************
  Function: convertRawAngleToDegrees
  In: angle data from AMS_5600 - an integer between 0 and 4095
  Out: human readable degrees as float
  Description: takes the raw angle from the sensor and
  calculates a float value in degrees.
*********************************************************************/
float convertRawAngleToDegrees(word newAngle) {
  /* Raw data reports 0 - 4095 segments, which is 0.087 of a degree */
  float retVal = newAngle * 0.087;
  return retVal;
}


/********************************************************************
  Function: changeMuxPort
  In: the I2C port you want to update - an Integer between 0 and 7
  Out: N/A
  Description: receives a port number and sends a signal to the
               I2C Multiplexer to switch to the requested port.
               
  Note: this bit of code was originally added because I have eventual
  plans to add a second rotary encoder that measures yaw, too. Since 
  the AS5600 has a hard-coded I2C address, there's no way to distinguish
  between the pitch and yaw encoders without a separate piece of 
  hardware called a Multiplexer. In simple terms, a multiplexer is 
  sort of like a telephone switchboard operator who connects calls 
  by plugging wires into sockets. In this case, you "call" the 
  "operator" and ask to be connected to a specific sensor. Once connected,
  you ask for the sensor's reading. The multiplexer will stay connected
  to the device until you ask it to connect to another one of its many 
  device interfaces. From your perspective, though, you only ever call
  the operator to re-route the call
*********************************************************************/
void changeMuxPort(uint8_t bus) {
  /* remember that the Multiplexer needs to have its A0 jumper soldered
     or the I2C port will be 0x70 instead - this won't work with the
     Arduino Nano 33 BLE board - it appears to use 0x70 for the
     Bluetooth module */
  Wire.beginTransmission(0x71);
  Wire.write(1 << bus);
  Wire.endTransmission();

  /* pause for 50ms to let the Multiplexer do its thing before carrying
     on - this may or may not be necessary .... */
  delay(50);
}
