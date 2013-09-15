/*
 * Astromech RSeries Controller for the R2 Builders Club
 * Skwerl's Fork 
 *  
 * Heavily based on Michael Erwin's
 * RSeries Open Controller Project
 * http://code.google.com/p/rseries-open-control/
 *
 * Requires Arduino 1.0 IDE
 *
*/

/* 

	The goal of this sketch is to help develop a new wireless, touchscreen,
	R-Series Astromech Controller & Tranceiver system platform for the
	R2 Builders Club.
	
	Currently due to variable usage, we need more than 2048 bytes of SRAM.
	So we must use an Arduino MEGA 2560 or ATMEGA 2560 for the Transmitter
	
	Digi Xbee Series 2 & 2B PRO modules are FCC approved, and recommended.
	
	No FCC license is required. 
	Xbee S2 API Mode, ZigBee  & Radio Control communication understanding is helpful.
	
	It's going to take the output sent from the controller and interpret it into Servo & other data signals
	
	3 main servo outputs:
	servo1Pin = Joystick X, Forward/Reverse with Speed
	servo2Pin = Joystick Y, Left/Right
	servo3Pin = Accelerometer X, Dome Rotation
	
	However the controller sends all 5 motion channels from the handheld nunchuck controller to the receiver for completeness and future options.
	
	2 Default Nunchuck button activations:
	cbut = C Button Activate : make a ALERT or Scream noise. This is the TOP button on the Nunchuck
	zbut = Z Button Activate : make a CUTE random noise. This is the BOTTOM button on the Nunchuck
	
	32,767 virtual switches & configuration options via the LCD touchscreen
	We're going to limit our use 84 displayed events, 254 total events.
	If you need more than 84, feel free to mod the sketch in groups of 7.
	
	Receive & displaying telemetry from the Receiver.
	Charging Status = Future
	Battery Status = Implemented
	Current Draw = Implemented
	Calculates an Estimated run time = Future
	
	Nunchuck I2C Configuration Information
	UNO:         Green I2C Data= A4, Yellow I2C Clock A5
	MEGA 2560:   Green I2C Data = C21, Yellow I2C Clock Pin C22
	
	NOTE: Servo output code is in the Receiver sketch
	
	servoPin1 = Joystick X, Forward/Reverse with Speed
	servoPin2 = Joystick Y, Left/Right
	servoPin3 = Accelerometer X, Dome Rotation via A
	button1 = C Button Activate : Tell Astromech to make a ALERT or Scream noise.
	button2 = Z Button Activate : Tell Astromech to make a cute random noise.
	
	The I2C output from Nunchuck is made up of 6 bytes and stored in outbuf
	0 Joystick X
	1 Joystick Y
	2 Accel X
	3 Accel Y
	4 Accel Z
	5 C & Z Buttons

*/

/*////////////////////////////////////////////////////////////////////////////////////////////////*/
///////////////////////* Global Config *////////////////////////////////////////////////////////////
/*////////////////////////////////////////////////////////////////////////////////////////////////*/

#define astromechName "R2-D2"
#define VERSION "Forked 0.4.5"
#define OWNER "Skwerl"

/*////////////////////////////////////////////////////////////////////////////////////////////////*/
///////////////////////* Libraries *////////////////////////////////////////////////////////////////
/*////////////////////////////////////////////////////////////////////////////////////////////////*/

#include <Arduino.h>
#include <Adafruit_GFX.h>       // Core graphics library
#include <Adafruit_TFTLCD.h>    // Hardware-specific library
#include <TFTLCD.h>             // TFT LCD functions - adafruit.com @ https://github.com/adafruit/TFTLCD-Library
#include <TouchScreen.h>        // Touch Screen functions - adafruit.com @ https://github.com/adafruit/Touch-Screen-Library
#include <XBee.h>               // Using to get RSSI values in displayRSSI
#include <Wire.h>               // Used to read the I2C data from Nunchuck - Arduino
#include <ArduinoNunchuk.h>     // Arduino Nunchuk - Gabriel Bianconi @  http://www.gabrielbianconi.com/projects/arduinonunchuk/

/*////////////////////////////////////////////////////////////////////////////////////////////////*/
///////////////////////* RSeries Configuration *////////////////////////////////////////////////////
/*////////////////////////////////////////////////////////////////////////////////////////////////*/


//byte rxArray1[60];				// Allows for 60 Bytes to be read at once
									// XBee S2B only has room for about 100 packets of 54 bytes 
									// Arduino 022 only has 128 buffers for each Serial(0,1,2,3)
									// Also note use must use FastSerial.h 
     
int buttonval;

unsigned int rxVCC;
unsigned int rxVCA;

float dmmVCC;						// Display variable for Voltage from Receiver
float dmmVCA;						// Display variable for Amperage from Receiver

float testdmmVCA;					// Display variable for Amperage from Receiver

float previousdmmVCC = 00.0;
float previousdmmVCA = 00.0;

unsigned char telemetryVCCMSB = 0;
unsigned char telemetryVCCLSB = 0;
unsigned char telemetryVCAMSB = 0;
unsigned char telemetryVCALSB = 0;

float yellowVCC = 12.0;				// If RX voltage drops BELOW these thresholds, text will turn yellow then red.
float redVCC = 11.5;

float yellowVCA = 50.0;				// If current goes ABOVE these thresholds, text will turn yellow then red.
float redVCA = 65.0;

long lastStatusBarUpdate = 0;
long nextStatusBarUpdate = 0;

int updateSTATUSdelay = 3500;		// Update Status Bar Display every 5000ms (5 Seconds), caution on reducing to low

int ProcessStateIndicator = 0;		// Alternates between 0 & 1 High

int RSSI = 0;						// XBee Received packet Signal Strength Indicator, 0 means 0 Bars (No Signal)
int RSSIpin = 6;					// Arduino PWM Digital Pin used to read PWM signal from XBee Pin 6 RSSI 6 to 6 KISS
unsigned long RSSIduration;			// Used to store Pulse Width from RSSIpin
int xbRSSI = 0;						// XBee Received packet Signal Strength Indicator, 0 means 0 Bars (No Signal)

int xbATResponse = 0xFF;			// To verify Coordinator XBee is setup and ready, set to 0xFF to prevent false positives

long lastTriggerEventTime = millis();

// Define which digital pins on the arduino are for servo data signals 
//int servo1Pin = 6;    // Channel 1 - Left Right  
//int servo2Pin = 7;    // Channel 2 - Forward & Reverse Speed
//int servo3Pin = 10;   // Channel 3 - Dome Rotation 

// Servo Center Adjustment 
int chan1Center = 90;   // Channel 1 - Left Right  
int chan2Center = 90;   // Channel 2 - Forward & Reverse Speed
int chan3Center = 90;   // Channel 3 - Dome Rotation 

// Channel Adjustment 
int chan1Adjust = 0;    // Channel 1 - Left Right  
int chan2Adjust = 0;    // Channel 2 - Forward & Reverse Speed
int chan3Adjust = 0;    // Channel 3 - Dome Rotation 

// Channel Center Adjustment
int chan1Trim = 0;     // Channel 1 - Left Right  
int chan2Trim = 0;     // Channel 2 - Forward & Reverse Speed
int chan3Trim = 0;      // Channel 3 - Dome Rotation 

// Neutral Adjustments 
int chan1Neutral = 130; // Channel 1 - Left Right  
int chan2Neutral = 130; // Channel 2 - Forward & Reverse Speed
int chan3Neutral = 130; // Channel 3 - Dome Rotation 

// Neutral Width Adjustment, which takes Neutral + & - these values. So 120 to 140 would be considered Neutral or 90
int chan1Width = 10;    // Channel 1 - Left Right  
int chan2Width = 10;    // Channel 2 - Forward & Reverse Speed
int chan3Width = 10;    // Channel 3 - Dome Rotation 

                        // Nunchuck End Points Channel Adjustment 
int chan1Min = 27;      // Channel 1 Min - Left Right  
int chan1Max = 229;     // Channel 1 Max - Left Right
int chan2Min = 30;      // Channel 2 Min - Forward & Reverse Speed 
int chan2Max = 232;     // Channel 2 Max - Forward & Reverse Speed
int chan3Min = 77;      // Channel 3 Min - Dome Rotation  
int chan3Max = 180;     // Channel 3 Max - Dome Rotation 

//Servo chan1servo;  // create servo object to control a servo 
//Servo chan2servo;  // create servo object to control a servo 
//Servo chan3servo;  // create servo object to control a servo 

int loop_cnt=0;

/*////////////////////////////////////////////////////////////////////////////////////////////////*/
///////////////////////* Touchscreen Configuration *////////////////////////////////////////////////
/*////////////////////////////////////////////////////////////////////////////////////////////////*/

char* radiostatus = "...";

int menuScreens = 12;
int displaygroup = 1;
int displayitem = 1;
int scrollymin = 23;
int scrollyinc = 16;
int scrollymax = 167;
int scrollyloc = 23;

//		Note: menu Item code = byte sent to RSeries Receiver sketch
//		Triggering menuItem[42] will cause Byte equivelant of 42 to be sent, which is B00101010 
//
//		In future version this will move to a .txt file stored on micro-SD card
//
//		Keep Descriptions 13 Chars or less.
//
//							"12345678901234567890"	// This is just a sample of max length of 20 for TextSize=2
char* menuItem[] =		{   "1234567890123",		// DO NOT CHANGE or REMOVE this is Item 0

							"Alarm 1",				// Page 1
							"Alarm 2",
							"Cantina Song",
							"Doot Doot",
							"Failure",
							"Humming",
							"Leia Message",
							"Patrol",				// Page 2
							"Scream 1",
							"Scream 2",
							"Scream 3",
							"Scream 4",
							"Processing",
							"Short Circuit",
							"Startup Sound",		// Page 3
							"Item",
							"Item",
							"Item",
							"Item",
							"Item",

							"Item",
							"Item",
							"Item",
							"Item",
							"Item",
							"Item",
							"Item",
							"Item",
							"Item",
							"Item",

							"Item",
							"Item",
							"Item",
							"Item",
							"Item",
							"Item",
							"Item",
							"Item",
							"Item",
							"Item",

							"Item",
							"Item",
							"Item",
							"Item",
							"Item",
							"Item",
							"Item",
							"Item",
							"Item",
							"Item",

							"Item",
							"Item",
							"Item",
							"Item",
							"Item",
							"Item",
							"Item",
							"Item",
							"Item",
							"Item",

							"Item",
							"Item",
							"Item",
							"Item",
							"Item",
							"Item",
							"Item",
							"Item",
							"Item",
							"Item",

							"Item",
							"Item",
							"Item",
							"Item",
							"Item",
							"Item",
							"Item",
							"Item",
							"Item",
							"Item",

							"Item",
							"Item",
							"Item",
							"Item",
							"Item",
							"Item",
							"Item",
							"Item",
							"Item",
							"Item"

						};

int itemsel;
                        // Controller shield battery monitor variables
int analogVCCinput = 5; // RSeries Controller default VCC input is A5
float R1 = 47000.0;     // >> resistance of R1 in ohms << the more accurate these values are
float R2 = 24000.0;     // >> resistance of R2 in ohms << the more accurate the measurement will be
float vout = 0.0;       // for voltage out measured analog input
int VCCvalue = 0;       // used to hold the analog value coming out of the voltage divider
float vin = 0.0;        // voltage calulcated... since the divider allows for 15 volts

float vinSTRONG=3.4;    // If vin is above vinSTRONG display GREEN battery
float vinWEAK=2.9;		// if vin is above vinWEAK display YELLOW otherwise display RED
float vinDANGER=2.7;    // If 3.7v LiPo falls below this your in real danger.

						// Touch Screen Pin Configuration - Need to change A2 & A3, so as not to share
#define YM 9			// Y- (Minus) digital pin UNO = D9, MEGA = 9   // Orig 9
#define XM A8			// X- (Minus) must be an analog pin, use "An" notation! // Orig A2
#define YP A9			// Y+ (Plus)  must be an analog pin, use "An" notation! // Orig A3
#define XP 8			// X+ (Plus)  digital pin UNO = 8, MEGA = 8   // Orig 9

						// These can be adjusted for greater precision 
#define TS_MINX 131		// Orig = 150
#define TS_MINY 120		// Orig = 120
#define TS_MAXX 920		// Orig = 920
#define TS_MAXY 950		// Orig = 940

#define rotation 3		// Which only changes the orientation of the LCD not the touch screen

uint16_t touchedY;
uint16_t touchedX;

// For better pressure precision, we need to know the resistance
// between X+ and X- Use any multimeter to read it
// For the one we're using, its 300 ohms across the X plate
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 500);			// Orig (XP, YP, XM, YM, 300)

#define LCD_CS A3
#define LCD_CD A2
#define LCD_WR A1
#define LCD_RD A0 
                                  
#define LCD_RESET A4		// Use A4 to reset the Pin 7 of the TFT Display - Not Optional

// Color definitions
#define	BLACK           0x0000
#define	BLUE            0x001F
#define	RED             0xF800
#define	GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0 
#define WHITE           0xFFFF
#define WINE            0x8888
#define DRKBLUE         0x1111
#define TAN             0xCCCC
#define GRAY            0x038F

#define optionCOLOR RED

Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);

/*////////////////////////////////////////////////////////////////////////////////////////////////*/
///////////////////////* XBee Configuration *///////////////////////////////////////////////////////
/*////////////////////////////////////////////////////////////////////////////////////////////////*/

int xbeebps = 19200;
uint8_t payload[] = { '0', '0', '0', '0', '0', '0', '0', '0', '0'}; // Our XBee Payload of 9 potential values
Rx16Response rx16 = Rx16Response();
XBee xbee = XBee();
XBeeResponse response = XBeeResponse();

// Create reusable response objects for responses we expect to handle 
ZBRxResponse rx = ZBRxResponse();
ModemStatusResponse msr = ModemStatusResponse();
 
XBeeAddress64 addr64 = XBeeAddress64(0x0013a200, 0x40a8e65b);
ZBTxRequest zbTx = ZBTxRequest(addr64, payload, sizeof(payload));
ZBTxStatusResponse txStatus = ZBTxStatusResponse();

uint8_t shCmd[] = {'S','H'};
uint8_t slCmd[] = {'S','L'};
uint8_t opCmd[] = {'O','P'};

AtCommandRequest atRequest = AtCommandRequest(opCmd);
AtCommandResponse atResponse = AtCommandResponse();

boolean presstocontinue=false;
boolean controllerstatus=false;	
boolean transmitterstatus=false;	
boolean networkstatus=false;	
boolean receiverstatus=false;	
boolean telemetrystatus=false;	
boolean rxpacketvalid=false;
boolean rxpacketstart=false;
boolean txbegin=false;

int t = 0;
long rx1DBm;
long lastrx1time;
long lasttx1time;
long nexttx1time;
int rx1ErrorCount = 0;				// If >5 RX packets in a row are invalid, change status from OK to RX in YELLOW
int rx1ErrorCountMAX = 8;			// if >8 & receive errors, change status from OK to RX in RED: 8 packets ~1 Sec

boolean rxDEBUG = false;			// Set to monitor invalid TX packets via Serial Monitor baud 115200
boolean txDEBUG = false;			// Set to monitor sent TX packets via Serial Monitor baud 115200

/*////////////////////////////////////////////////////////////////////////////////////////////////*/
///////////////////////* Wii Nunchuk Configuration *////////////////////////////////////////////////
/*////////////////////////////////////////////////////////////////////////////////////////////////*/

ArduinoNunchuk nunchuk = ArduinoNunchuk();

byte joyx, joyy, accx, accy, accz, zbut, cbut;
byte triggeritem;

/*////////////////////////////////////////////////////////////////////////////////////////////////*/
///////////////////////* Arduino Functions *////////////////////////////////////////////////////////
/*////////////////////////////////////////////////////////////////////////////////////////////////*/

void setup() {

	Serial.begin(9600);									// DEBUG CODE

	Serial1.begin(xbeebps);
	xbee.setSerial(Serial1);							// Setup Xbee to use Serial1
	xbee.begin(xbeebps);								// Setup Xbee to begin at 19200
	
	pinMode(analogVCCinput, INPUT);
	
	tft.reset();										// A4 must be connected to TFT Break out Pin 7

	// Prep debugger...
	Serial.println(" ");
	Serial.println(" ");
	
	uint16_t identifier = tft.readRegister(0x0);
	if (identifier == 0x9325) {
		Serial.println("Found ILI9325");
	} else if (identifier == 0x9328) {
		Serial.println("Found ILI9328");
	} else if (identifier == 0x7575) {
		Serial.println("Found HX8347G");
	} else {
		Serial.print("Unknown driver chip ");
		Serial.println(identifier, HEX);
		while (1);
	}  
	
	tft.begin(identifier);
	
	tft.setRotation(rotation); 
	tft.fillScreen(BLACK);
	
	nunchuk.init();										// Initialize Nunchuk using I2C
	buttonval = 0;
	
	//displaySPLASH();									// Display Splash Screen...
	//delay(1500);										// ...for 1500 ms
	tft.fillScreen(BLACK);								// Clear screen.
	
	displayPOST();										// Display POST Routine on TFT
	
	// Serial.println("POST Finished Successfully");	// DEBUG CODE
	
	radiostatus = "OK";
	//  dmmVCC=rxVCC/10.0;								// Convert telemetry into floats
	//  dmmVCA=rxVCA/10.0;								// Convert telemetry into floats
	
	tft.fillScreen(BLACK);
	
	//  Serial.println("Display Status with BLUE");		// DEBUG CODE
	displaySTATUS(BLUE);								// Displays status bar at TOP
	displaySCROLL();									// Displays scroll buttons
	displayOPTIONS();									// Display trigger Options
	t=0;
	//  pinMode(RSSIpin, INPUT);						// Setup RSSIpin as Digital Input for 
	//  nextrx1time = millis();							// 
	
}

void loop() {

	RXdata();
		
	nunchuk.update();											// ALL data from nunchuk is continually sent to Receiver
	joyx = map(nunchuk.analogX, chan1Min, chan1Max, 60, 120);	// Channel 1 joyx & Channel 2 joyy from NunChuck Joystick
	joyy = map(nunchuk.analogY, chan2Min, chan2Max, 120, 60);	// Map it to Min & Max of each channel
	accx = nunchuk.accelX/2/2;									// ranges from approx 70 - 182
	accy = nunchuk.accelZ/2/2;									// ranges from approx 65 - 173
	accz = nunchuk.accelY/2/2;									// ranges from approx 65 - 173
	zbut = nunchuk.zButton;										// either 0 or 1
	cbut = nunchuk.cButton;										// either 0 or 1
																// Switched Z & Y on purpose. Now: X = left/right, Y = up/down, Z = forward/back.
	
	/*
	Serial.print("joyx: "); Serial.print((byte)joyx,DEC);		// DEBUG CODE
	Serial.print("\tjoyy: "); Serial.print((byte)joyy,DEC);		// DEBUG CODE
	Serial.print("\taccx: "); Serial.print((byte)accx,DEC);		// DEBUG CODE
	Serial.print("\taccy: "); Serial.print((byte)accy,DEC);		// DEBUG CODE
	Serial.print("\taccz: "); Serial.print((byte)accz,DEC);		// DEBUG CODE
	Serial.print("\tzbut: "); Serial.print((byte)zbut,DEC);		// DEBUG CODE
	Serial.print("\tcbut: "); Serial.println((byte)cbut,DEC);	// DEBUG CODE
	*/ 
	
	// Normalize, emulate "dead stick" zones...
	
	if (joyx < 82) { joyx = joyx; }
	else if (joyx > 98) { joyx = joyx; }
	else { joyx = 90; }

	if (joyy < 82) { joyy = joyy; }
	else if (joyy > 98) { joyy = joyy; }
	else { joyy = 90; }
	
	if (accx < 90) { accx = 60; }
	else if (accx > 160) { accx = 120; }
	else { accx = 90; }

	if (zbut == 1 && cbut == 0) { triggeritem = 101; TXdata(); }
	else if (zbut == 0 && cbut == 1) {triggeritem = 102; TXdata(); }
	else if (zbut == 1 && cbut == 1) {triggeritem = 103; TXdata(); }
	
	getTouch();
	
	TXdata();
	delay(100);
	displaySendclear();
	
	if (millis() >= nextStatusBarUpdate) {
		updateSTATUSbar(BLUE);
	}

}

void getTouch() {                     //
  Point p = ts.getPoint();            // Check for TFT Input
                                      // NOTE: TouchScreen X&Y (X Height from Bottom Left, Y= 320 Width from Left to Right) 
                                      // aligns with LCD X 320 Width (Left to Right) & Y Height Top to Bottom (240 Height)
                                      // so we are going to realign during the map
                                    
//  pinMode(XM, OUTPUT);                // switch back to OUTPUT for XM Pins - Moving to A8 & A9, so we can remove this
//  pinMode(YP, OUTPUT);                // switch back to OUTPUT for YP Pins

                                      // we have some minimum pressure we consider 'valid'
                                      // pressure of 0 means no pressing!

  if (p.z > ts.pressureThreshhold) {  // OK Someone touched the screen
                                      // Now lets map the touch point

  Serial.print("RAW X = "); Serial.print(p.x);          // DEBUG CODE
  Serial.print("\tY = "); Serial.print(p.y);            // DEBUG CODE
  Serial.print("\tPressure = "); Serial.print(p.z);     // DEBUG CODE
//  delay (10);
                                                        // We also have to re-align LCD X&Y to Touch X&Y
  touchedY = map(p.x, TS_MINX, TS_MAXX, 240, 0);        // Notice mapping touchedY to p.x
  touchedX = map(p.y, TS_MAXY, TS_MINY, 320, 0);        // Notice mapping touchedX to p.y
                                                        // and turn from 0->1023 to X & Y screen coordinates

  
  Serial.print("\tTouched X = "); Serial.print(touchedX);            // DEBUG CODE
  Serial.print("\tTouched Y = "); Serial.print(touchedY);            // DEBUG CODE
//  Serial.println(" ");                                               // DEBUG CODE
  

                                                         // Did they select a Menu Item?
    if (touchedX >= 75 && touchedY>= 40) {               // They touched inside the Menu Item area, need to decode
    
//    Serial.println("Touched Menu Item Area");            // DEBUG CODE
    
      if (touchedY >= 40 && touchedY < 70){              // Item 1
        itemsel=1;
        sendtrigger();
      }
      else if (touchedY >= 70 && touchedY < 95){              // Item 2
        itemsel=2;
        sendtrigger();
      }
      else if (touchedY >= 95 && touchedY < 125){             // Item 3
        itemsel=3;
        sendtrigger();
      }
      else if (touchedY >= 125 && touchedY < 150){           // Item 4
        itemsel=4;
        sendtrigger();
      }
      else if (touchedY >= 150 && touchedY < 185){           // Item 5
        itemsel=5;
        sendtrigger();  
      }
      else if (touchedY >= 185 && touchedY < 210){           // Item 6
        itemsel=6;
        sendtrigger();
      }
      else if (touchedY >= 210 && touchedY <= 240){         // Item 7
        itemsel=7;
        sendtrigger();
      }
    }
 
    
    if (touchedY >= 40 && touchedX < 60) {       // Scroll Button Area Touched
                                                   
      if (touchedY >=40 && touchedY <=105) {     // Up Arrow Pressed?
      
        displaygroup = displaygroup - 1;         // reduce displaygroup by 1

//        Serial.print("Display Group = ");Serial.println(displaygroup);// DEBUG CODE
//        Serial.println("Updating Display Option");            // DEBUG CODE

        if (displaygroup < 1) {                 // unless displaygroup now <=0
          displaygroup = 12;                     // in which case setup everything to wrap to group 12
          displayitem = 78;                      // display group 12 & start with item 78
          scrollyloc = scrollymax;               // set scroll indicator to home top location
        } 
       else if (displaygroup >=1) {                   // So we are good to 
          displayitem = displaygroup * 7 - 6;    // Since we can only display 7 items with TextSize=3
          scrollyloc = scrollyloc - scrollyinc;  // move scroll indicator loc by subtracting increment from location
        }
        
       
       displayOPTIONS();                         // Refresh OPTIONS Screen
      }                                          // Finshed with Up Arrow
      
      if (touchedY >=180 && touchedX < 60) {    // Down Arrow Pressed?
        displaygroup = displaygroup + 1;         // increase displaygroup
        if (displaygroup > menuScreens) {                // unless displaygroup now >=12
          displaygroup = 1;                     // in which case setup everything to
          displayitem = 1;                      // display group 11 & start with item 78
          scrollyloc = scrollymin;               // set scroll indicator to bottom scroll location
        }
        else if (displaygroup < menuScreens) {                  // So we are good to 
          displayitem = displaygroup * 7 - 6;    // Since we can only display 7 items with TextSize=3
          scrollyloc = scrollyloc + scrollyinc;  // move scroll indicator loc by adding increment to location
        }
        displayOPTIONS();                        // Refresh OPTIONS Screen
      }                                          // Finished with Down Arrow
    }                                            // Finished with Scroll Area
  }                                              // Finished with Touch Pressure
}

void displaySCROLL() {                // Display Scroll Buttons
  tft.drawTriangle(30, 40,            // Up Scroll Button
                    0, 100,
                   60,100, BLUE); 
  tft.setCursor(20, 80);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("UP");

  tft.drawTriangle(0, 175,              // Down Scroll Button
                  60, 175,
                  30, 235, BLUE); 

  tft.setCursor(20, 180);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("DN");

}

void displaySTATUS(uint16_t color) {          // This builds the status line at top of display when in normal operation on the controller
//  tft.setRotation(rotation); 
//  tft.fillScreen(BLACK);
  tft.setCursor(0, 0);
  tft.setTextColor(color);
  tft.setTextSize(2);
  tft.println(astromechName);
  displayBATT();
  displayRSSI();
  tft.setCursor(128, 0);
  if (radiostatus == "OK") {
    tft.setTextColor(GREEN);
  } else {
    tft.setTextColor(RED);
  }
  tft.println(radiostatus);
  
  
 if (ProcessStateIndicator == 0) {  // Display a ball heart beat in between Radio Status & dmmVCC
    tft.fillCircle(160,7, 5, BLUE);          
    ProcessStateIndicator = 1;
  } else {
    tft.fillCircle(160,7, 5, RED);
    ProcessStateIndicator = 0;
  }
  
  tft.setCursor(170, 0);
  
  tft.setTextColor(GREEN);    // Default dmmVCC text color
  if (dmmVCC <= yellowVCC){    // If dmmVCC drops BELOW, change text to YELLOW or RED
    tft.setTextColor(YELLOW);}
  if (dmmVCC <= redVCC){
   tft.setTextColor(RED);}
   
  tft.setTextSize(2);
  tft.println(dmmVCC,2);      // Display dmmVCC FLOAT 2 Places
  tft.setCursor(230, 0);
  tft.println("V");
  
  tft.setTextColor(GREEN);    // Default dmmVCA text color
  if (dmmVCA >= yellowVCA){    // If dmmVCA goes ABOVE, change text to YELLOW or RED
    tft.setTextColor(YELLOW);}
  if (dmmVCA >= redVCA){
    tft.setTextColor(RED);}
   
  tft.setCursor(250, 0);
  tft.println(dmmVCC,2);      // Display dmmVCA FLOAT 2 Places
  tft.setCursor(310, 0);
  tft.println("A");
   
  tft.drawFastHLine(0, 19, tft.width(), BLUE);
  
  lastStatusBarUpdate = millis();        // Could remove this one now..
  nextStatusBarUpdate = millis() + updateSTATUSdelay; // Update Status Bar on Display every X + millis();
  
}

void updateSTATUSbar(uint16_t color) {

  tft.setTextSize(2);
  displayBATT();
  displayRSSI();
  tft.setCursor(128, 0);
  
  if (rx1ErrorCount <= 5 && radiostatus == "OK") {   // rx1ErrorCount & radiostatus error conditions GREEN OK
    tft.setTextColor(GREEN);
  } else if (rx1ErrorCount >5 && radiostatus == "OK") {  // Caution YELLOW OK
    tft.setTextColor(YELLOW);
  } else if (rx1ErrorCount >5 && radiostatus != "OK") {  // DANGER RED ERROR CONDITION
    tft.setTextColor(RED);
  }
    
  
  tft.println(radiostatus);          // Display the OK or RX or TX radio controll status.
 
  
  if (ProcessStateIndicator == 0) {  // Display a ball heart beat in between Radio Status & dmmVCC
    tft.fillCircle(160,7, 5, BLUE);          
    ProcessStateIndicator = 1;
  } else {
    tft.fillCircle(160,7, 5, RED);
    ProcessStateIndicator = 0;
  }
  
  tft.setCursor(170, 0);
  
  tft.fillRect(170, 0, 59, 17, BLACK);
  
  tft.setTextColor(GREEN);    // Default dmmVCC text color
  if (dmmVCC <= yellowVCC){    // If dmmVCC drops BELOW, change text to YELLOW or RED
    tft.setTextColor(YELLOW);}
  if (dmmVCC <= redVCC){
   tft.setTextColor(RED);}
     
  tft.setTextSize(2);
  tft.println(dmmVCC,2);      // Display dmmVCA FLOAT 2 Places
  tft.setCursor(230, 0);
  tft.println("V");
  
  tft.setTextColor(GREEN);    // Default dmmVCA text color
  if (dmmVCA >= yellowVCA){    // If dmmVCA goes ABOVE, change text to YELLOW or RED
    tft.setTextColor(YELLOW);}
  if (dmmVCA >= redVCA){
    tft.setTextColor(RED);}
  
  tft.fillRect(250, 0, 59, 17, BLACK);
 
  tft.setCursor(250, 0);
  tft.println(dmmVCA,2);      // Display dmmVCA FLOAT 2 Places
  tft.setCursor(310, 0);
  tft.println("A");
   
//  tft.drawFastHLine(0, 19, tft.width(), BLUE);  // we should be able remove this from here
  
//  lastStatusBarUpdate = millis();        // can remove this one now..
  nextStatusBarUpdate = millis() + updateSTATUSdelay; // Update Status Bar on Display every X + millis();
  
}

void displayRSSI() {                        // Display Received packet Signal Strength Indicator

//  RSSIduration = pulseIn(RSSIpin, LOW, 200);  // with 200us timeout
//  RSSIduration = pulseIn(6, HIGH);  // with 200us timeout

RSSIduration = rx16.getRssi();

//  Serial.print(millis());                                              // DEBUG CODE
//  Serial.print("RSSIpin = ");Serial.println(RSSIpin, HEX);             // DEBUG CODE
//  Serial.print("\tRSSIduration = ");Serial.println(RSSIduration);        // DEBUG CODE

// rx1DBm = (41 * RSSIduration) - 5928;
 
 rx1DBm = RSSIduration;
 
// Serial.print("rx1DBm = ");Serial.println(rx1DBm);             // DEBUG CODE
 
//delay(100);
//RSSI (in dBm) is converted to PWM counts using the following equation: PWM counts = (41 * RSSI_Unsigned) - 5928
                                                         // Signal Bars, Well its good enough for Apple.
  if (rx1DBm == 0) {                                     // No Signal Text
    tft.fillRect(100, 0, 22, 17, BLACK);                 // Erase Signal Bar area
    tft.setTextSize(1);
    tft.setTextColor(WHITE);
    tft.setCursor(100, 0);
    tft.println("No");
    tft.setCursor(100, 9);
    tft.println("Sgnl");
    tft.setTextSize(2);
    } else if (rx1DBm >=1 && rx1DBm <=20) {RSSI=1;}                 // Signal Bars, Well its good enough for Apple.
      else if (rx1DBm >=21 && rx1DBm<=50) {RSSI=2;}
      else if (rx1DBm >=51 && rx1DBm<=100) {RSSI=3;}
      else if (rx1DBm >=101 && rx1DBm<=175) {RSSI=4;}
      else if (rx1DBm >=176 && rx1DBm<=255) {RSSI=5;}
     
  tft.fillRect(100, 0, 22, 17, BLACK);                 // Erase Signal Bar area

  tft.drawFastVLine(100, 12, 2, GRAY);  // Signal =1 
  tft.drawFastVLine(101, 12, 2, GRAY);
  
  tft.drawFastVLine(105, 10, 4, GRAY);  // Signal =2
  tft.drawFastVLine(106, 10, 4, GRAY);
  
  tft.drawFastVLine(110, 8, 6, GRAY);  // Signal =3 
  tft.drawFastVLine(111, 8, 6, GRAY);
  
  tft.drawFastVLine(115, 5, 9, GRAY);  // Signal =4 
  tft.drawFastVLine(116, 5, 9, GRAY);
  
  tft.drawFastVLine(120, 1, 13, GRAY);  // Signal =5 
  tft.drawFastVLine(121, 1, 13, GRAY);
  
  if (RSSI>=1) {
    tft.drawFastVLine(100, 12, 2, WHITE);  // Signal =1 
    tft.drawFastVLine(101, 12, 2, WHITE);
  }  
  if (RSSI>=2) {
    tft.drawFastVLine(105, 10, 4, WHITE);  // Signal =2
    tft.drawFastVLine(106, 10, 4, WHITE);
  }
  if (RSSI>=3) {
    tft.drawFastVLine(110, 8, 6, WHITE);  // Signal =3 
    tft.drawFastVLine(111, 8, 6, WHITE);
  }
  if (RSSI>=4) {
    tft.drawFastVLine(115, 5, 9, WHITE);  // Signal =4 
    tft.drawFastVLine(116, 5, 9, WHITE);
  }
  if (RSSI>=5) {
    tft.drawFastVLine(120, 1, 13, WHITE);  // Signal =5 
    tft.drawFastVLine(121, 1, 13, WHITE);
  }
  }

/*
tft.fillCircle(10,10, 1, BLUE);             // Signal =1 
tft.fillCircle(15,10, 2, BLUE);             // Signal =2
tft.fillCircle(22,10, 3, BLUE);             // Signal =3
tft.fillCircle(31,10, 4, BLUE);             // Signal =4
tft.fillCircle(42,10, 5, GRAY);             // Signal =5
*/

void displayOPTIONS() {                  // Display Event Trigger Options 

  tft.fillRect(0, 21, 310, 17, BLACK);   // Clear Message Area and leave scroll indicator

  tft.fillRect(0, 127, 68, 17, BLACK);   // Clear Center Pg X

  tft.setTextColor(YELLOW);
  tft.setTextSize(2);
  tft.setCursor(5, 128);
  tft.println("Pg");
  tft.setCursor(35, 128);
    
  tft.println(displaygroup);
 
  tft.fillRect(69, 38, 241, 202, BLACK); // Clear options, leave scroll bar

  int xoffset = 70;                      // Display each 7 Item Names
  int yoffset=40;
  tft.setTextColor(optionCOLOR);
  tft.setTextSize(3);                    // More usable than 2
  int yTextSizeoffset= 29;               // 20 if TextSize =2, 29 if TextSize=3  

  tft.setCursor(xoffset, yoffset);
  tft.println(menuItem[displayitem]);    // Item 1
  
  tft.setCursor(xoffset, yoffset+yTextSizeoffset);
  tft.println(menuItem[displayitem+1]);  // Item 2
  
  tft.setCursor(xoffset, yoffset+(yTextSizeoffset*2));
  tft.println(menuItem[displayitem+2]);  // Item 3
  
  tft.setCursor(xoffset, yoffset+(yTextSizeoffset*3));
  tft.println(menuItem[displayitem+3]);  // Item 4
  
  tft.setCursor(xoffset, yoffset+(yTextSizeoffset*4));
  tft.println(menuItem[displayitem+4]);  // Item 5
  
  tft.setCursor(xoffset, yoffset+(yTextSizeoffset*5));  
  tft.println(menuItem[displayitem+5]);  // Item 6
  
  tft.setCursor(xoffset, yoffset+(yTextSizeoffset*6));
  tft.println(menuItem[displayitem+6]);  // Item 7
  
  tft.fillRect(315, 23, 4, 216, BLACK);       // draw scroll indicator  
  tft.drawRect(314, 22, 6, 218, WHITE);       // draws outline of the scroll bar
  tft.fillRect(315, scrollyloc, 4, 15, BLUE); // draw scroll indicator
}

void displaySPLASH() {									// Display a Retro SPLASH Title Screen!
  tft.setCursor(40, 80);
  tft.setTextColor(WHITE);
  tft.setTextSize(3);
  tft.println("R2-D2");
  tft.setCursor(60,110);
  tft.println("Controller");
  tft.setCursor(130,140);
  tft.setTextSize(2);
  tft.println(VERSION);
  tft.setTextColor(BLUE);
  tft.setTextSize(2);
  tft.drawFastHLine(0,0, tft.width(), BLUE);
  tft.drawFastHLine(0,10, tft.width(), BLUE);
  tft.setCursor(20,170);
  tft.println("Skwerl's Fork");							// No vanity here... :-)
  tft.setCursor(20,190);
  tft.println("Royal Engineers of Naboo");				// Need to make it look like SW Canon
  tft.drawFastHLine(0, 229, tft.width(), BLUE); 
  tft.drawFastHLine(0, 239, tft.width(), BLUE);   
}

void sendtrigger() {                          // We need to see how long since we last sent a trigger event
                                              // Would like to keep to many events happening, so about 100ms
Serial.println("Arrived at sendtrigger");            // DEBUG CODE

  displayitem = displaygroup * 7 - 6;
  itemsel = itemsel - 1;
  triggeritem = displayitem + itemsel;
  if (triggeritem == 17) {triggeritem = 104;} // workaround to fix 17 & 19 trigger issue on XBees
    else if (triggeritem == 19) {triggeritem = 105;} // workaround to fix 17 & 19 trigger issue on XBees
  
  
  Serial.print("Event Triggered --------------------------> triggeritem = ");Serial.println(triggeritem);             // DEBUG CODE
  
  tft.setCursor(80, 22);
  tft.setTextColor(GREEN);
  tft.setTextSize(2);
  tft.println("Sending:");
  tft.setCursor(180, 22);
  tft.println(int(triggeritem)); // using INT to make the byte readable on the display as a number vs a single byte
  
  Serial.print("SENDING triggeritem = ");Serial.println(triggeritem);             // DEBUG CODE
 
}

void displaySendclear() {
	tft.fillRect(80, 21, 220, 17, BLACK);		// Clear Trigger Message
	triggeritem=0;								// Zero out selection variables
	zbut=0;
	cbut=0;
	itemsel = 0;
}

void displayPOST() {									// Power Up Self Test and Init

	presstocontinue=true;								// User must press something to continue?
	controllerstatus=false;								// Nunchuk connected?
	transmitterstatus=false;							// Are we transmitting?
	networkstatus=false;								// Are we networked?
	receiverstatus=false;								// Are we receiving?
	telemetrystatus=false;								// Are we receiving telemetry?
	
	tft.setCursor(0, 0);
	tft.setTextColor(GREEN);
	tft.setTextSize(2);
	tft.println("Testing...");
	tft.setCursor(20,20);
	tft.println("Wii Nunchuk:");
	tft.setCursor(220,20);
	tft.setTextColor(RED);
	tft.println("...");
	
	while(controllerstatus == false) {
		nunchuk.update();								// ALL data from nunchuk is continually sent to Receiver
		joyx = nunchuk.analogX;							// ranges from approx 30 - 220
		joyy = nunchuk.analogY;							// ranges from approx 29 - 230
		accx = nunchuk.accelX;							// ranges from approx 70 - 182
		accy = nunchuk.accelY;							// ranges from approx 65 - 173
		accz = nunchuk.accelZ;							// ranges from approx 65 - 173
		zbut = nunchuk.zButton;							// either 0 or 1
		cbut = nunchuk.cButton;							// either 0 or 1
		if (joyx > 0 && joyy > 0) {
			controllerstatus = true;
		}
	}
	
	tft.fillRect(220,20,100,17,BLACK);					// Clear Waiting Message
	tft.setCursor(220,20);
	tft.setTextColor(GREEN);
	tft.println("OK!");
	
	tft.setTextColor(GREEN);
	tft.setCursor(20,40);
	tft.println("Transmitter:");
	tft.setCursor(220,40);
	tft.setTextColor(RED);
	tft.println("...");
	
	while(transmitterstatus == false) {
		xbeeSL();										// Send an AT command via API mode
		Serial.print("xbATResponse = ");				// DEBUG CODE
		Serial.println(xbATResponse, HEX);				// DEBUG CODE
		if (xbATResponse == 0x00) {						// to verify Coordinator XBee is setup and ready.
			transmitterstatus = true;
			Serial.println("Transmitter Status Good");	// DEBUG CODE
		}
	}
	
	tft.fillRect(220, 40, 100, 17, BLACK);
	tft.setCursor(220,40);
	tft.setTextColor(GREEN);
	tft.println("OK!");  

	tft.setCursor(20,60);
	tft.setTextColor(GREEN);
	tft.println("Receiver:");
	tft.setCursor(220,60);
	tft.setTextColor(RED);
	tft.println("Pinging");

	while(receiverstatus == false) {
		payload[0] = byte(0x90);						// joyx
		payload[1] = byte(0x90);						// joyy
		payload[2] = byte(0x90);						// accx
		payload[3] = byte(0x90);						// accy
		payload[4] = byte(0x90);						// accz
		payload[5] = byte(0x00);						// zButton
		payload[6] = byte(0x00);						// cButton
		payload[7] = byte(0x00);						// TriggerEvent
		payload[8] = byte(0x00);						// Future Use
		xbee.send(zbTx);								// Send a Test Payload to Receiver
		Serial.println("Sending test payload...");		// DEBUG CODE
		if (xbee.readPacket(500)) {
			if (xbee.getResponse().getApiId() == ZB_TX_STATUS_RESPONSE) {
				xbee.getResponse().getZBTxStatusResponse(txStatus);
				if (txStatus.getDeliveryStatus() == SUCCESS) {
					receiverstatus = true;
				} else {
					// Packet not received...
				}
			}
		} else if (xbee.getResponse().isError()) {
			// Error reading packet...
		} else {
			// TX status timeout...
		}
		delay(1000);
	}
	Serial.println("Receiver Responded, Status Good");   // DEBUG CODE
	tft.fillRect(220, 60, 100, 17, BLACK);
	tft.setCursor(220,60);
	tft.setTextColor(GREEN);
	tft.println("OK!");  

	// Looking for the Volts & Amperage from Receiver (Router)
	tft.setTextColor(GREEN);
	tft.setCursor(20,80);
	tft.println("Telemetry:");
	tft.setCursor(220,80);
	tft.setTextColor(RED);
	tft.println("...");
	
	while(telemetrystatus == false) {
		RXdata();
		Serial.print("POST Received Telemetry -->> rxVCC =");
		Serial.print(rxVCC);  // DEBUG CODE
		Serial.print("\trxVCA =");Serial.println(rxVCA);                                  // DEBUG CODE
		if (rxVCC > 0 && rxVCA > 0){
			telemetrystatus = true;
		}
		telemetrystatus = true;							// Screw it...
	}
	
	tft.fillRect(220, 80, 100, 17, BLACK);// Clear Waiting Message
	tft.setCursor(220,80);
	tft.setTextColor(BLUE);
	tft.println("Skip");  
	
	delay(50); // Wait 50msec second before continueing

}

void RXdata() {
 
  xbee.readPacket(50); // wait 50msec to see if we received a packet
  
  if (xbee.getResponse().isAvailable()) {
  if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) {
    xbee.getResponse().getZBRxResponse(rx);


/*    To receive the 2 byte values of Telemetry.
long -> byte[4]

buf[0] = (byte) n;
buf[1] = (byte) n >> 8;
buf[2] = (byte) n >> 16;
buf[3] = (byte) n >> 24;
//
// byte[4] -> long
//long value = (unsigned long)(buf[4] << 24) | (buf[3] << 16) | (buf[2] << 8) | buf[1];
*/
//int telemetryvCC = (int)(rx.getData()[0]; << 8) | rx.getData[1];
//int telemetryvCA = (int)(rx.getData()[2]; << 8) | rx.getData[3];

    telemetryVCCMSB = rx.getData()[0];
    telemetryVCCLSB = rx.getData()[1];
    Serial.print("\ttelemetryVCCMSB =");Serial.print(telemetryVCCMSB);  // DEBUG CODE
    Serial.print("\ttelemetryVCCLSB =");Serial.print(telemetryVCCLSB);                      // DEBUG CODE
  
    
    telemetryVCAMSB = rx.getData()[2];
    telemetryVCALSB = rx.getData()[3];
    
    Serial.print("\t\ttelemetryVCAMSB =");Serial.print(telemetryVCAMSB);                        // DEBUG CODE
    Serial.print("\ttelemetryVCALSB =");Serial.println(telemetryVCALSB);                      // DEBUG CODE



    rxVCC = (unsigned int)word(telemetryVCCMSB,telemetryVCCLSB);
    
    rxVCA = (unsigned int)word(telemetryVCAMSB,telemetryVCALSB);

//    Serial.print("\trxVCC =");Serial.print(rxVCC);                        // DEBUG CODE                                          // DEBUG CODE
//    Serial.print("\trxVCA =");Serial.println(rxVCA);                      // DEBUG CODE


    
//    Serial.print("\ttelemetryVCAMSB =");                                                      // DEBUG CODE
//    Serial.print(telemetryVCAMSB);                                                            // DEBUG CODE
//    Serial.print("\ttelemetryVCALSB =");Serial.println(telemetryVCALSB);                      // DEBUG CODE



      dmmVCC = (float)rxVCC / 10.0;                   // Float & move the point for Volts
      dmmVCA = (float)rxVCA / 10.0;                   // float & move the point for Amps
                  
    Serial.print("Received Telemetry -->> rxVCC =");Serial.print(rxVCC);  // DEBUG CODE
    Serial.print("\trxVCA =");Serial.println(rxVCA);                      // DEBUG CODE

    }
  }
 }

void TXdata() {

	payload[0]=joyx;			// 17 JoyX ranges from approx 30 - 220
	payload[1]=joyy;			// 18 JoyY ranges from approx 29 - 230
	payload[2]=accx;			// 19 AccX ranges from approx 70 - 182
	payload[3]=accy;			// 20 AccY ranges from approx 65 - 173
	payload[4]=accz;			// 21 AccZ ranges from approx 65 - 173
	payload[5]=zbut;			// 22 ZButton Status
	payload[6]=cbut;  			// 23 CButton Status
	payload[7]=triggeritem;		// 24 0 to 254  If you have more than 254 events... need to rework event code
	payload[8]=0x00;			// 25 - Future USE

	xbee.send(zbTx);

}  

void xbeeSL() {

	// Serial.println("Running xbeeSL()...");

	// Get Serial number Low (SL) of the Transmitter
	// See Chapter 9, API Operation (~Page 99) of Digi 
	// XBee/Xbee-PRO ZB RF Modules Manual for details
	
	atRequest.setCommand(slCmd);  
	xbee.send(atRequest);

	if (xbee.readPacket(5000)) {						// Wait up to 5 seconds for the status response...
		// Got a response!

		xbee.getResponse().getAtCommandResponse(atResponse);			
		if (atResponse.isOk()) {
			Serial.print("Command [");
			Serial.print(atResponse.getCommand()[0]);
			Serial.print(atResponse.getCommand()[1]);
			Serial.println("] was successful!");
			xbATResponse = 0x00;
			if (atResponse.getValueLength() > 0) {
				Serial.print("Command value length is ");
				Serial.println(atResponse.getValueLength(), DEC);
				Serial.print("Command value: ");
				for (int i = 0; i < atResponse.getValueLength(); i++) {
					Serial.print(atResponse.getValue()[i], HEX);
					Serial.print(" ");
				}
				Serial.println("");
			}
		} else {
			Serial.print("Command return error code: ");
			Serial.println(atResponse.getStatus(), HEX);
		}

	} else {
		if (xbee.getResponse().isError()) {
			Serial.print("Error reading packet.  Error code: ");  
			Serial.println(xbee.getResponse().getErrorCode());
		} else {
			Serial.println("No response from radio!");  
		}
	}

}

void getVCC() {
//    vin = random(80,100)/10.0;             // DEBUG CODE
   VCCvalue = analogRead(analogVCCinput); // this must be between 0.0 and 5.0 - otherwise you'll let the blue smoke out of your ar
   vout= (VCCvalue * 5.0)/1024.0;         //voltage coming out of the voltage divider
   vin = vout / (R2/(R1+R2));             //voltage based on vout to display battery status
// Serial.print("Battery Voltage =");Serial.println(vin,1);  // DEBUG CODE

}

void displayBATT() {                        			// Display Local Battery Status

	getVCC();
	
	tft.fillRect(84, 0, 9, 17, BLACK);       			// Erase Battery Status area
	
	// Draw the battery outline in white
	tft.drawFastHLine(86, 0, 4, WHITE);					// This is the little top of the battery
	tft.drawFastHLine(86, 1, 4, WHITE);
	tft.drawRect(84, 2, 8, 15, WHITE);					// Body of the battery
	
	if (vin >= vinSTRONG) {   
		tft.fillRect(85, 3, 6, 14, GREEN);				// If Battery is strong then GREEN  
	} else if (vin >=vinWEAK) {
		tft.fillRect(85, 8, 6, 9, YELLOW);				// If Battery is medium then YELLOW
	} else {
		tft.fillRect(85, 12, 6, 4, RED);				// If Battery is low then RED
	}    

}