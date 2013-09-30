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
///////////////////////* Libraries *////////////////////////////////////////////////////////////////
/*////////////////////////////////////////////////////////////////////////////////////////////////*/

#include <Arduino.h>
#include <Adafruit_GFX.h>       // Core graphics library
#include <Adafruit_TFTLCD.h>    // Hardware-specific library
#include <TFTLCD.h>             // TFT LCD functions - adafruit.com @ https://github.com/adafruit/TFTLCD-Library
#include <TouchScreen.h>        // Touch Screen functions - adafruit.com @ https://github.com/adafruit/Touch-Screen-Library
#include <XBee.h>               
#include <Wire.h>               // Used to read the I2C data from Nunchuck
#include <ArduinoNunchuk.h>     // Gabriel Bianconi @ http://www.gabrielbianconi.com/projects/arduinonunchuk/
#include <HashMap.h>			// http://playground.arduino.cc/Code/HashMap#Download

/*////////////////////////////////////////////////////////////////////////////////////////////////*/
///////////////////////* Global Config *////////////////////////////////////////////////////////////
/*////////////////////////////////////////////////////////////////////////////////////////////////*/

#define codeBranch "Skwerl's R-Series"
#define codeVersion "0.6"
#define astromechName "R2-D2"
#define astromechOwner "Kevin Cogill"

/*////////////////////////////////////////////////////////////////////////////////////////////////*/
///////////////////////* Trigger Configuration *////////////////////////////////////////////////////
/*////////////////////////////////////////////////////////////////////////////////////////////////*/

#define TOTAL_TRIGGERS 22
#define TOTAL_STICKY 6

//  Until TX payload has been reworked, we have a limit of 251 triggers.
//  Trigger names should be 13 characters or less.
//  |-------------|
char* gridTriggers[TOTAL_TRIGGERS] = {
	"Holos",
	"Autopilot",
	"Casual",
	"Happy",
	"Cautious",
	"Angry",	
	"Laugh",
	"Alarm 1",
	"Alarm 2",	
	"Cantina Song",
	"Doot Doot",
	"Failure",
	"Humming",
	"Leia Message",
	"Patrol",
	"Scream 1",
	"Scream 2",
	"Scream 3",
	"Scream 4",
	"Processing",
	"Short Circuit",
	"Startup Sound"
};

int stickyTriggers[TOTAL_STICKY] = {
	1,2,3,4,5,6
};

int boxWidth = 100;
int boxHeight = 52;
int boxMargin = 10;

int boxesPerRow = 3;
int rowsPerPage = 3;
int itemsPerPage = boxesPerRow*rowsPerPage;

int hashMapIndex = 0;
HashType<int,boolean> hashRawArray[TOTAL_TRIGGERS]; 
HashMap<int,boolean> stickyHash = HashMap<int,boolean>(hashRawArray, TOTAL_TRIGGERS); 

/*////////////////////////////////////////////////////////////////////////////////////////////////*/
///////////////////////* LCD Display Configuration *////////////////////////////////////////////////
/*////////////////////////////////////////////////////////////////////////////////////////////////*/

int countedPages = 0;
int curPage = 1;

int startAt;
int triggered;

// not yet sure how to get this to return the right number... 
//int totalTriggers = ((sizeof(gridTriggers)/sizeof(char))-1);

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
///////////////////////* Touch Configuration *//////////////////////////////////////////////////////
/*////////////////////////////////////////////////////////////////////////////////////////////////*/

// For better pressure precision, we need to know the resistance between X+ and X-
// Use any multimeter to read it. Sample baseline is 300 (ohms)
#define XR 200
int sensitivity = 30;	// Default is 10
						// But this is in relation to X resistance

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

int touchedRow;
int touchedCol;

TouchScreen ts = TouchScreen(XP, YP, XM, YM, XR);

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

uint8_t xbRSSI;						// Used to store Pulse Width from RSSIpin

int xbATResponse = 0xFF;			// To verify Coordinator XBee is setup and ready, set to 0xFF to prevent false positives

boolean rxDEBUG = false;			// Set to monitor invalid TX packets via Serial Monitor baud 115200
boolean txDEBUG = false;			// Set to monitor sent TX packets via Serial Monitor baud 115200

/*////////////////////////////////////////////////////////////////////////////////////////////////*/
///////////////////////* Wii Nunchuk Configuration *////////////////////////////////////////////////
/*////////////////////////////////////////////////////////////////////////////////////////////////*/

ArduinoNunchuk nunchuk = ArduinoNunchuk();

byte joyx, joyy, accx, accy, accz, zbut, cbut;

byte triggerEvent;

/*////////////////////////////////////////////////////////////////////////////////////////////////*/
///////////////////////* Telemetry Configuration *//////////////////////////////////////////////////
/*////////////////////////////////////////////////////////////////////////////////////////////////*/

byte txSignature = 0;
byte rxSignature = 0;
byte rxResponded = 0;
byte lastSigSent = txSignature;

int signalCode = 0;										// 0:  No Radio
														// 1:  Radio OK
														// 2:  Sending Signal
														// 3:  Signal Received
														// -1: Signal Timeout

int signalTimeout = 3000;								// How long do we wait for signal to be received?
unsigned long signalLastSent = 0;
unsigned long signalNextMilli = 0;
unsigned long lastTransmission = 0;

int updateStatusDelay = 3500;							// Update Status Bar Display every 3500ms (3.5 Seconds), caution on reducing
unsigned long nextStatusBarUpdate = 0;

int analogVCCinput = 5;									// RSeries Controller default VCC input is A5
float R1 = 47000.0;										// >> resistance of R1 in ohms << the more accurate these values are
float R2 = 24000.0;										// >> resistance of R2 in ohms << the more accurate the measurement will be

/* CONTROLLER BATTERY */

unsigned int txVCC = 0;
unsigned int txVCA = 0;
float txVCCout;											// Display variable for Voltage from Receiver
float txVCAout;											// Display variable for Amperage from Receiver

/* TELEMETRY FROM RECEIVER */

int rxRssi;
int rssiMin = 90;
int rssiMax = 40;

unsigned int rxVCC;
unsigned int rxVCA;
float rxVCCout;											// Display variable for Voltage from Receiver
float rxVCAout;											// Display variable for Amperage from Receiver

float vinSTRONG = 3.4;									// If vin is above vinSTRONG display GREEN battery
float vinWEAK = 2.9;									// if vin is above vinWEAK display YELLOW otherwise display RED
float vinDANGER = 2.7;									// If 3.7v LiPo falls below this you're in real danger.

/*
float yellowVCC = 12.0;									// If RX voltage drops BELOW these thresholds, text will turn yellow then red.
float redVCC = 11.5;

float yellowVCA = 50.0;									// If current goes ABOVE these thresholds, text will turn yellow then red.
float redVCA = 65.0;
*/

/*////////////////////////////////////////////////////////////////////////////////////////////////*/
///////////////////////* Arduino Functions *////////////////////////////////////////////////////////
/*////////////////////////////////////////////////////////////////////////////////////////////////*/

void setup() {

	Serial.begin(9600);

	Serial1.begin(xbeebps);
	xbee.setSerial(Serial1);							// Setup Xbee to use Serial1
	xbee.begin(xbeebps);								// Setup Xbee to begin at 19200
	
	pinMode(analogVCCinput, INPUT);

	countPages();									// Process trigger arrays

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
	
	//displaySplash();									// Display Splash Screen...
	//delay(1500);										// ...for 1500 ms
	//tft.fillScreen(BLACK);							// Then clear screen.
	
	bootTests();										// Display POST Routine on TFT

	tft.fillScreen(BLACK);	
	updateStatus();										// Displays status bar at top
	updateGrid();

	toggleStickyTrigger(3);								// Casual mood on by default

}

void loop() {

	getVCC();
	RXdata();

	nunchuk.update();											// ALL data from nunchuk is continually sent to Receiver
	joyx = nunchuk.analogX;										// 0-255, center is 128
	joyy = nunchuk.analogY;										// 0-255, center is 128
	accx = nunchuk.accelX/2/2;									// Ranges from approx 70-182
	accy = nunchuk.accelY/2/2;									// Ranges from approx 65-173
	accz = nunchuk.accelZ/2/2;									// Ranges from approx 65-173
	zbut = nunchuk.zButton;										// Either 0 or 1
	cbut = nunchuk.cButton;										// Either 0 or 1

	if (zbut == 1 && cbut == 0) { triggerEvent = 253; }
	else if (zbut == 0 && cbut == 1) {triggerEvent = 252; }
	else if (zbut == 1 && cbut == 1) {triggerEvent = 254; }
	
	getTouch();
	TXdata();
	updateSignal();
	checkPulse();

	if (millis() >= nextStatusBarUpdate) {
		updateStatus();
	}

}

/*////////////////////////////////////////////////////////////////////////////////////////////////*/
///////////////////////* RSeries Functions *////////////////////////////////////////////////////////
/*////////////////////////////////////////////////////////////////////////////////////////////////*/

void displaySplash() {									// Display a Retro SPLASH Title Screen!
  tft.setCursor(40, 80);
  tft.setTextColor(WHITE);
  tft.setTextSize(3);
  tft.println("R2-D2");
  tft.setCursor(60,110);
  tft.println("Controller");
  tft.setCursor(130,140);
  tft.setTextSize(2);
  tft.println(astromechName);
  tft.setTextColor(BLUE);
  tft.setTextSize(2);
  tft.drawFastHLine(0,0, tft.width(), BLUE);
  tft.drawFastHLine(0,10, tft.width(), BLUE);
  tft.setCursor(20,170);
  tft.println(codeBranch);
  tft.setCursor(20,190);
  tft.println("Royal Engineers of Naboo");				// Need to make it look like SW Canon
  tft.drawFastHLine(0, 229, tft.width(), BLUE); 
  tft.drawFastHLine(0, 239, tft.width(), BLUE);   
}

void bootTests() {

	tft.setCursor(0,0);
	tft.setTextColor(WHITE);
	tft.setTextSize(2);
	tft.print(codeBranch);
	tft.print(" ");
	tft.print(codeVersion);

	tft.setTextSize(1);
	tft.setCursor(0,20);
	tft.print("Astromech: ");
	tft.print(astromechName);
	tft.setCursor(0,30);
	tft.print("Builder: ");
	tft.print(astromechOwner);

	tft.setTextColor(GREEN);
	tft.setTextSize(2);
	
	/* BOOT TEST SEQUENCE */

	controllerstatus=false;								// Nunchuk connected?
	transmitterstatus=false;							// Are we transmitting?
	networkstatus=false;								// Are we networked?
	receiverstatus=false;								// Are we receiving?
	telemetrystatus=false;								// Are we receiving telemetry?

	tft.setCursor(20,60);
	tft.println("Wii Nunchuk:");
	tft.setCursor(220,60);
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
	
	tft.fillRect(220,60,100,17,BLACK);					// Clear Waiting Message
	tft.setCursor(220,60);
	tft.setTextColor(GREEN);
	tft.println("OK!");
	
	tft.setTextColor(GREEN);
	tft.setCursor(20,80);
	tft.println("Transmitter:");
	tft.setCursor(220,80);
	tft.setTextColor(RED);
	tft.println("...");
	
	while(transmitterstatus == false) {
		xbeeSL();										// Send an AT command via API mode
		Serial.print("xbATResponse = ");				// DEBUG CODE
		Serial.println(xbATResponse, HEX);				// DEBUG CODE
		if (xbATResponse == 0x00) {						// to verify Coordinator XBee is setup and ready.
			signalCode = 1;
			transmitterstatus = true;
			Serial.println("Transmitter Status Good");	// DEBUG CODE
		}
	}
	
	tft.fillRect(220, 80, 100, 17, BLACK);
	tft.setCursor(220,80);
	tft.setTextColor(GREEN);
	tft.println("OK!");  

	tft.setCursor(20,100);
	tft.setTextColor(GREEN);
	tft.println("Receiver:");
	tft.setCursor(220,100);
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
	Serial.println("Receiver Responded...");   // DEBUG CODE
	tft.fillRect(220, 100, 100, 17, BLACK);
	tft.setCursor(220,100);
	tft.setTextColor(GREEN);
	tft.println("OK!");  

	// Looking for the Volts & Amperage from Receiver (Router)
	tft.setTextColor(GREEN);
	tft.setCursor(20,120);
	tft.println("Telemetry:");
	tft.setCursor(220,120);
	tft.setTextColor(RED);
	tft.println("...");

	while(telemetrystatus == false) {
		RXdata();
		//Serial.print("POST Received Telemetry -->> rxVCC =");
		//Serial.print(rxVCC);
		//Serial.print("\trxVCA =");Serial.println(rxVCA);
		if (rxVCC > 0 && rxVCA > 0) {
			telemetrystatus = true;
			signalCode = 1;
		}
	}
	
	tft.fillRect(220, 120, 100, 17, BLACK);// Clear Waiting Message
	tft.setCursor(220,120);
	tft.setTextColor(GREEN);
	tft.println("OK!");  
	
	delay(1000);

}

void updateStatus() {

	tft.setCursor(18, 3);
	tft.setTextColor(WHITE);
	tft.setTextSize(2);
	tft.println(astromechName);
	tft.drawFastHLine(0, 19, tft.width(), WHITE);

	updateRSSI();
	updateBattery(180,txVCCout,"TX");
	updateBattery(242,rxVCCout,"RX");

	nextStatusBarUpdate = millis() + updateStatusDelay;

}

void updateGrid() {

	int i;
	int xOffset = 0;
	int yOffset = 30;
	int colIndex = 1;
	int buttonId;

	int stickyIndex;
	boolean stickyFlag;
	
	startAt = ((curPage-1)*itemsPerPage);
	
	tft.fillRect(0, 21, 320, 220, BLACK);

	// Render Pagination
	tft.fillRect(0, 220, 130, 20, WHITE);
	tft.fillRect(190, 220, 130, 20, WHITE);
	tft.fillTriangle(5,230, 25,225, 25,235, BLACK);
	tft.fillTriangle(315,230, 295,225, 295,235, BLACK);
	tft.setCursor(150, 222);
	tft.setTextColor(WHITE);
	tft.setTextSize(2);
	tft.println(pad(curPage,2));

	for (i=startAt; i<(startAt+itemsPerPage); i++) {

		//int totalTriggers = sizeof(gridTriggers)/sizeof(char*);
		//Serial.print("Size of array: ");
		//Serial.println(totalTriggers);

		if ((i >= TOTAL_TRIGGERS) || (i>251)) { break; }

		xOffset = (colIndex-1)*(boxMargin+boxWidth);

		tft.fillRect(xOffset, yOffset, boxWidth, boxHeight, GRAY);

		buttonId = i+1;
		
		stickyFlag = stickyHash.getValueOf(buttonId);
		//Serial.print("Rendering trigger ID# "); Serial.println(buttonId);

		stickyIndex = getStickyTrigger(stickyTriggers, buttonId);
		if (stickyIndex >= 0) {
			stickyFlag = stickyHash.getValueOf(buttonId);
			//Serial.print("Sticky status: "); Serial.println(stickyFlag);
			if (stickyFlag) {
				tft.drawRect(xOffset, yOffset, boxWidth, boxHeight, WHITE);			
			}
		} else {
			//Serial.println("Not sticky.");
		}

		tft.setCursor((xOffset+10), (yOffset+((boxHeight/2)-4)));
		tft.setTextColor(WHITE);
		tft.setTextSize(1);
		tft.println(gridTriggers[i]);

		colIndex++;
		if (colIndex > boxesPerRow) {
			colIndex = 1;
			yOffset = yOffset+boxMargin+boxHeight;
		}

	}

}

void updateRSSI() {

	int bars = map(rxRssi, rssiMin, rssiMax, 0, 5);

	//Serial.print("RSSI bars: ");
	//Serial.print(bars);
	//Serial.print(" (RSSI value was ");
	//Serial.print(rxRssi);
	//Serial.println(")");

	tft.fillRect(298, 2, 22, 17, BLACK);

	tft.drawFastVLine(298, 14, 2, GRAY);
	tft.drawFastVLine(299, 14, 2, GRAY);
	
	tft.drawFastVLine(303, 12, 4, GRAY);
	tft.drawFastVLine(304, 12, 4, GRAY);
	
	tft.drawFastVLine(308, 10, 6, GRAY);
	tft.drawFastVLine(309, 10, 6, GRAY);
	
	tft.drawFastVLine(313, 7, 9, GRAY);
	tft.drawFastVLine(314, 7, 9, GRAY);
	
	tft.drawFastVLine(318, 3, 13, GRAY);
	tft.drawFastVLine(319, 3, 13, GRAY);
	
	if (bars>=1) {
		tft.drawFastVLine(298, 14, 2, WHITE);			// Signal =1 
		tft.drawFastVLine(299, 14, 2, WHITE);
	}  
	if (bars>=2) {
		tft.drawFastVLine(303, 12, 4, WHITE);			// Signal =2
		tft.drawFastVLine(304, 12, 4, WHITE);
	}
	if (bars>=3) {
		tft.drawFastVLine(308, 10, 6, WHITE);			// Signal =3 
		tft.drawFastVLine(309, 10, 6, WHITE);
	}
	if (bars>=4) {
		tft.drawFastVLine(313, 7, 9, WHITE);			// Signal =4 
		tft.drawFastVLine(314, 7, 9, WHITE);
	}
	if (bars>=5) {
		tft.drawFastVLine(318, 3, 13, WHITE);			// Signal =5 
		tft.drawFastVLine(319, 3, 13, WHITE);
	}

	if (bars<1) {
		signalCode = 0;
	} else {
		if (signalCode < 2) {
			signalCode = 1;
		}
	}

}

void updateBattery(int battx, float vcc, String display) {

	int battIcon = battx+27;

	tft.fillRect(battIcon, 0, 9, 17, BLACK);       				// Erase Battery Status area
	
	// Draw the battery outline in white
	tft.drawFastHLine((battIcon+2), 0, 4, WHITE);				// This is the little top of the battery
	tft.drawFastHLine((battIcon+2), 1, 4, WHITE);
	tft.drawRect(battIcon, 2, 8, 15, WHITE);					// Body of the battery
	
	if (vcc >= vinSTRONG) {   
		tft.fillRect((battIcon+1), 3, 6, 14, GREEN);			// If Battery is strong then GREEN  
	} else if (vcc >=vinWEAK) {
		tft.fillRect((battIcon+1), 8, 6, 9, YELLOW);			// If Battery is medium then YELLOW
	} else {
		tft.fillRect((battIcon+1), 12, 6, 4, RED);				// If Battery is low then RED
	}    

	tft.setCursor((battx+39), 10);	
	tft.setTextColor(WHITE);
	tft.setTextSize(1);
	tft.println(display);

	/* VOLTAGE & AMPERAGE DISPLAY */

	tft.fillRect(battx, 0, 20, 18, BLACK);

	tft.setCursor(battx, 2);	
	tft.setTextColor(GREEN);
	if (vcc < 4.9) { tft.setTextColor(YELLOW); }
	if (vcc < 4.0) { tft.setTextColor(RED); }
	if (vcc >= 7) { tft.setTextColor(RED); }
	tft.setTextSize(1);
	if (vcc > 9.9) { tft.print("10+"); }
	else { tft.print(vcc,1); }
	tft.println("V");

	tft.setCursor(battx, 10);	
	tft.setTextColor(GREEN);
	if (vcc < 4.9) { tft.setTextColor(YELLOW); }
	if (vcc < 4.0) { tft.setTextColor(RED); }
	tft.setTextSize(1);
	if (vcc > 9.9) { tft.print("10+"); }
	else { tft.print(vcc,1); }
	tft.println("A");
	
}

void updateSignal() {

	int signalLag = 600; // Meaningful signals shouldn't blink by too quick...

	boolean displayDelay = true;
	unsigned long thisMilli = millis();
	
	Serial.print("signalCode: ");
	Serial.print(signalCode);
	Serial.print(" tx/rx signatures: ");
	Serial.print(txSignature);
	Serial.print("/");
	Serial.println(rxSignature);

	if ((signalCode == 2) && (thisMilli >= signalLastSent+signalTimeout)) {
		signalCode = -1;
		newSignature();
	}

	if (thisMilli >= signalNextMilli) {
		displayDelay = false;
	}

	if (!displayDelay) {
		switch(signalCode) {
			case -1:
				tft.fillCircle(7, 9, 6, RED);
				break;
			case 1:
				tft.fillCircle(7, 9, 6, WHITE);
				break;	
			case 2:
				tft.fillCircle(7, 9, 6, YELLOW);
				break;	
			case 3:
				tft.fillCircle(7, 9, 6, GREEN);
				break;
			default:
				tft.fillCircle(7, 9, 6, GRAY);
		}
		if (signalCode == 3) {
			signalCode = 1;
		}
		signalNextMilli = thisMilli+signalLag;
	}

}

void getTouch() {  
	
	int touchedRelativeX;
	int touchedRelativeY;

	Point p = ts.getPoint();

	//Serial.print("Registered touch, pressure: "); Serial.println(p.z);
	//sensitivity = ts.pressureThreshhold; // Default?

	if (p.z > sensitivity) {

		Serial.print("RAW X = "); Serial.print(p.x);				// DEBUG CODE
		Serial.print("\tY = "); Serial.print(p.y);					// DEBUG CODE
		Serial.print("\tPressure = "); Serial.print(p.z);			// DEBUG CODE

		// We also have to re-align LCD X&Y to Touch X&Y
		touchedY = map(p.x, TS_MINX, TS_MAXX, 240, 0);				// Notice mapping touchedY to p.x
		touchedX = map(p.y, TS_MAXY, TS_MINY, 320, 0);				// Notice mapping touchedX to p.y

		Serial.print("\tTouched X = "); Serial.print(touchedX);		// DEBUG CODE
		Serial.print("\tTouched Y = "); Serial.print(touchedY);		// DEBUG CODE
		Serial.println(" ");										// DEBUG CODE
		
		if (touchedY >= 206 && touchedX < 160) {
			curPage--;
			if (curPage < 1) { curPage = countedPages; }
			updateGrid();
		}		

		if (touchedY >= 206 && touchedX > 160) {
			curPage++;
			if (curPage > countedPages) { curPage = 1; }
			updateGrid();
		}		

		if (touchedY >= 20 && touchedY <= 205) {

			touchedRelativeX = touchedX;
			touchedRelativeY = touchedY-20;

			touchedRow = (touchedRelativeY/(186/rowsPerPage))+1;
			touchedCol = (touchedRelativeX/(320/boxesPerRow))+1;

			//Serial.print("Touched item: ");
			//Serial.print(touchedRow);
			//Serial.print(",");
			//Serial.print(touchedCol);

			triggerEvent = ((startAt+(touchedRow-1)*boxesPerRow)+touchedCol);

			//Serial.print(" (");
			//Serial.print(triggerEvent);
			//Serial.println(")");

			int touchedSticky = getStickyTrigger(stickyTriggers, triggerEvent);
			if (touchedSticky >= 0) {
				toggleStickyTrigger(triggerEvent);
			}

		}
		
	}

}

void RXdata() {
 
	xbee.readPacket(50);
	
	if (xbee.getResponse().isAvailable()) {
	
		if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) {

			lastTransmission = millis();
			if (signalCode < 2) {
				signalCode = 1;
			}

			Serial.println("Got ZB_RX_RESPONSE...");

			xbee.getResponse().getZBRxResponse(rx);

			Serial.print("Rx:");
			Serial.print("\t");   Serial.print(rx.getData()[0]);
			Serial.print("\t");   Serial.print(rx.getData()[1]);
			Serial.print("\t");   Serial.print(rx.getData()[2]);
			Serial.print("\t");   Serial.print(rx.getData()[3]);
			Serial.print("\t");   Serial.print(rx.getData()[4]);
			Serial.print("\t"); Serial.println(rx.getData()[5]);
			Serial.println("\tReceived zbTx");

			rxVCC = rx.getData()[0];
			rxVCA = rx.getData()[1];
			rxRssi = rx.getData()[2];

			rxSignature = rx.getData()[3];
			rxResponded = rx.getData()[4];

			rxVCCout = (float)rxVCC/10.0;
			rxVCAout = (float)rxVCA/10.0;

			Serial.print("Receiver RSSI: "); Serial.println(rxRssi);
			Serial.print("Receiver Voltage: "); Serial.println(rxVCCout);
			Serial.print("Receiver Amperage: "); Serial.println(rxVCAout);

			if ((signalCode == 2) && (txSignature == rxSignature)) {
				signalCode = 3;
				signalNextMilli = 0;
				newSignature();
			}

		}

	}
	
}

void TXdata() {

	byte sendSig = 0;
	if (triggerEvent > 0) {
		sendSig = txSignature;
	}

	payload[0]=joyx;			// JoyX ranges from approx 30 - 220
	payload[1]=joyy;			// JoyY ranges from approx 29 - 230
	payload[2]=accx;			// AccX ranges from approx 70 - 182
	payload[3]=accy;			// AccY ranges from approx 65 - 173
	payload[4]=accz;			// AccZ ranges from approx 65 - 173
	payload[5]=zbut;			// Z Button Status
	payload[6]=cbut;  			// C Button Status
	payload[7]=triggerEvent;	// 0 to 254
	payload[8]=sendSig;			// Signature of triggerEvent

	/*
	Serial.print("joyx: "); Serial.print((byte)joyx,DEC);
	Serial.print("\tjoyy: "); Serial.print((byte)joyy,DEC);
	Serial.print("\taccx: "); Serial.print((byte)accx,DEC);
	Serial.print("\taccy: "); Serial.print((byte)accy,DEC);
	Serial.print("\taccz: "); Serial.print((byte)accz,DEC);
	Serial.print("\tzbut: "); Serial.print((byte)zbut,DEC);
	Serial.print("\tcbut: "); Serial.print((byte)cbut,DEC);
	Serial.print("\ttrigger: "); Serial.println(triggerEvent);
	*/
	
	xbee.send(zbTx);

	if (triggerEvent > 0) {
		signalCode = 2;
		signalLastSent = millis();
		signalNextMilli = 0;
		lastSigSent = txSignature;
	}
	
	triggerEvent = 0;
	zbut=0;
	cbut=0;

}  

void xbeeSL() {

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

	int VCCvalue = analogRead(analogVCCinput);
	float vout = 0.0;											// For voltage out measured analog input
	float vcc = 0.0;											// Voltage calculated, since the divider allows for 15 volts

	vout= (VCCvalue * 5.0)/1024.0;								// Voltage coming out of the voltage divider
	vcc = vout / (R2/(R1+R2));									// Voltage based on vout to display battery status
	txVCC = (vcc)*10;
	
	//Serial.print("Battery Voltage: "); Serial.println(txVCC);
	
}

void getVCA() {
	//VCAvalue = random(1,999);
}

void checkPulse() {
	unsigned long timeNow = millis();
	if (timeNow >= (lastTransmission+signalTimeout+1000)) {
		signalCode = 0;
		signalNextMilli = 0;
		rxVCCout = 0;
		rxRssi = rssiMin;
	}
}

/*////////////////////////////////////////////////////////////////////////////////////////////////*/
///////////////////////* Utilities *////////////////////////////////////////////////////////////////
/*////////////////////////////////////////////////////////////////////////////////////////////////*/

void countPages() {
	initStickyHash();
	countedPages = TOTAL_TRIGGERS/itemsPerPage;
	if ((TOTAL_TRIGGERS % itemsPerPage) > 0) {
		countedPages++;
	}
	//Serial.print("total pages: ");
	//Serial.println(countedPages);	
}

void initStickyHash() {

	for (int i=0; i<TOTAL_STICKY; i++) {
		addStickyTrigger(stickyTriggers[i],0);
	}

	//stickyHash.debug();

}

void addStickyTrigger(int trigger, boolean init) {
	stickyHash[hashMapIndex](trigger,init);
	hashMapIndex++;
}

int getStickyTrigger(int* haystack, int needle) {
	int found = -1;
	for (int i=0; i<TOTAL_STICKY; i++) {
		if (needle == haystack[i]) {
			found = i;
			break;
		}
	}
	return found;
}

void toggleStickyTrigger(int trigger) {

	boolean triggerState = stickyHash.getValueOf(trigger);
	int triggerIndex = stickyHash.getIndexOf(trigger);

	if ((trigger <= (startAt+itemsPerPage)) && (trigger > startAt)) {

		int screenPos = trigger-startAt;
		int screenRow = (trigger/boxesPerRow)+1;
		int screenCol = (trigger%boxesPerRow);
		if (screenCol == 0) {
			screenRow--;
			screenCol = boxesPerRow;
		}
		//Serial.print("Touched pad: ");
		//Serial.print(screenRow); Serial.print(",");
		//Serial.println(screenCol);

		int touchedPadX = (((boxWidth+boxMargin)*(screenCol-1))+0);
		int touchedPadY = (((boxHeight+boxMargin)*(screenRow-1))+20+boxMargin);

		if (triggerState) {
			tft.drawRect(touchedPadX, touchedPadY, boxWidth, boxHeight, GRAY);			
		} else {
			tft.drawRect(touchedPadX, touchedPadY, boxWidth, boxHeight, WHITE);			
		}

	}

	triggerState = !triggerState;
	stickyHash[triggerIndex](trigger,triggerState);
}

void newSignature() {
	txSignature++;
	if (txSignature >= 250) {
		txSignature = 0;
	}
}

String pad(int number, byte width) {
	String output;
	int currentMax = 10;
	for (byte i=1; i<width; i++) {
		if (number < currentMax) {
			output += "0";
		}
		currentMax *= 10;
	} 
	output += String(number);
	//Serial.print("output: ");
	//Serial.println(output);
	return output;
}