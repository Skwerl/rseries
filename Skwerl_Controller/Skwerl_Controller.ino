/*
 * Skwerl's FioSaber Controller
 *
 * For FioSaber Prototype A
 * Arduino Fio v3 Shield
 * Requires Arduino 1.0 IDE
 *
*/

/*////////////////////////////////////////////////////////////////////////////////////////////////*/
///////////////////////* Libraries *////////////////////////////////////////////////////////////////
/*////////////////////////////////////////////////////////////////////////////////////////////////*/

#include <Arduino.h>
#include <XBee.h>               
#include <Wire.h>               // Used to read the I2C data from Nunchuck
#include <ArduinoNunchuk.h>     // Gabriel Bianconi @ http://www.gabrielbianconi.com/projects/arduinonunchuk/
#include <WiiClassic.h>			// Tim Hirzel @ http://playground.arduino.cc/Main/WiiClassicController

// If your Wire.h doesn't understand "send" or "receive" (no member error),
// Update Wire.h, or open WiiClassic.h and try replacing with "write" and "read" respectively.

/*////////////////////////////////////////////////////////////////////////////////////////////////*/
///////////////////////* Mode Configuration *///////////////////////////////////////////////////////
/*////////////////////////////////////////////////////////////////////////////////////////////////*/

int txLED = 19;
int rgbLED[] = {21,20,22};
int motorPin = 5;

int currentMode = 0;
unsigned long modeTimer = 0;
unsigned long modeDelay = 200;
unsigned long timeNow = 1000;

const boolean ON = LOW;
const boolean OFF = HIGH;
const boolean RED[]     = { ON, OFF, OFF };
const boolean GREEN[]   = { OFF, ON, OFF };
const boolean BLUE[]    = { OFF, OFF, ON };
const boolean YELLOW[]  = { ON, ON, OFF };
const boolean CYAN[]    = { OFF, ON, ON };
const boolean MAGENTA[] = { ON, OFF, ON };
const boolean WHITE[]   = { ON, ON, ON };
const boolean BLACK[]   = { OFF, OFF, OFF };
const boolean* COLORS[] = { RED, GREEN, BLUE, YELLOW, CYAN, MAGENTA, WHITE, BLACK };

/*////////////////////////////////////////////////////////////////////////////////////////////////*/
///////////////////////* Button Configuration *///////////////////////////////////////////////////////
/*////////////////////////////////////////////////////////////////////////////////////////////////*/

int buttonPin = 18;
int buttonLongPressTime = 25;

enum { EV_NONE=0, EV_SHORTPRESS, EV_LONGPRESS };
boolean buttonPressed;
int buttonPressCounter;

/*////////////////////////////////////////////////////////////////////////////////////////////////*/
///////////////////////* XBee Configuration *///////////////////////////////////////////////////////
/*////////////////////////////////////////////////////////////////////////////////////////////////*/

int xbeebps = 19200;
uint8_t payload[] = { '0', '0', '0', '0', '0', '0', '0', '0', '0'};
Rx16Response rx16 = Rx16Response();
XBee xbee = XBee();
XBeeResponse response = XBeeResponse();

ZBRxResponse rx = ZBRxResponse();
ModemStatusResponse msr = ModemStatusResponse();
 
XBeeAddress64 addr64 = XBeeAddress64(0x0013a200, 0x40a8e65b);
ZBTxRequest zbTx = ZBTxRequest(addr64, payload, sizeof(payload));
ZBTxStatusResponse txStatus = ZBTxStatusResponse();

uint8_t opCmd[] = {'O','P'};
AtCommandRequest atRequest = AtCommandRequest(opCmd);
AtCommandResponse atResponse = AtCommandResponse();

int xbATResponse = 0xFF;

/*////////////////////////////////////////////////////////////////////////////////////////////////*/
///////////////////////* Wii Nunchuk Configuration *////////////////////////////////////////////////
/*////////////////////////////////////////////////////////////////////////////////////////////////*/

ArduinoNunchuk nunchuk = ArduinoNunchuk();

byte joyx, joyy, accx, accy, accz, zbut, cbut;

byte triggerEvent;

/*////////////////////////////////////////////////////////////////////////////////////////////////*/
///////////////////////* Arduino Functions *////////////////////////////////////////////////////////
/*////////////////////////////////////////////////////////////////////////////////////////////////*/

void setup() {

	Serial1.begin(xbeebps);
	xbee.setSerial(Serial1);
	xbee.begin(xbeebps);

	pinMode(buttonPin, INPUT);
	digitalWrite(buttonPin, HIGH);
	buttonPressed = false;
	buttonPressCounter = 0;
	
	pinMode(txLED, OUTPUT);	
	for(int i=0; i<3; i++) { 
		pinMode(rgbLED[i], OUTPUT); 
	} 
	pinMode(motorPin, OUTPUT);
			
	nunchuk.init();										// Initialize Nunchuk using I2C

	setColor(rgbLED, BLACK);
	switchMode();	

}

void loop() {
	
	timeNow = millis();

	boolean event = handle_button();
	
	switch (event) {
		case EV_SHORTPRESS:
			switchMode();	
			break;
	}

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

	if (joyy <= 123 && zbut == 1 && cbut == 1) {
		switchMode();	
	}

	TXdata();

}

/*////////////////////////////////////////////////////////////////////////////////////////////////*/
///////////////////////* RSeries Functions *////////////////////////////////////////////////////////
/*////////////////////////////////////////////////////////////////////////////////////////////////*/

int handle_button() {

	int event = EV_NONE;
	int buttonPressing = !digitalRead(buttonPin);

	if (!buttonPressing && buttonPressed) {
		if (buttonPressCounter < buttonLongPressTime) {
			event = EV_SHORTPRESS;
		}
	}
	if (buttonPressing) {
		buttonPressCounter++;
		if (buttonPressCounter >= buttonLongPressTime) {
			event = EV_LONGPRESS;
		}		
	} else {
		buttonPressCounter = 0;
	}

	buttonPressed = buttonPressing;
	return event;

}

void switchMode() {
	if (timeNow >= modeTimer+modeDelay) {
		vibeMotor();
		modeTimer = millis();
		switch(currentMode) {
			case 1:
				currentMode = 2;
				setColor(rgbLED, GREEN);
				break;		
			case 2:
				currentMode = 3;
				setColor(rgbLED, YELLOW);
				break;		
			case 3:
				currentMode = 1;
				setColor(rgbLED, RED);
				break;		
			default:
				currentMode = 1;
				setColor(rgbLED, RED);
		}
	}
}

void vibeMotor() {
	digitalWrite(motorPin, HIGH);
	delay(110);
	digitalWrite(motorPin, LOW);
}

void TXdata() {

	payload[0]=joyx;			// JoyX ranges from approx 30 - 220
	payload[1]=joyy;			// JoyY ranges from approx 30 - 220
	payload[2]=accx;			// AccX ranges from approx 70 - 182
	payload[3]=accy;			// AccY ranges from approx 65 - 173
	payload[4]=accz;			// AccZ ranges from approx 65 - 173
	payload[5]=zbut;			// Z Button Status
	payload[6]=cbut;  			// C Button Status
	payload[7]=triggerEvent;	// 0 to 254
	payload[8]=currentMode;		// What control mode is selected?

	if (triggerEvent > 0) {
		digitalWrite(txLED, HIGH);
	} else {
		digitalWrite(txLED, LOW);
	}

	Serial1.write(triggerEvent);

	xbee.send(zbTx);

	triggerEvent = 0;
	zbut=0;
	cbut=0;

}

void setColor(int* led, const boolean* color) { 
	for(int i=0; i<3; i++) { 
		digitalWrite(led[i], color[i]); 
	} 
}