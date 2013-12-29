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

int mla = 18;
int mlb = 19;
int mlc = 20;
int mld = 21;
int motorPin = 5;
int currentMode = 0;
unsigned long modeTimer = 0;
unsigned long modeDelay = 200;
unsigned long timeNow = 1000;

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

	pinMode(motorPin, OUTPUT);
	pinMode(mla, HIGH);	
	pinMode(mlb, HIGH);	
	pinMode(mlc, HIGH);	
	pinMode(mld, HIGH);	
			
	nunchuk.init();										// Initialize Nunchuk using I2C

	switchMode();	

}

void loop() {
	
	timeNow = millis();

	//digitalWrite(motorPin, HIGH);
	//delay(150);
	//digitalWrite(motorPin, LOW);
	//delay(150);

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

void switchMode() {
	if (timeNow >= modeTimer+modeDelay) {
		modeTimer = millis();
		switch(currentMode) {
			case 1:
				currentMode = 2;
				digitalWrite(mla, LOW);
				digitalWrite(mlb, HIGH);
				break;		
			case 2:
				currentMode = 3;
				digitalWrite(mlb, LOW);
				digitalWrite(mlc, HIGH);
				break;		
			case 3:
				currentMode = 4;
				digitalWrite(mlc, LOW);
				digitalWrite(mld, HIGH);
				break;		
			case 4:
				currentMode = 1;
				digitalWrite(mld, LOW);
				digitalWrite(mla, HIGH);
				break;
			default:
				currentMode = 1;
				digitalWrite(mla, HIGH);
		}
	}
}

void TXdata() {

	payload[0]=joyx;			// JoyX ranges from approx 30 - 220
	payload[1]=joyy;			// JoyY ranges from approx 29 - 230
	payload[2]=accx;			// AccX ranges from approx 70 - 182
	payload[3]=accy;			// AccY ranges from approx 65 - 173
	payload[4]=accz;			// AccZ ranges from approx 65 - 173
	payload[5]=zbut;			// Z Button Status
	payload[6]=cbut;  			// C Button Status
	payload[7]=triggerEvent;	// 0 to 254
	payload[8]=currentMode;		// What control mode is selected?

	xbee.send(zbTx);

	triggerEvent = 0;
	zbut=0;
	cbut=0;

}