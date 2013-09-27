/*
 * Astromech RSeries Receiver for the R2 Builders Club
 * Skwerl's Fork 
 *  
 * Heavily based on Michael Erwin's
 * RSeries Open Controller Project
 * http://code.google.com/p/rseries-open-control/
 *
 * Requires Arduino 1.0 IDE
 *
*/

/*////////////////////////////////////////////////////////////////////////////////////////////////*/
///////////////////////* Global Config *////////////////////////////////////////////////////////////
/*////////////////////////////////////////////////////////////////////////////////////////////////*/

// Placeholder

/*////////////////////////////////////////////////////////////////////////////////////////////////*/
///////////////////////* Libraries *////////////////////////////////////////////////////////////////
/*////////////////////////////////////////////////////////////////////////////////////////////////*/

#include <XBee.h>
#include <Servo.h>
#include <Wire.h>

/*////////////////////////////////////////////////////////////////////////////////////////////////*/
///////////////////////* XBee Configuration *///////////////////////////////////////////////////////
/*////////////////////////////////////////////////////////////////////////////////////////////////*/

XBee xbee = XBee();
XBeeResponse response = XBeeResponse();
ZBRxResponse rx = ZBRxResponse();
ModemStatusResponse msr = ModemStatusResponse();

uint8_t idCmd[] = {'I','D'};
uint8_t opCmd[] = {'O','P'};
uint8_t payload[] = { '0', '0', '0', '0', '0', '0'}; // Our XBee Payload of 6 values (txVCC=2, txVCA=2, future=2)

AtCommandRequest atRequest = AtCommandRequest(opCmd);
AtCommandResponse atResponse = AtCommandResponse();

XBeeAddress64 addr64 = XBeeAddress64(0x0013a200, 0x40a8e69c);
ZBTxRequest zbTx = ZBTxRequest(addr64, payload, sizeof(payload));
ZBTxStatusResponse txStatus = ZBTxStatusResponse();

byte xbAPIidResponse = 0x00;

/*////////////////////////////////////////////////////////////////////////////////////////////////*/
///////////////////////* Control & Sound Configuration *////////////////////////////////////////////
/*////////////////////////////////////////////////////////////////////////////////////////////////*/

byte joyx, joyy, accx, accy, accz, zbut, cbut;
int triggerEvent;

int randNum;
int mp3Byte = 0;
boolean mp3Playing = false;

int joyxmin = 0;
int joyxmax = 255;
int joyymin = 0;
int joyymax = 255;
int accxmin = 70;
int accxmax = 182;
int accymin = 65;
int accymax = 173;
int acczmin = 65;
int acczmax = 173;

/*////////////////////////////////////////////////////////////////////////////////////////////////*/
///////////////////////* PWM & Servo Configuration *////////////////////////////////////////////////
/*////////////////////////////////////////////////////////////////////////////////////////////////*/

Servo chan1servo;
Servo chan2servo;
Servo chan3servo;

int servo1Pin = 2;
int servo2Pin = 3;
int servo3Pin = 4;

// Ranges
int chan1Min = 30;			// Channel 1 Min - Left Right  
int chan1Max = 220;			// Channel 1 Max - Left Right
int chan2Min = 30;			// Channel 2 Min - Forward & Reverse Speed 
int chan2Max = 220;			// Channel 2 Max - Forward & Reverse Speed
int chan3Min = 105;			// Channel 3 Min - Dome Rotation LEFT 
int chan3Max = 75;			// Channel 3 Max - Dome Rotation RIGHT

// Weirdness Corrections
int chan1correct = 0;
int chan2correct = 0;
int chan3correct = -2;		// Syren10 neutral is more like 92

// Neutral Adjustments 
int chan1Neutral = min(chan1Min,chan1Max)+(abs(chan1Max-chan1Min)/2)+chan1correct;
int chan2Neutral = min(chan2Min,chan2Max)+(abs(chan2Max-chan2Min)/2)+chan2correct;
int chan3Neutral = min(chan3Min,chan3Max)+(abs(chan3Max-chan3Min)/2)+chan3correct;

/*////////////////////////////////////////////////////////////////////////////////////////////////*/
///////////////////////* Telemetry Configuration *//////////////////////////////////////////////////
/*////////////////////////////////////////////////////////////////////////////////////////////////*/

int loop_cnt = 0;

byte telemetryVCCMSB;									// MSB of Voltage VCC
byte telemetryVCCLSB;									// LSB of Voltage VCC
 
byte telemetryVCAMSB;									// MSB of Current VCA
byte telemetryVCALSB;									// LSB of Current VCA

unsigned int txVCC = 0;
unsigned int txVCA = 0;

int rxErrorCount = 0;

int analogVCCinput = 5;									// RSeries Receiver default VCC input is A5
float R1 = 47000.0;										// >> resistance of R1 in ohms << the more accurate these values are
float R2 = 24000.0;										// >> resistance of R2 in ohms << the more accurate the measurement will be

/*////////////////////////////////////////////////////////////////////////////////////////////////*/
///////////////////////* Arduino Functions *////////////////////////////////////////////////////////
/*////////////////////////////////////////////////////////////////////////////////////////////////*/

void setup() {
	
	Serial.begin(9600);
	Serial1.begin(19200);
	Serial2.begin(38400);

	xbee.setSerial(Serial1);							// Setup Xbee to use Serial1
	xbee.begin(19200);									// Setup Xbee to begin at 19200

	Serial.println(" ");  
	Serial.println(" ");  
	Serial.println("Listening...");  

	startupChime();

	chan1servo.attach(servo1Pin);  // Foot motors left/right
	chan2servo.attach(servo2Pin);  // Foot motors forward/back
	chan3servo.attach(servo3Pin);  // Dome motor 

	clearServos();

	//delay(3000);
	//getOP();

}

void loop() {

	loop_cnt++;

	if (loop_cnt > 50) {
		sendTelemetry();
		loop_cnt = 0;
	}

	xbee.readPacket();
	
	if (Serial2.available()) {
		mp3Byte = Serial2.read();
		Serial.print("I received: ");
		Serial.println(mp3Byte, DEC);
		if (mp3Byte == 88) {
			mp3Playing = false;
		}
	}

	if (xbee.getResponse().isAvailable()) {
		//Serial.println("Got something...");
		if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) {
			Serial.println("Got a ZB RX Packet...");
			xbee.getResponse().getZBRxResponse(rx);
			handleEvent();
			if (rx.getOption() == ZB_PACKET_ACKNOWLEDGED) {
				Serial.println("Sender got ACK");  
			} else {
				Serial.println("Sender did not get ACK");  
			}
			// Set dataLed PWM to value of the first byte in the data
			// analogWrite(dataLed, rx.getData(0));
		} else if (xbee.getResponse().getApiId() == MODEM_STATUS_RESPONSE) {
			xbee.getResponse().getModemStatusResponse(msr);
			// The local XBee sends this response on certain events, like association/dissociation
			if (msr.getStatus() == ASSOCIATED) {
				Serial.println("Associated");
				getOP();
			} else if (msr.getStatus() == DISASSOCIATED) {
				Serial.println("Disassociated");  
			} else {
				Serial.println("Hmm...");  
			}
		} else {
			Serial.println("WTF?");  
		}
	} else if (xbee.getResponse().isError()) {
		Serial.print("Error reading packet.  Error code: ");  
		Serial.println(xbee.getResponse().getErrorCode());
	}
}

/*////////////////////////////////////////////////////////////////////////////////////////////////*/
///////////////////////* RSeries Functions *////////////////////////////////////////////////////////
/*////////////////////////////////////////////////////////////////////////////////////////////////*/

void handleEvent() {

	int joyx = rx.getData()[0];				// Left/Right
	int joyy = rx.getData()[1];				// Up/Down
	int accx = rx.getData()[2];				// Nunchuk X Acceleramator
	int accy = rx.getData()[3];				// Nunchuk Y Acceleramator
	int accz = rx.getData()[4];				// Nunchuk Z Acceleramator
	triggerEvent = rx.getData()[7];			// TriggerEvent
	//int futureEvent = rx.getData()[8];	// Future Event Payload

	/*
	Serial.print(">> joyx =");Serial.print(joyx);
	Serial.print("\tjoyy =");Serial.print(joyy);
	Serial.print("\taccx =");Serial.print(accx);
	Serial.print("\taccy =");Serial.print(accy);
	Serial.print("\taccz =");Serial.print(accz);
	Serial.print("\ttriggerEvent =");Serial.println(triggerEvent);
	*/

	// Normalize, emulate "dead stick" zones...
	
	if (joyx < 120) { joyx = joyx; }
	else if (joyx > 136) { joyx = joyx; }
	else { joyx = 128; }

	if (joyy < 128) { joyy = joyy; }
	else if (joyy > 136) { joyy = joyy; }
	else { joyy = 128; }
	
	//if (accx < 90) { accx = 60; }
	//else if (accx > 160) { accx = 120; }
	//else { accx = 90; }

	switch (triggerEvent) {

		// Nunchuk triggers first...
		
		case 252:
			Serial.println("C Button Pressed");
			if (joyy >= 133) {
				// Joystick up, play "happy" sound:
				randNum = random(41,83);
				playSound(randNum);
			} else if (joyx >= 133) {
				// Joystick left, play "scared" sound:
				randNum = random(83,137);
				playSound(randNum);
			} else if (joyx <= 123) {
				// Joystick right, play "angry" sound:
				randNum = random(137,150);
				playSound(randNum);
			} else if (joyy <= 123) {
				// Joystick down, Send STOP command:
				stopSound();
			} else {
				// Joystick neutral, play "casual" sound:
				randNum = random(1,41);
				playSound(randNum);
			}
			break;

		case 253:
			Serial.println("Z Button Pressed");

			// Dome spin:

			joyx = map(joyx, joyxmin, joyxmax, chan3Min, chan3Max);

			Serial.print("Writing ");
			Serial.print(joyx);
			Serial.println(" to servo 3...");

			chan3servo.write(joyx);

			break;

		case 254:
			Serial.println("Z+C Buttons Pressed");
			// Play Leia's message:
			playSound(156);
			break;

		// Any touch screen triggers?

		case 1:
			playSound(15);
			break;
		case 2:
			playSound(15);
			break;
		case 3:
			playSound(15);
			break;
		case 4:
			playSound(43);
			break;
		case 5:
			playSound(150);
			break;
		case 6:
			playSound(151);
			break;
		case 7:
			playSound(152);
			break;
		case 8:
			playSound(153);
			break;
		case 9:
			playSound(154);
			break;
		case 10:
			playSound(155);
			break;
		case 11:
			playSound(156);
			break;
		case 12:
			playSound(157);
			break;
		case 13:
			playSound(158);
			break;
		case 14:
			playSound(159);
			break;
		case 15:
			playSound(160);
			break;
		case 16:
			playSound(161);
			break;
		case 17:
			playSound(162);
			break;
		case 18:
			playSound(163);
			break;
		case 19:
			playSound(164);
			break;

		default:
			clearServos();

	}

}

void startupChime() {

	// This function can execute any chime or sequence
	// You wish to see/hear when your droid comes online

	Serial2.write('t');
	//Serial2.write(164);
	Serial2.write(15);

}

void clearServos() {

	chan3servo.write(chan3Neutral);

}

void playSound(int sample) {
	if (mp3Playing == false) {
		Serial.print("Playing sample #");
		Serial.println(sample);
		Serial2.write('t');
		Serial2.write(sample);
		mp3Playing = true;
	}
}

void stopSound() {
	if (mp3Playing == true) {
		Serial.println("Stop");
		Serial2.write('O');
		mp3Playing = false;
	}
	clearServos();
}

void sendTelemetry() {

	getVCC();
	getVCA();
	
	Serial.print("txVCC="); Serial.print(txVCC);
	Serial.print("\ttxVCA="); Serial.println(txVCA);

	// Take the value of stored in txVCC & txVCA, and convert them to MSB & LSB 2 bytes via a bitshift operation 
	telemetryVCCMSB = (txVCC >> 8) & 0xFF;  
	telemetryVCCLSB = txVCC & 0xFF;

	telemetryVCAMSB = (txVCA >> 8) & 0xFF;  
	telemetryVCALSB = txVCA & 0xFF;

	/*
	//  NOTE:  To make the 2 Byte (MSB & LSB) values back into an int use the following code:    
	int telemetryVCC = (int)(word(telemetryVCCMSB,telemetryVCCLSB));    
	int telemetryVCA = (int)(word(telemetryVCAMSB,telemetryVCALSB));
	*/
	
	payload[0] = telemetryVCCMSB;						// MSB of txVCC Voltage from Receiver. Test Voltage sent is 136 as a 2 byte byte 0x88h, this will get divided by 10
	payload[1] = telemetryVCCLSB;						// LSB of txVCC
	payload[2] = telemetryVCAMSB;						// MSB of txVCA Amperage from Receiver. 
	payload[3] = telemetryVCALSB;						// LSB of txVCA
	payload[4] = '1';									// Future Use - User usable
	payload[5] = '2';									// Future Use - Transmitting RX Error codes back to the controller
	
	xbee.send(zbTx); 

	Serial.print("Tx:");
	Serial.print("\t");   Serial.print(payload[0]);
	Serial.print("\t");   Serial.print(payload[1]);
	Serial.print("\t");   Serial.print(payload[2]);
	Serial.print("\t");   Serial.print(payload[3]);
	Serial.print("\t");   Serial.print(payload[4]);
	Serial.print("\t"); Serial.println(payload[5]);
	Serial.println("\tSent zbTx");

}

void getVCC() {

	int VCCvalue = analogRead(analogVCCinput);
	float vout = 0.0;											// For voltage out measured analog input
	float vcc = 0.0;											// Voltage calculated, since the divider allows for 15 volts

	vout= (VCCvalue * 5.0)/1024.0;								// Voltage coming out of the voltage divider
	vcc = vout / (R2/(R1+R2));									// Voltage based on vout to display battery status
	txVCC = (vcc)*10;
	
	Serial.print("Battery Voltage: "); Serial.println(txVCC);
	
}
  
void getVCA() {
	txVCA = random(1,999);
}  

void getOP() {

	atRequest.setCommand(opCmd);  
	xbee.send(atRequest);
	if (xbee.readPacket(5000)) {
		xbee.getResponse().getAtCommandResponse(atResponse);			
		if (atResponse.isOk()) {
			Serial.print("Command [");
			Serial.print(atResponse.getCommand()[0]);
			Serial.print(atResponse.getCommand()[1]);
			Serial.println("] was successful!");
			if (atResponse.getValueLength() > 0) {
				Serial.print("Command value length is ");
				Serial.println(atResponse.getValueLength(), DEC);
				Serial.print("Command value: ");
				for (int i = 0; i < atResponse.getValueLength(); i++) {
					Serial.print(atResponse.getValue()[i], HEX);
					Serial.print(" ");
				}
				Serial.println("");
			} else {
				Serial.print("Response empty");
			}
		} else {
			Serial.print("Command failed");
		}
	}

}