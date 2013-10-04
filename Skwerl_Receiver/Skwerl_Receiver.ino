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
uint8_t dbCmd[] = {'D','B'};
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

boolean holosOn = false;
boolean domeSpinning = false;

boolean moodChill = true;
boolean moodHappy = false;
boolean moodScary = false;
boolean moodAngry = false;

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

int stickMapped = 0;
int stickCenterX = 128;
int stickCenterY = 128;

// Autopilot
boolean autoPilotEngaged = false;
unsigned long baseInterval = 0;
int nextInterval = 0;
int autoPilotStep = 1;
int autoLoopCount = 0;
int autoDomeCnt = 0;
int autoDomePWM = 90;

/*////////////////////////////////////////////////////////////////////////////////////////////////*/
///////////////////////* PWM & Servo Configuration *////////////////////////////////////////////////
/*////////////////////////////////////////////////////////////////////////////////////////////////*/

Servo chan1servo;
Servo chan2servo;
Servo chan3servo;
Servo chan4servo;

int servo1Pin = 2;					// Channel 1
int servo2Pin = 3;					// Channel 2
int servo3Pin = 4;					// Channel 3
int servo4Pin = 5;					// Channel 4
int servo5Pin = 7;					// Channel 5
int servo6Pin = 8;					// Channel 6
int servo7Pin = 9;					// Channel 7
int servo8Pin = 10;					// Channel 8
int servo9Pin = 11;					// Channel 9
int servo10Pin = 12;				// Channel 10

// Ranges
int chan1Min = 30;					// Channel 1 Min - Left/Right  
int chan1Max = 220;					// Channel 1 Max - Left/Right
int chan2Min = 30;					// Channel 2 Min - Forward & Reverse Speed 
int chan2Max = 220;					// Channel 2 Max - Forward & Reverse Speed
int chan3Min = 150;					// Channel 3 Min - Dome Rotation LEFT 
int chan3Max = 30;					// Channel 3 Max - Dome Rotation RIGHT

// Weirdness Corrections
int chan1correct = 0;
int chan2correct = 0;
int chan3correct = -2;

// Neutral Adjustments 
int chan1Neutral = min(chan1Min,chan1Max)+(abs(chan1Max-chan1Min)/2)+chan1correct;
int chan2Neutral = min(chan2Min,chan2Max)+(abs(chan2Max-chan2Min)/2)+chan2correct;
int chan3Neutral = 88;

/*////////////////////////////////////////////////////////////////////////////////////////////////*/
///////////////////////* Telemetry Configuration *//////////////////////////////////////////////////
/*////////////////////////////////////////////////////////////////////////////////////////////////*/

int floodControl = 50;									// Only send feedback every X milliseconds
int telemetryPoll = 2000;								// Delay between internal telemetry updates
unsigned long lastPoll = 0;								// Timestamp of last internal poll
unsigned long lastSignal = 0;							// Timestamp of last telemetry transmission
unsigned long loopTimer = 0;

byte txSignature = 0;
byte txResponder = 0;

int txRSSI;

unsigned int txVCC = 0;
unsigned int txVCA = 0;

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

	chan1servo.attach(servo1Pin);  // Foot motors left/right
	chan2servo.attach(servo2Pin);  // Foot motors forward/back
	chan3servo.attach(servo3Pin);  // Dome motor 
	chan4servo.attach(servo4Pin);  // HPs switch 

	stickCenterX = min(joyxmin,joyxmax)+(abs(joyxmax-joyxmin)/2);
	stickCenterY = min(joyymin,joyymax)+(abs(joyymax-joyymin)/2);

	clearServos();
	autoDomePWM = chan3Neutral;

	//delay(3000);
	//getOP();

	startupChime();
	
	//testServo();

}

void loop() {

	loopTimer = millis();

	if (loopTimer >= (lastPoll+telemetryPoll)) {
		getRSSI();
		getVCC();
		getVCA();
		lastPoll = loopTimer;
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
			//Serial.println("Got a ZB RX Packet...");
			xbee.getResponse().getZBRxResponse(rx);
			handleEvent();
			if (rx.getOption() == ZB_PACKET_ACKNOWLEDGED) {
				//Serial.println("Sender got ACK");  
			} else {
				//Serial.println("Sender did not get ACK");  
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
	//	} else {
	//		Serial.println("WTF?");  
		}
	} else if (xbee.getResponse().isError()) {
		//Serial.print("Error reading packet.  Error code: ");  
		//Serial.println(xbee.getResponse().getErrorCode());
	}

	sendTelemetry();

	autoPilot();

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
	triggerEvent = rx.getData()[7];
	byte gotSig = rx.getData()[8];

	if (gotSig > 0) {
		txSignature = rx.getData()[8];	
	}

	/*
	Serial.print(">> joyx =");Serial.print(joyx);
	Serial.print("\tjoyy =");Serial.print(joyy);
	Serial.print("\taccx =");Serial.print(accx);
	Serial.print("\taccy =");Serial.print(accy);
	Serial.print("\taccz =");Serial.print(accz);
	Serial.print("\ttriggerEvent =");Serial.println(triggerEvent);
	*/

	char* stick = "CENTER";
	if (joyx >= 133) {
		stick = "RIGHT";
	} else if (joyx <= 123) {
		stick = "LEFT";
	} else if (joyy >= 133) {
		stick = "UP";
	} else if (joyy <= 123) {
		stick = "DOWN";
	}	

	switch (triggerEvent) {

		// Nunchuk triggers first...
		
		case 252:
			Serial.println("C Button Pressed");
			// Play sound...
			if (stick == "UP") {
				// Play "happy" sound:
				randomSound(false, true, false, false);
			} else if (stick == "LEFT") {
				// Play "scared" sound:
				randomSound(false, false, true, false);
			} else if (stick == "RIGHT") {
				// Play "angry" sound:
				randomSound(false, false, false, true);
			} else if (stick == "DOWN") {
				// Send STOP command:
				stopSound();
			} else {
				randomSound(moodChill, moodHappy, moodScary, moodAngry);
			}
			break;

		case 253:
			Serial.println("Z Button Pressed");
			// Spin dome...
			if (stick == "LEFT") {
				domeSpinning = true;
				stickMapped = map(joyx, stickCenterX, joyxmin, chan3Neutral, chan3Min);
				chan3servo.write(stickMapped);
			} else if (stick == "RIGHT") {
				domeSpinning = true;
				stickMapped = map(joyx, stickCenterX, joyxmax, chan3Neutral, chan3Max);
				chan3servo.write(stickMapped);
			} else {
				domeSpinning = false;
				chan3servo.write(chan3Neutral);
			}
			break;

		case 254:
			Serial.println("Z+C Buttons Pressed");
			// Special stuff...
			if (stick == "UP") {
				// Toggle Holos:
				toggleHPs();
			} else if (stick == "DOWN") {
				// Play Leia's message:
				playSound(156);
			}
			break;

		// Any touch screen triggers?

		case 1:
			toggleHPs();
			break;
		case 2:
			//Serial.println("Toggling autopilot... ");
			//Serial.println(autoPilotEngaged);
			autoPilotEngaged = !autoPilotEngaged;
			break;
		case 3:
			moodChill = !moodChill;
			//Serial.print("chill? "); Serial.println(moodChill);
			break;
		case 4:
			moodHappy = !moodHappy;
			//Serial.print("happy? "); Serial.println(moodHappy);
			break;
		case 5:
			moodScary = !moodScary;
			//Serial.print("scary? "); Serial.println(moodScary);
			break;
		case 6:
			moodAngry = !moodAngry;
			//Serial.print("angry? "); Serial.println(moodAngry);
			break;
		case 7:
			playSound(43);
			break;
		case 8:
			playSound(150);
			break;
		case 9:
			playSound(151);
			break;
		case 10:
			playSound(152);
			break;
		case 11:
			playSound(153);
			break;
		case 12:
			playSound(154);
			break;
		case 13:
			playSound(155);
			break;
		case 14:
			playSound(156);
			break;
		case 15:
			playSound(157);
			break;
		case 16:
			playSound(158);
			break;
		case 17:
			playSound(159);
			break;
		case 18:
			playSound(160);
			break;
		case 19:
			playSound(161);
			break;
		case 20:
			playSound(162);
			break;
		case 21:
			playSound(163);
			break;
		case 22:
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

	domeSpinning = false;
	chan3servo.write(chan3Neutral);

}

void sendTelemetry() {
	unsigned long timeNow = millis();
	if (timeNow >= (lastSignal+floodControl)) {

		//Serial.print("txVCC="); Serial.print(txVCC);
		//Serial.print("\ttxVCA="); Serial.println(txVCA);
		
		payload[0] = txVCC;									// Voltage to one decimal place * 10 (3.3v = 33, 12v = 120)
		payload[1] = txVCA;									// Amperage to one decimal place * 10
		payload[2] = txRSSI;								// RSSI Value 
		payload[3] = txSignature;							// Signature of last executed signal
		payload[4] = txResponder;							// Data to send back to receiver
		payload[5] = '5';									// Future Use
		
		xbee.send(zbTx);
	
		Serial.print("Tx:");
		Serial.print("\t"); Serial.print(payload[0]);
		Serial.print("\t"); Serial.print(payload[1]);
		Serial.print("\t"); Serial.print(payload[2]);
		Serial.print("\t"); Serial.print(payload[3]);
		Serial.print("\t"); Serial.print(payload[4]);
		Serial.print("\t"); Serial.println(payload[5]);

		lastSignal = timeNow;

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
	txVCA = random(0,255);
}  

void getRSSI() {

	atRequest.setCommand(dbCmd);  
	xbee.send(atRequest);
	if (xbee.readPacket(5000)) {
		xbee.getResponse().getAtCommandResponse(atResponse);			
		if (atResponse.isOk()) {
			if (atResponse.getValueLength() > 0) {
				txRSSI = atResponse.getValue()[0];
				//Serial.print("RSSI Value: ");
				//Serial.println(txRSSI);
			} else {
				//Serial.print("Response empty");
			}
		} else {
			//Serial.print("Command failed");
		}
	}

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

/*////////////////////////////////////////////////////////////////////////////////////////////////*/
///////////////////////* Custom Droid Functions *///////////////////////////////////////////////////
/*////////////////////////////////////////////////////////////////////////////////////////////////*/

void autoPilot() {

	if (autoPilotEngaged) {

		autoLoopCount++;
		if (autoLoopCount <= autoDomeCnt) {
			if (!domeSpinning) {
				chan3servo.write(autoDomePWM);
			}
		} else {
			autoDomeCnt = 0;
			autoLoopCount = 0;
			if (!domeSpinning) {
				chan3servo.write(chan3Neutral);
			}
		}

		// Init baseInterval & nextInterval
		if (baseInterval == 0) {
			baseInterval = loopTimer;
			nextInterval = random(4000,5000);
		}
		unsigned long timerInterval = (loopTimer-baseInterval);

		int noiseSeed = random(0,1000);			// 1 in X chance of a sound playng each loop...
		if (noiseSeed == 1) {
			randomSound(moodChill, moodHappy, moodScary, moodAngry);
		}

		Serial.print(timerInterval);
		Serial.print("/");
		Serial.println(nextInterval);

		if (timerInterval >= nextInterval) {

			//Serial.print("Step: ");
			//Serial.println(autoPilotStep);

			switch(autoPilotStep) {
			
				case 1:
					//Serial.println("Step 1");

					// Twitch dome left over a few loops:
					autoDomeCnt = 4;
					autoDomePWM = 130;

					autoPilotStep++;
					nextInterval = random(500,1000);
					break;

				case 2:
					//Serial.println("Step 2");

					// Twitch dome right over 3 loops:
					autoDomeCnt = 4;
					autoDomePWM = 50;

					autoPilotStep++;
					nextInterval = random(500,1000);
					break;

				default:
					//Serial.println("Loop");
					autoPilotStep = 1;

			}

			baseInterval = 0;

		}

	}

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

void randomSound(boolean chill, boolean happy, boolean scary, boolean angry) {
	// Play "casual" sound, or random sound apprpriate for mood:
	if (chill && !happy && !scary && !angry) {
		// Casual mode:
		Serial.print("Playing casual sound...");
		randNum = random(1,41);
	} else if (!chill && happy && !scary && !angry) {
		// Happy mode:
		Serial.print("Playing happy sound...");
		randNum = random(41,83);
	} else if (!chill && !happy && scary && !angry) {
		// Scared mode:
		Serial.print("Playing cautious sound...");
		randNum = random(137,150);
	} else if (!chill && !happy && !scary && angry) {
		// Angry mode:
		Serial.print("Playing angry sound...");
		randNum = random(83,137);
	} else if (!chill && !happy && !scary && !angry) {
		// No mode selected, pick anything or else we go into an endless loop:
		randNum = random(1,150);
	} else {
		// Multiple moods active, loop for appropriate sound ID...
		Serial.print("Selecting from multiple: ");
		Serial.print(chill); Serial.print(" ");
		Serial.print(happy); Serial.print(" ");
		Serial.print(scary); Serial.print(" ");
		Serial.print(angry); Serial.println(" ");
		boolean goodSound = false;
		while (!goodSound) {
			randNum = random(1,150);
			if (((randNum>=1)&&(randNum<41)) && chill) { goodSound = true; }
			if (((randNum>=41)&&(randNum<83)) && happy) { goodSound = true; }
			if (((randNum>=83)&&(randNum<137)) && scary) { goodSound = true; }
			if (((randNum>=137)&&(randNum<=149)) && angry) { goodSound = true; }
		}
	}
	playSound(randNum);
}

void toggleHPs() {
	if (holosOn) {
		Serial.println("HPs OFF");
		chan4servo.write(80);
	} else {
		Serial.println("HPs ON");
		chan4servo.write(100);
	}
	holosOn = !holosOn;
}

void testServo() {

	// Repurposeable function for testing servo ranges...

	int i = 0;
	boolean reverse = false;

	while(1) {

		if (i > 255) { reverse = true; }
		if (i < 0) { reverse = false; }

		chan3servo.write(i);

		Serial.print("PWM Signal: ");
		Serial.println(i);
	
		if (!reverse) {
			i++;		
		} else {
			i--;
		}
		
		delay(500);
	
	}

}