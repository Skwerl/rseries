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

unsigned long loopTimer = 0;

byte joyx, joyy, accx, accy, accz, zbut, cbut;
int triggerEvent;

int randNum;
int mp3Byte = 0;
boolean mp3Playing = false;

boolean holosOn = false;

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
int stickMirror = 0;
int stickCenterX = 128;
int stickCenterY = 128;

int controlMode = 0;

/*////////////////////////////////////////////////////////////////////////////////////////////////*/
///////////////////////* PWM & Servo Configuration *////////////////////////////////////////////////
/*////////////////////////////////////////////////////////////////////////////////////////////////*/

Servo chan1servo;
Servo chan2servo;
Servo chan3servo;
Servo chan4servo;
Servo chan5servo;
Servo chan6servo;

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
int chan1Min = 50;					// Channel 1 Min - Forward & Reverse Speed 
int chan1Max = 150;					// Channel 1 Max - Forward & Reverse Speed
int chan2Min = 50;					// Channel 2 Min - Left/Right 
int chan2Max = 150;					// Channel 2 Max - Left/Right
int chan3Min = 120;					// Channel 3 Min - Dome Rotation LEFT 
int chan3Max = 56;					// Channel 3 Max - Dome Rotation RIGHT
int chan4Min = 120;					// Channel 4 Min - Holo Movement Up/Down
int chan4Max = 50;					// Channel 4 Max - Holo Movement Up/Down
int chan5Min = 120;					// Channel 5 Min - Holo Movement Left/Right
int chan5Max = 50;					// Channel 5 Max - Holo Movement Left/Right

// Weirdness Corrections
int chan1correct = 0;
int chan2correct = 0;
int chan3correct = 0;
int chan4correct = 0;
int chan5correct = 0;

// Neutral Adjustments 
int chan1Neutral = min(chan1Min,chan1Max)+(abs(chan1Max-chan1Min)/2)+chan1correct;
int chan2Neutral = min(chan2Min,chan2Max)+(abs(chan2Max-chan2Min)/2)+chan2correct;
int chan3Neutral = min(chan3Min,chan3Max)+(abs(chan3Max-chan3Min)/2)+chan3correct;
int chan4Neutral = min(chan4Min,chan4Max)+(abs(chan4Max-chan4Min)/2)+chan4correct;
int chan5Neutral = min(chan5Min,chan5Max)+(abs(chan5Max-chan5Min)/2)+chan5correct;

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

	chan1servo.attach(servo1Pin);  // Foot Motors Forward/Back
	chan2servo.attach(servo2Pin);  // Foot Motors Left/Right
	chan3servo.attach(servo3Pin);  // Dome Motor 
	chan4servo.attach(servo4Pin);  // Holo Up/Down 
	chan5servo.attach(servo5Pin);  // Holo Left/Right
	chan6servo.attach(servo6Pin);  // HPs Switch 

	stickCenterX = min(joyxmin,joyxmax)+(abs(joyxmax-joyxmin)/2);
	stickCenterY = min(joyymin,joyymax)+(abs(joyymax-joyymin)/2);

	clearServos();

	//delay(3000);
	//getOP();

	startupChime();
	
	//testServo();

}

void loop() {

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
	controlMode = rx.getData()[8];

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
				stickMapped = map(joyx, stickCenterX, joyxmin, chan3Neutral, chan3Min);
				chan3servo.write(stickMapped);
			} else if (stick == "RIGHT") {
				stickMapped = map(joyx, stickCenterX, joyxmax, chan3Neutral, chan3Max);
				chan3servo.write(stickMapped);
			} else {
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
				// Cycle Mode:
				cycleMode();
			}
			break;

		case 0:

			if (controlMode == 1) {

				// Park; Holo Control

				if (stick == "UP") {
					stickMapped = map(joyy, stickCenterY, joyymin, chan4Neutral, chan4Min);
					chan4servo.write(stickMapped);
				} else if (stick == "DOWN") {
					stickMapped = map(joyy, stickCenterY, joyymax, chan4Neutral, chan4Max);
					chan4servo.write(stickMapped);
				}
	
				if (stick == "LEFT") {
					stickMapped = map(joyx, stickCenterX, joyxmin, chan5Neutral, chan5Min);
					chan5servo.write(stickMapped);
				} else if (stick == "RIGHT") {
					stickMapped = map(joyx, stickCenterX, joyxmax, chan5Neutral, chan5Max);
					chan5servo.write(stickMapped);
				}
	
				if (stick != "UP" && stick != "DOWN" && stick != "LEFT" && stick != "RIGHT") {
					//Serial.println("Clearing servos...");
					clearServos();
				}
			
			
			}

			if (controlMode == 2) {

				// Drive; Motor Control

				if (stick == "UP") {
	
					stickMapped = map(joyy, stickCenterY, joyymax, chan1Neutral, chan1Max);
					stickMirror = map(joyy, stickCenterY, joyymax, chan2Neutral, chan2Max);
	
					chan1servo.write(stickMapped);
					chan2servo.write(stickMirror);
	
				} else if (stick == "DOWN") {
	
					stickMapped = map(joyy, stickCenterY, joyymin, chan1Neutral, chan1Min);
					stickMirror = map(joyy, stickCenterY, joyymin, chan2Neutral, chan2Min);
	
					chan1servo.write(stickMapped);
					chan2servo.write(stickMirror);
	
				} else if (stick == "LEFT") {
	
					stickMapped = map(joyx, stickCenterX, joyxmin, chan1Neutral, chan1Max);
					stickMirror = map(joyx, stickCenterX, joyxmin, chan2Neutral, chan2Min);
	
					chan1servo.write(stickMapped);
					chan2servo.write(stickMirror);
	
				} else if (stick == "RIGHT") {
	
					stickMapped = map(joyx, stickCenterX, joyxmax, chan1Neutral, chan1Min);
					stickMirror = map(joyx, stickCenterX, joyxmax, chan2Neutral, chan2Max);
	
					chan1servo.write(stickMapped);
					chan2servo.write(stickMirror);
	
				} else {
	
					chan1servo.write(chan1Neutral);
					chan2servo.write(chan2Neutral);
	
				}

			}

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

	chan1servo.write(chan1Neutral);
	chan2servo.write(chan2Neutral);	
	chan3servo.write(chan3Neutral);
	chan4servo.write(chan4Neutral);
	chan5servo.write(chan5Neutral);

}

void getRSSI() {

	atRequest.setCommand(dbCmd);  
	xbee.send(atRequest);
	if (xbee.readPacket(5000)) {
		xbee.getResponse().getAtCommandResponse(atResponse);			
		if (atResponse.isOk()) {
			if (atResponse.getValueLength() > 0) {
				//txRSSI = atResponse.getValue()[0];
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
		chan6servo.write(80);
	} else {
		Serial.println("HPs ON");
		chan6servo.write(100);
	}
	holosOn = !holosOn;
}

void cycleMode() {
	// Logic happens in controller...
	Serial.print("Control Mode:"); Serial.println(controlMode);
	playSound(15);
}

void testServo() {

	// Repurposeable function for testing servo ranges...

	int i = 0;
	boolean reverse = false;

	while(1) {

		if (i > 255) { reverse = true; }
		if (i < 0) { reverse = false; }

		chan1servo.write(i);
		//chan2servo.write(i);

		Serial.print("PWM Signal: ");
		Serial.println(i);
	
		if (!reverse) {
			i++;		
		} else {
			i--;
		}
		
		delay(200);
	
	}

}