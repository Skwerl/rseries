#include <XBee.h>

XBee xbee = XBee();
XBeeResponse response = XBeeResponse();
ZBRxResponse rx = ZBRxResponse();
ModemStatusResponse msr = ModemStatusResponse();

uint8_t idCmd[] = {'I','D'};
uint8_t opCmd[] = {'O','P'};

AtCommandRequest atRequest = AtCommandRequest(opCmd);
AtCommandResponse atResponse = AtCommandResponse();

byte joyx, joyy, accx, accy, accz, zbut, cbut;
int triggerEvent;
int randNum;

void setup() {
	
	Serial.begin(9600);
	Serial1.begin(19200);
	Serial2.begin(38400);

	xbee.setSerial(Serial1);							// Setup Xbee to use Serial1
	xbee.begin(19200);									// Setup Xbee to begin at 19200

	Serial.println(" ");  
	Serial.println(" ");  
	Serial.println("Listening...");  

	playSound(164);

	delay(3000);
	getOP();

}

void loop() {

	xbee.readPacket();

	if (xbee.getResponse().isAvailable()) {
		//Serial.println("Got something...");

		if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) {
			//Serial.println("Got a ZB RX Packet...");
			xbee.getResponse().getZBRxResponse(rx);
			
			int joyx = rx.getData()[0];    // Left Right
			int joyy = rx.getData()[1];    // Fwd Back
			int accx = rx.getData()[2];    // Dome
			//    int accy = rx.getData()[3];  // Nunchuk Y Acceleramator 
			//    int accz = rx.getData()[4];  // Nunchuk Z Acceleramator
			triggerEvent = rx.getData()[7];// TriggerEvent
			//    int futureEvent = rx.getData()[8]; // Future Event Payload
			
			Serial.print(">> joyx =");Serial.print(joyx);								// DEBUG CODE
			Serial.print("\tjoyy =");Serial.print(joyy);								// DEBUG CODE
			Serial.print("\taccx =");Serial.print(accx);								// DEBUG CODE
			Serial.print("\taccy =");Serial.print(accy);								// DEBUG CODE
			Serial.print("\taccz =");Serial.print(accz);								// DEBUG CODE
			Serial.print("\ttriggerEvent =");Serial.println(triggerEvent);				// DEBUG CODE

			if (triggerEvent == 102) {
				Serial.println("C Button Pressed");
				if (joyy <= 70) {
					// Joystick up, play "happy" sound:
					randNum = random(41,83);
					playSound(randNum);
				} else if (joyx >= 110) {
					// Joystick left, play "scared" sound:
					randNum = random(83,137);
					playSound(randNum);
				} else if (joyx <= 70) {
					// Joystick right, play "angry" sound:
					randNum = random(137,150);
					playSound(randNum);
				} else {
					// Joystick down or neutral, play "casual" sound:
					randNum = random(1,41);
					playSound(randNum);
				}
			}

			if (triggerEvent == 101) {
				Serial.println("Z Button Pressed");
				// Play cantina song:
				playSound(152);				
			}

			if (triggerEvent == 103) {
				Serial.println("Z+C Buttons Pressed");
				// Play Leia's message:
				playSound(156);				
			}
			
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
		} else {
			Serial.println("WTF?");  
		}
	} else if (xbee.getResponse().isError()) {
		Serial.print("Error reading packet.  Error code: ");  
		Serial.println(xbee.getResponse().getErrorCode());
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

void playSound(int sample) {

	Serial.print("Playing sample #");
	Serial.println(sample);

	Serial2.write('t');
	Serial2.write(sample);

}