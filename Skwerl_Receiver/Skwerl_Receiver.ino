#include <XBee.h>
#include <MP3Trigger.h>

int xbeebps = 19200;
int mp3bps = 38400;

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

MP3Trigger trigger;

void setup() {
	
	Serial.begin(9600);
	Serial1.begin(xbeebps);
	Serial2.begin(mp3bps);

	xbee.setSerial(Serial1);							// Setup Xbee to use Serial1
	xbee.begin(xbeebps);								// Setup Xbee to begin at 19200

	trigger.setup(&Serial2);							// Setup MP3Trigger to use Serial2

	Serial.println(" ");  
	Serial.println(" ");  
	Serial.println("Listening...");  
	trigger.setLooping(true,1);							// Just start looping Track 1?

	delay(4000);
	getOP();

}

void loop() {

	xbee.readPacket();
	trigger.update();

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

			if (triggerEvent == 101) {
				Serial.println("Z Button Pressed");
			}
			if (triggerEvent == 102) {
				trigger.setLooping(false,1);
				Serial2.write('T');
				Serial2.write(2);
				Serial.println("C Button Pressed");
			}
			if (triggerEvent == 103) {
				Serial.println("Z+C Buttons Pressed");
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