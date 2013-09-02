#include <XBee.h>

int xbeebps = 19200;

XBee xbee = XBee();
XBeeResponse response = XBeeResponse();
ZBRxResponse rx = ZBRxResponse();
ModemStatusResponse msr = ModemStatusResponse();

uint8_t idCmd[] = {'I','D'};
uint8_t opCmd[] = {'O','P'};

AtCommandRequest atRequest = AtCommandRequest(opCmd);
AtCommandResponse atResponse = AtCommandResponse();

void setup() {
	
	Serial.begin(9600);									// DEBUG CODE

	Serial1.begin(xbeebps);
	xbee.setSerial(Serial1);							// Setup Xbee to use Serial1
	xbee.begin(xbeebps);								// Setup Xbee to begin at 19200

	Serial.println(" ");  
	Serial.println(" ");  
	Serial.println("Listening...");  

	delay(4000);
	getOP();

}

void loop() {

	xbee.readPacket();

	if (xbee.getResponse().isAvailable()) {
		Serial.println("Got something...");

		if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) {
			Serial.println("Got a ZB RX Packet...");
			xbee.getResponse().getZBRxResponse(rx);

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