// Required libraries
#include "variant.h"
#include <due_can.h>

//Leave defined if you use native port, comment if using programming port
#define Serial SerialUSB

#define ECU_ID 0x07E0 //also 0x07E1
//the above is the ECU bus ID

#define ECU_RESPONSE_ID1 0x07E8
#define ECU_RESPONSE_ID2 0x07E9

int testerPresentWait = 100;
int requestDataWait = 1000;

static unsigned long testerPresentLast = 0;
static unsigned long requestDataLast = 0;

int currentState = 0;
int nextState = 0;

bool noDataYet = false;

bool debugMode = false;

#define NoDataYetDelay 1000

//below are the CAN definitions for the various operations
uint16_t OBDVehicleSpeed[8] = {0x02, 0x01, 0x0D, 0x55, 0x55, 0x55, 0x55, 0x55};
uint16_t OBDEngineRPM[8] = {0x02, 0x01, 0x0C, 0x55, 0x55, 0x55, 0x55, 0x55};
uint16_t OBDPedalPosition[8] = {0x02, 0x01, 0x5A, 0x55, 0x55, 0x55, 0x55, 0x55};
uint16_t OBDOdometerData[8] = {0x02, 0x01, 0xA6, 0x55, 0x55, 0x55, 0x55, 0x55};

uint16_t OBDSupportedPIDS[8] = {0x02, 0x01, 0x00, 0x55, 0x55, 0x55, 0x55, 0x55};
uint16_t OBDTesterPresent[2] = {0x01, 0x3E};


	//********** TX REQUEST FRAME ****************
	//where OBD PID response is CANID: BYTE_0, BYTE_1, BYTE_2, BYTE_3, BYTE_4, BYTE_5, BYTE_6, BYTE_7 
	//
	//                  ----------upper payload-------       -----------------lower payload----------------
	//
	//                 | add bytes (2) | mode | PID  |  value[0] (0x55 = NA) |  value[1]  |   value[2]  |  value[3]  |   NA  |
	//
	//
	//NOTE: for transmisison requests now we are going to use the main broadcast message to speak to all ECU's 0x7DF
	//
	//********** RX RECEIVE FRAME ****************
	//
	//where OBD PID response is CANID: BYTE_0, BYTE_1, BYTE_2, BYTE_3, BYTE_4, BYTE_5, BYTE_6, BYTE_7 
	//
	//                  ----------upper payload-------       -----------------lower payload----------------
	//
	//                 | add bytes | mode & 0x40 (ack) | PID |  value[0] |  value[1]  |   value[2]  |  value[3]  |   NA  |
	//
	//NOTE: for now we are going to assume the main ECU response 0x7E8

void printFrame(CAN_FRAME &frame) {
	Serial.print("ID: 0x");
	Serial.print(frame.id, HEX);
	Serial.print(" Len: ");
	Serial.print(frame.length);
	Serial.print(" Data: 0x");
	for (int count = 0; count < frame.length; count++) {
			 Serial.print(frame.data.bytes[count], HEX);
			 Serial.print(" ");
	}
	Serial.print("\r\n");
}

void sendCANPacket(int ID, uint16_t data[]) {
	CAN_FRAME outgoing;
	outgoing.id = ID;
	outgoing.extended = false;
	outgoing.priority = 4;

	int len = sizeof(data)/sizeof(uint16_t); //set proper length
	outgoing.length = len;

	for (int i=0; i<len; i++) {
		outgoing.data.byte[i] = data[i];
	}

  	Can0.sendFrame(outgoing);
}

bool getCanData(int (*data)[8]) {
	if (Can0.available() <= 0) {
		return false;
	}

	CAN_FRAME incoming;
	Can0.read(incoming);
	if (debugMode) {
		printFrame(incoming);
	}

	int realData[8];
	for (int count = 0; count < incoming.length; count++) {
		realData[count] = int(incoming.data.bytes[count]); //convert to int
	}
	//so, there's some C++ fuckery here. basically first we have to set up a new int array with the actual bytes from CAN, converted

	int (*pointerizedData)[8] = &realData; //then, we have to turn it into a pointer so the types are compatible
	data = pointerizedData; //and then we can assign the types

	return true;
}


void changeState(int newState, bool allowNoData) { //these 2 helper functions basically make it so I can use whatever syntax I want
	changeStateReal(newState, allowNoData);
}

void changeState(int newState) {
	changeStateReal(newState, false);
}

void changeStateReal(int newState, bool allowNoData) {
	if (debugMode) {
		Serial.print("Changing state to ");
		Serial.println(newState);
	}

	currentState = newState;
	
	int data[8];
	bool isDataPresent = getCanData(&data); //does double duty of fetching data and setting bool flag
	if (!isDataPresent && !allowNoData) {
		if (debugMode) {
			Serial.println("no response from CAN bus, waiting and trying again...");
		}
		delay(NoDataYetDelay);
		noDataYet = true; //set flag to change again
    	return;
	}
	nextState = -1;


	switch(newState) {
		case 1: {
			sendCANPacket(ECU_ID, OBDTesterPresent); //send tester present signal
			nextState = 2; //when data is recieved will change state to 2
			break;
		}

		case 2: {
			OBDSupportedPIDS[2] = 0x00; //set proper request byte
			sendCANPacket(ECU_ID, OBDSupportedPIDS); //send mode 1 pid 0, request supported modes
			nextState = 3;
			break;
		}

		case 3: {
			Serial.println("bytes 4-7, supported OBD pids 0x00"); //print supported PIDs
			Serial.print(data[4]);
			Serial.print(data[5]);
			Serial.print(data[6]);
			Serial.println(data[7]);

			changeState(4, true);
			break;
		}

		case 4: {
			OBDSupportedPIDS[2] = 0x20; //set proper request byte
			sendCANPacket(ECU_ID, OBDSupportedPIDS); //Send mode 1, pid 20, request supported PIDS for latter addresses
			nextState = 5;
			break;
		}

		case 5: {
			Serial.println("bytes 4-7, supported OBD pids 0x20"); //print supported PIDs
			Serial.print(data[4]);
			Serial.print(data[5]);
			Serial.print(data[6]);
			Serial.println(data[7]);
			changeState(6, true);
			break;
		}

		case 6: {
			OBDSupportedPIDS[2] = 0x40; //set proper request byte
			sendCANPacket(ECU_ID, OBDSupportedPIDS); //Send mode 1, pid 40, request supported PIDS for latter addresses
			nextState = 7;
			break;
		}

		case 7: {
			Serial.println("bytes 4-7, supported OBD pids 0x40"); //print supported PIDs
			Serial.print(data[4]);
			Serial.print(data[5]);
			Serial.print(data[6]);
			Serial.println(data[7]);
			changeState(8, true);
			break;
		}

		case 8: {
			OBDSupportedPIDS[2] = 0x60; //set proper request byte
			sendCANPacket(ECU_ID, OBDSupportedPIDS); //Send mode 1, pid 60, request supported PIDS for latter addresses
			nextState = 9;
			break;
		}

		case 9: {
			Serial.println("bytes 4-7, supported OBD pids 0x60"); //print supported PIDs
			Serial.print(data[4]);
			Serial.print(data[5]);
			Serial.print(data[6]);
			Serial.println(data[7]);
			changeState(10, true);
			break;
		}

		case 10: {
			OBDSupportedPIDS[2] = 0x80; //set proper request byte
			sendCANPacket(ECU_ID, OBDSupportedPIDS); //Send mode 1, pid 80, request supported PIDS for latter addresses
			nextState = 11;
			break;
		}

		case 11: {
			Serial.println("bytes 4-7, supported OBD pids 0x80"); //print supported PIDs
			Serial.print(data[4]);
			Serial.print(data[5]);
			Serial.print(data[6]);
			Serial.println(data[7]);
			changeState(12, true);
			break;
		}

		case 12: {
			Serial.println("getting odo data");
			sendCANPacket(ECU_ID, OBDOdometerData); //ask for odometer data
			nextState = 13;
			break;
		}

		case 13: {
			Serial.println("odometer data");
			Serial.print(data[4]);
			Serial.print(data[5]);
			Serial.print(data[6]);
			Serial.println(data[7]);
			delay(5000);
			changeState(14, true);
			break;
		}

		//ACTUAL PID CHECKER
		case 14: {
			sendCANPacket(ECU_ID, OBDPedalPosition); //ask for relative pedal position (exciting)
			nextState = 15;
			break;
		}

		case 15: {
			Serial.println("acc pedal position");
			Serial.print((100/255)*data[4]);
			Serial.println("%");
			changeState(16, true); //loop it here
			break;
		}

		case 16: {
			sendCANPacket(ECU_ID, OBDEngineRPM); //ask for engine rpm
			nextState = 17;
			break;
		}

		case 17: {
			long rpm = ((256*data[4])+data[5])/4;

			if (rpm > 10) {
				Serial.println("engine rpm");
				Serial.print(rpm);
				Serial.println("rpm");
			}
			changeState(16, true);
			break;
		}

		case 18: {
			sendCANPacket(ECU_ID, OBDVehicleSpeed); //ask for vehicle speed
			nextState = 19;
			break;
		}

		case 19: {
			Serial.println("vehicle speed");
			Serial.print(data[4]);
			Serial.println("km/h");
			delay(500);
			changeState(14, true);
			break;
		}
	}
}

void loop(){

	if (Can0.available() > 0) {
		if (nextState > 0) {
			changeState(nextState);
		}
	} else if (noDataYet) { //retry sending message
		if (nextState > 0) {
			changeState(nextState);
		} else {
			changeState(currentState);
		}
	}
	
}

void setup()
{

	Serial.begin(115200);
	
	// Initialize CAN0 and CAN1, Set the proper baud rates here
	Can0.begin(CAN_BPS_500K);

	//setup mailboxes
	int filter;
	//extended
	for (filter = 0; filter < 3; filter++) {
	Can0.setRXFilter(filter, 0, 0, true);
	}  
	//standard
	for (int filter = 3; filter < 7; filter++) {
	Can0.setRXFilter(filter, 0, 0, false);
	}

	Serial.println("waiting to send data for 5000ms");
	delay(3000);
	changeState(1, true);
}


