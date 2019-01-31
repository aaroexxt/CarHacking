//Reads all traffic on CAN0 and forwards it to CAN1 (and in the reverse direction) but modifies some frames first.
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

bool requestedData = false;

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



void sendTesterPresent()
{
	Serial.println("sending tester present");
	CAN_FRAME outgoing;
	outgoing.id = ECU_ID;
	outgoing.extended = false;
	outgoing.priority = 4; //0-15 lower is higher priority
	outgoing.length = 2;
	
	outgoing.data.byte[0] = 0x01;
	outgoing.data.byte[1] = 0x3E;
	Can0.sendFrame(outgoing);
}
void requestEngineData()
{
	
	
	Serial.println("sending engine request");
	CAN_FRAME outgoing;
	outgoing.id = ECU_ID;
	outgoing.extended = false;
	outgoing.priority = 4; //0-15 lower is higher priority
	outgoing.length = 8;
	
	outgoing.data.byte[0] = 0x02;
	outgoing.data.byte[1] = 0x01;
	outgoing.data.byte[2] = 0x00;
	outgoing.data.byte[3] = 0x55;
	outgoing.data.byte[4] = 0x55;
	outgoing.data.byte[5] = 0x55;
	outgoing.data.byte[6] = 0x55;
	outgoing.data.byte[7] = 0x55;

	//from this message, with some reverse engineering bytes 4-7 are what we want

	Can0.sendFrame(outgoing);
}

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
}

bool getCanData(int (*data)[8]) {
	if (Can0.available() <= 0) {
		return false;
	}

	CAN_FRAME incoming;
	Can0.read(incoming);
	printFrame(incoming);

	int realData[8];
	for (int count = 0; count < incoming.length; count++) {
		realData[count] = int(incoming.data.bytes[count]); //convert to int
	}
	//so, there's some C++ fuckery here. basically first we have to set up a new int array with the actual bytes from CAN, converted

	int (*pointerizedData)[8] = &realData; //then, we have to turn it into a pointer so the types are compatible
	data = pointerizedData; //and then we can assign the types

	return true;
}

void changeState(int newState) {
	Serial.print("Changing state to ");
	Serial.println(newState);

	currentState = newState;
	nextState = -1;
	
	int data[8];
	bool isDataPresent = getCanData(&data); //does double duty of fetching data and setting bool flag
	if (!isDataPresent) {
		Serial.println("no response from CAN bus, waiting and trying again...");
	}


	switch(newState) {
		case 1:
			sendCANPacket(ECU_ID, [0x01, 0x3E]); //send tester present signal
			nextState = 2; //when data is recieved will change state to 2
		case 2:
			sendCANPacket(ECU_ID, [0x02, 0x01, 0x00, 0x55, 0x55, 0x55, 0x55, 0x55]); //send mode 1 pid 0, request supported modes
			nextState = 3;
		case 3:
			Serial.println("bytes 4-7, supported OBD pids 0x00"); //print supported PIDs
			Serial.print(data[4]);
			Serial.print(data[5]);
			Serial.print(data[6]);
			Serial.println(data[7]);
			changeState(4);
		case 4:
			sendCANPacket(ECU_ID, [0x02, 0x01, 0x20, 0x55, 0x55, 0x55, 0x55, 0x55]); //Send mode 1, pid 20, request supported PIDS for latter addresses
			nextState = 5;
		case 5:
			Serial.println("bytes 4-7, supported OBD pids 0x20"); //print supported PIDs
			Serial.print(data[4]);
			Serial.print(data[5]);
			Serial.print(data[6]);
			Serial.println(data[7]);
			changeState(6);
		case 6:
			sendCANPacket(ECU_ID, [0x02, 0x01, 0x40, 0x55, 0x55, 0x55, 0x55, 0x55]); //Send mode 1, pid 20, request supported PIDS for latter addresses
			nextState = 7;
		case 7:
			Serial.println("bytes 4-7, supported OBD pids 0x40"); //print supported PIDs
			Serial.print(data[4]);
			Serial.print(data[5]);
			Serial.print(data[6]);
			Serial.println(data[7]);
			changeState(8);
		case 8:
			sendCANPacket(ECU_ID, [0x02, 0x01, 0x5A, 0x55, 0x55, 0x55, 0x55, 0x55]); //ask for relative pedal position (exciting)
			nextState = 9;
		case 9:
			Serial.println("acc pedal position");
			Serial.println((100/255)*data[4]);
			delay(100);
			changeState(8);
	}
}

void loop(){

	if (Can0.available() > 0) {
		Serial.println("rcv in loop");
		if (nextState > 0) {
			changeState(nextState);
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
	delay(5000);
	changeState(1);
}

