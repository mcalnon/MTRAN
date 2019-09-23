/***********************************************************************/
/*                                                                     */
/*  FILE        :mark.c        	                                       */
/*  DATE        :June, 30, 2009                                    	   */
/*  DESCRIPTION :Autonomous Docking Program by Mark Calnon             */
/*  CPU TYPE    :SH7047                                                */
/*                                                                     */
/***********************************************************************/
#include  <stdio.h>
#include  <string.h>
#include  <stdlib.h>
#include  <machine.h>
#include  "iodefine.h"
#include  "Api.h"
#include  "common.h"
#include  "h8.h"
#include  "h8IR.h"
#include  "control.h"
#include  "initialConf.h"

#include  "mark.h"
#include  "markPF.h"

//
// Global Variables
//
unsigned char BTMode = BT_MODE_COMMAND;		// Current Bluetooth Mode (Used in CheckBTMessage when forwarding incoming BT messages)
unsigned char CRNL[3] = "\r\n";				// Carriage Return & Newline
unsigned long BTAddress[11] =				// All Bluetooth Addresses (For Use with the CAN Command "BT_AT_SPPCONNECT")
{
	0,						// Node  40
	0x00043E05037E, 		// Node  41
	0,						// Node  42
	0,						// Node  43
	0,						// Node  44
	0x00043E050430,			// Node  45
	0x00043E050B50,			// Node  46
	0x00043E050257, 		// Node  47
	0,						// Node  48
	0,						// Node  49
	0x00043E05056F			// Node  50
};

unsigned char InhibitDockingRetry = 0;

extern struct {
	unsigned char mode;
	unsigned char face;
	unsigned char dark, bright;
} irDetect;

//
// Callback functions
//

// Executes a CAN Message
void ExecuteCANmsgMark(unsigned char *RingData)
{
	switch (RingData[ComArgment])
	{
		//
		// Bluetooth AT-ZV (command mode) commands
		//
		
		case BT_AT_ESCAPE: // Enters command mode
			BtAtEscape();
			break;
			
		case BT_AT_VERSION: // Displays the Zerial interface version number
			BtAtVersion();
			break;
			
		case BT_AT_DISCOVERY: // Finds and displays all available BT devices
			BtAtDiscovery();
			break;
			
		case BT_AT_SPPCONNECT: // Connects to an available BT device (by Node ID)
			BtAtSppConnect(RingData[Message1]);
			break;
			
		case BT_AT_SPPDISCONNECT: // Disconnects from the current BT device
			BtAtSppDisconnect();
			break;
		
		case BT_AT_BYPASS: // Enters bypass mode
			BtAtBypass();
			break;
			
		//
		// Helper commands
		//
		
		// Output commands
		case SEND_TO_MONITOR:
			SendToMonitor(&RingData[Command]);
			break;
		
		// LED commands
		case TURN_ON_LINK_LED: // Turns on the main link LED (green)
			lightLED(L_GREEN);
			break;
			
		case TURN_OFF_LINK_LED: // Turns of the main link LED
			lightLED(L_DARK);
			break;
		
		// IR LED commands
		case TURN_ON_ALL_IR_LEDS: // Turns on all active and passive IR LEDs for the 6 primary faces
			TurnOnAllIRLeds();
			break;
				
		case TURN_ON_ALL_ACTIVE_IR_LEDS: // Turns on all active IR LEDs for the 3 primary faces
			TurnOnAllActiveIRLeds();
			break;
				
		case TURN_ON_ALL_PASSIVE_IR_LEDS: // Turns on all passive IR LEDs for the 3 primary faces
			TurnOnAllPassiveIRLeds();
			break;
				
		case TURN_ON_ONE_ACTIVE_IR_LED: // Turns on one active IR LED (0-7)
			TurnOnOneActiveIRLed(RingData[Message1]);
			break;
				
		case TURN_ON_ONE_PASSIVE_IR_LED: // Turns on one passive IR LED (1,2,4,6-7)
			TurnOnOnePassiveIRLed(RingData[Message1]);
			break;
			
		case TURN_OFF_ALL_IR_LEDS: // Turns off all active and passive IR LEDs for all faces
			TurnOffAllIRLeds();
			break;
			
		// IR Sensor commands
		case DISPLAY_ONE_IR_ANYWHERE:
			DisplayOneIR(RingData[Message1], RingData[Message2], RingData[Message3], TRUE);
			break;
			
		case DISPLAY_ONE_IR:
			DisplayOneIR(RingData[Message1], RingData[Message2], RingData[Message3], FALSE);
			break;
			
		case DISPLAY_ALL_IR:
			DisplayAllIR(RingData[Message1], RingData[Message2]);
			break;
			
		// Movement commands
		case START_LOCOMOTION:
			StartLocomotion();
			break;
			
		case STOP_LOCOMOTION:
			StopLocomotion();
			break;
			
		case RESUME_LOCOMOTION:
			ResumeLocomotion();
			break;
			
		case ASSUME_THE_POSITION:
			AssumeThePosition();
			break;
			
		//
		// IR Data Collection Test:
		//		(Position Walker and Snake Nodes)
		//		(Initialize all Commander (A), Walker (B), and Snake (C) Node IDs)
		//		0.0 Turn OFF Snake Node's Face 1 IR LEDs
		//		0.1 Measure and display all Walker Node IRs
		//		1.0 Turn ON Snake Node's Face 1 Left IR LED
		//		1.1 Measure and display all Walker Node IRs
		//		2.0 Turn ON Snake Node's Face 1 Right IR LED
		//		2.1 Measure and display all Walker Node IRs
		//		3.0 Turn ON Snake Node's Face 1 All IR LEDs
		//		3.1 Measure and display all Walker Node IRs
		//
		
		case IR_DATA_TEST_NODEA_INIT_WALKER_NODE_IDS: // Initialize Walker Node IDs
			IRDataTest_NodeA_InitWalkerNodeIds(RingData[Message1], 
				RingData[Message2], RingData[Message3], RingData[Message4]);
			break;
	
		case IR_DATA_TEST_NODEA_INIT_SNAKE_NODE_ID: // Initialize Snake Node ID
			IRDataTest_NodeA_InitSnakeNodeId(RingData[Message1]);
			break;
	
		case IR_DATA_TEST_NODEA_INIT_COMMANDER_NODE_ID: // Initialize Commander Node ID (SND)
			IRDataTest_NodeA_InitCommanderNodeId(RingData[Message1]);
			break;
			
		case IR_DATA_TEST_ALL_NODES_INIT_COMMANDER_NODE_ID: // Initialize Commander Node ID (RCV)
			IRDataTest_AllNodes_InitCommanderNodeId(RingData[Message1]);
			break;

		case IR_DATA_TEST_NODEA_START_TEST: // Begin collecting data
			IRDataTest_NodeA_StartTest();
			break;
			
		case IR_DATA_TEST_NODEC_TURN_ON_LED: // Turn on/off Snake Node's Face 1 IR LED (Message1: 0 = Off, 1 = Left, 2 = Right, 3 = All) (RCV)
			IRDataTest_NodeC_TurnOnLed(RingData[Message1]);
			break;

		case IR_DATA_TEST_NODEA_TURN_ON_LED_CONFIRM: // Turn on/off Snake Node's Face 1 IR LED (CNF)
			IRDataTest_NodeA_TurnOnLedConfirm(RingData[Message1]);
			break;
			
		case IR_DATA_TEST_NODEB_DISPLAY_ALL_IR: // Measure and display all Walker Nodes' IRs (SND)
			IRDataTest_NodeB_DisplayAllIR(RingData[Message1], RingData[Message2]);
			break;
			
		case IR_DATA_TEST_NODEA_DISPLAY_ALL_IR_CONFIRM: // Measure and display all Walker Nodes' IRs (CNF)
			IRDataTest_NodeA_DisplayAllIRConfirm(RingData[Message1], RingData[Message2]);
			break;
			
		//
		// IR Docking Test:
		//		(Position Walker and Snake Nodes)
		//		(Initialize all Local IDs and Commander ID)
		//		0.0 Calculate Maximum Ambient IR
		//		1.0 Search until IR LED is found (while avoiding obstacles)
		//		1.1 Walk toward IR LED until value is maximized
		//		1.2 Re-orient docking face to maximized IR LED 
		//		2.0 Collaborate with external module to read IR values
		//		2.1 Use particle filter to correct misalignments
		//		2.2 Dock with external module
		//
		
		// Initialization
		case IR_DOCKING_TEST_NODEA_COMMANDER_START_TEST:
			IRDockingTest_NodeACommander_StartTest(RingData[Message2], RingData[Message1]);
			break;
			
		case IR_DOCKING_TEST_NODEA_INIT_COMMANDER_NODE_ID:
			IRDockingTest_NodeA_InitCommanderNodeId(RingData[Message1]);
			break;
			
		case IR_DOCKING_TEST_NODEA_INIT_NODE_B_ID:
			IRDockingTest_NodeA_InitNodeBID(RingData[Message1]);
			break;
			
		case IR_DOCKING_TEST_NODEA_COMMANDER_IR_AMBIENT_MAX_VALUE_SEND:
			IRDockingTest_NodeACommander_IRAmbientMaxValue_Send();
			break;
			
		case IR_DOCKING_TEST_NODEA_IR_AMBIENT_MAX_VALUE_SEND:
			IRDockingTest_NodeA_IRAmbientMaxValue_Send();
			break;
			
		case IR_DOCKING_TEST_NODEA_COMMANDER_IR_AMBIENT_MAX_VALUE_RECEIVE:
			IRDockingTest_NodeACommander_IRAmbientMaxValue_Receive(RingData[Message1]);
			break;
			
		case IR_DOCKING_TEST_NODEA_IR_AMBIENT_MAX_VALUE_RECEIVE:
			IRDockingTest_NodeA_IRAmbientMaxValue_Receive(RingData[Message1]);
			break;
			
		case IR_DOCKING_TEST_NODEA_DOCKING_IR_SEARCH_MODE_SEND:
			IRDockingTest_NodeADocking_IRSearchMode_Send();
			break;
			
		case IR_DOCKING_TEST_NODEA_IR_SEARCH_MODE_RECEIVE:
			IRDockingTest_NodeA_IRSearchMode_Receive();
			break;
			
		case IR_DOCKING_TEST_NODEA_COMMANDER_IR_DOCKING_MODE_SEND:
			IRDockingTest_NodeACommander_IRDockingMode_Send(RingData[Message1]);
			break;
			
		case IR_DOCKING_TEST_NODEA_IR_DOCKING_MODE_RECEIVE:
			IRDockingTest_NodeA_IRDockingMode_Receive(RingData[Message1]);
			break;
			
		case IR_DOCKING_TEST_NODEA_DOCKING_IR_DOCKING_COMPLETE_MODE_SEND:
			IRDockingTest_NodeADocking_IRDockingCompleteMode_Send();
			break;
			
		case IR_DOCKING_TEST_NODEA_IR_DOCKING_COMPLETE_MODE_RECEIVE:
			IRDockingTest_NodeA_IRDockingCompleteMode_Receive();
			break;
		
		// Search Mode
		case IR_DOCKING_TEST_NODEA_IR_LED_MAX_VALUE_SEND:
			IRDockingTest_NodeA_IRLedMaxValueSend(RingData[Message1], RingData[Message2]);
			break;
			
		case IR_DOCKING_TEST_NODEA_COMMANDER_IR_LED_MAX_VALUE_RECEIVE:
			IRDockingTest_NodeACommander_IRLedMaxValueReceive(RingData[Message1], RingData[Message2], RingData[Message3]);
			break;
			
		// Docking Mode
		case IR_DOCKING_TEST_NODEA_DOCKING_INIT_IR_COLLABORATION_SEND:
			IRDockingTest_NodeADocking_InitIRCollaboration_Send();
			break;
			
		case IR_DOCKING_TEST_NODEA_INIT_IR_COLLABORATION_SEND:
			IRDockingTest_NodeA_InitIRCollaboration_Send(RingData[Message1]);
			break;
			
		case IR_DOCKING_TEST_NODEA_DOCKING_INIT_IR_COLLABORATION_RECEIVE:
			IRDockingTest_NodeADocking_InitIRCollaboration_Receive();
			break;
			
		case IR_DOCKING_TEST_NODEA_DOCKING_IR_COLLABORATION_SEND:
			IRDockingTest_NodeADocking_IRCollaboration_Send(RingData[Message1]);
			break;
			
		case IR_DOCKING_TEST_NODEB_IR_COLLABORATION_RECEIVE:
			IRDockingTest_NodeB_IRCollaboration_Receive(RingData[Message1], RingData[Message2]);
			break;
			
		case IR_DOCKING_TEST_NODEA_DOCKING_IR_COLLABORATION_RECEIVE:
			IRDockingTest_NodeADocking_IRCollaboration_Receive(RingData[Message1], RingData[Message2]);
			break;
			
		case IR_DOCKING_TEST_NODEA_DOCKING_UPDATE_LOCOMOTION:
			IRDockingTest_NodeADocking_UpdateLocomotion();
			break;
		
		//
		// Motion Model Test:
		//
		
		case 252: // Resume walking (and timing)
			MotionModelTest_NodeA_Test(TRUE);

		case 251: // Stop walking (and timing)
			MotionModelTest_NodeA_Test(FALSE);
			break;
		
		//
		// DEBUG
		//
		
		case 253:  
		{
			/*	timing tests:
			void PoseParticlesIteration(unsigned char loc_direction, unsigned long elapsed_time);
			*/
			
			char buffer[32];
			unsigned long start, elapsed;
			
			// reset random number generator
			srand(RingData[Message1]);
			
			// time init
			start = GetTickCount();
			InitializePoseParticles();
			elapsed = GetTickCount() - start;
			
			sprintf(buffer, "Init%d = %ums", NUM_POSE_PARTICLES, elapsed);
			sendStringToMonitor(buffer);
			
			// time predict
			start = GetTickCount();
			PredictPoseParticles(FORWARD_DIRECTION, 2000);
			elapsed = GetTickCount() - start;
			
			sprintf(buffer, "Predi = %ums", elapsed);
			sendStringToMonitor(buffer);
			
			// time update
			start = GetTickCount();
			UpdatePoseParticles();
			elapsed = GetTickCount() - start;
			
			sprintf(buffer, "Updat = %ums", elapsed);
			sendStringToMonitor(buffer);
			
			// time best pose
			start = GetTickCount();
			EstimateBestPose();
			elapsed = GetTickCount() - start;
			
			sprintf(buffer, "Estmt = %ums", elapsed);
			sendStringToMonitor(buffer);
			
			// time best pose mean
			start = GetTickCount();
			EstimateBestPoseMean();
			elapsed = GetTickCount() - start;
			
			sprintf(buffer, "Estmt = %ums", elapsed);
			sendStringToMonitor(buffer);
			
			// time resample
			start = GetTickCount();
			ResamplePoseParticles();
			elapsed = GetTickCount() - start;
			
			sprintf(buffer, "Resamp = %ums", elapsed);
			sendStringToMonitor(buffer);
		}
		break;
	}
}

// Processes a Bluetooth AT-ZV response (command mode)
void ProcessBtAtResponse(unsigned char *ATResponse)
{
	char ResponseBuffer[32];
	
	// Truncate Carriage Return & Newline
	char *newline = strstr(ATResponse, CRNL);
	if (newline != 0) *newline = 0;
	
	/*
	// Debug AT-ZV Response 
	sprintf(ResponseBuffer, "Rsp%d", strlen(ATResponse));
	SendToMonitor(ResponseBuffer);
	*/
	
	// Retreive AT-ZV Response
	sscanf(ATResponse, "AT-ZV %s", ResponseBuffer);
	
	//
	// Process AT-ZV Responses
	//
	
	// "^#^$^%" (Escape Sequence) Response
	if (strcmp(ResponseBuffer, "-CommandMode-") == 0)
	{
		// Set Bluetooth Mode to Command
		BTMode = BT_MODE_COMMAND;
		
		// Debug Command Mode (Display to Screen)
		SendToMonitor("CmdMod");
	}
	
	// "AT+ZV Version" Response
	else if (strcmp(ResponseBuffer, "ZerialVer") == 0) 
	{
		int VersionNumberBig, VersionNumberSmall;
		
		// Retreive Zerial Version Number
		sscanf(ATResponse, "AT-ZV ZerialVer %d.%d", &VersionNumberBig, &VersionNumberSmall);
		
		// Debug Version Number (Display to Screen)
		sprintf(ResponseBuffer, "Ver%d.%d", VersionNumberBig, VersionNumberSmall);
		SendToMonitor(ResponseBuffer);
	}
	
	// "AT+ZV Discovery" Response (1)
	else if (strcmp(ResponseBuffer, "InqPending") == 0)
	{
		// Debug Discovery (Display to Screen)
		SendToMonitor("InqPnd");
	}
	
	// "AT+ZV Discovery" Response (2)
	else if (strcmp(ResponseBuffer, "DiscoveryPending") == 0)
	{
		int DeviceCount;
		
		// Retreive Number of Discovered Devices
		sscanf(ATResponse, "AT-ZV DiscoveryPending %d", &DeviceCount);
		
		// Debug Discovery (Display to Screen)
		SendToMonitor("DscPnd");
		sprintf(ResponseBuffer, "DCnt%d", DeviceCount);
		SendToMonitor(ResponseBuffer);
	}
	
	// "AT+ZV Discovery" Response (3)
	else if (strcmp(ResponseBuffer, "Device") == 0)
	{
		char BluetoothAddress[13];
		int BTAddrDebug[6];
		
		// Retreive Bluetooth Device Address (As Hexidecimal String)
		sscanf(ATResponse, "AT-ZV Device %s", BluetoothAddress);
		
		// Debug Discovery (Display to Screen)
		SendToMonitor("DvcFnd");
		
		// Retreive Bluetooth Device Address (As Integers)
		sscanf(BluetoothAddress, "%2x%2x%2x%2x%2x%2x", 
			&BTAddrDebug[0], &BTAddrDebug[1], &BTAddrDebug[2], 
			&BTAddrDebug[3], &BTAddrDebug[4], &BTAddrDebug[5]);
		
		// Debug Bluetooth Device Address (Display to Screen)
		sprintf(ResponseBuffer, "%c%c%c%c%c%c", 
			(char)BTAddrDebug[0], (char)BTAddrDebug[1], (char)BTAddrDebug[2], 
			(char)BTAddrDebug[3], (char)BTAddrDebug[4], (char)BTAddrDebug[5]);
		SendToMonitor(ResponseBuffer);
		
		/*
		// Debug Full Message (Display to Screen)
		sendStringToMonitor(ATResponse);
		*/
	}
	
	// "AT+ZV SPPConnect" Response (Success)
	else if (strcmp(ResponseBuffer, "ConnectionUp") == 0)
	{
		// Debug SPPConnect (Display to Screen)
		SendToMonitor("ConUp");
	}
	
	// "AT+ZV SPPConnect" Response (Failure) or "AT+ZV SPPDisconnect" Response (Success)
	else if (strcmp(ResponseBuffer, "SPPConnectionClosed") == 0)
	{
		// Debug SPPConnect (Display to Screen)
		SendToMonitor("ConCls");
	}
	
	// "AT+ZV SPPConnect" or "AT+ZV Bypass" Response (Success)
	else if (strcmp(ResponseBuffer, "-BypassMode-") == 0)
	{
		// Set Bluetooth Mode to Bypass
		BTMode = BT_MODE_BYPASS; 
		
		// Debug Bypass (Display to Screen)
		SendToMonitor("Bypass");
	}
	
	// "AT+ZV Bypass" Response (Failure)
	else if (strcmp(ResponseBuffer, "ConnectionDown") == 0)
	{
		// Debug Bypass (Display to Screen)
		SendToMonitor("ConDwn");
	}
	
	// Unrecognized AT-ZV Response
	else
	{
		int i;
		
		// Debug Response (Display to Screen)
		SendToMonitor("UknRsp");
		sendStringToMonitor(ResponseBuffer);
	}
}

//
// Bluetooth AT-ZV (Command Mode) Commands
//

// Enters command mode
void BtAtEscape()
{
	SendBtAtCommand("^#^$^%");
}

// Displays the Zerial interface version number
void BtAtVersion()
{
	SendBtAtCommand("AT+ZV Version");
}

// Finds and displays all available BT devices
void BtAtDiscovery()
{
	SendBtAtCommand("AT+ZV Discovery");
}

// Connects to an available BT device (by Node ID)
void BtAtSppConnect(int NodeId)
{
	char ConnectBuffer[64];
	
	sprintf(ConnectBuffer, "AT+ZV SPPConnect %012X", GetBTAddress(NodeId));
		
	SendBtAtCommand(ConnectBuffer);
}

// Disconnects from the current BT device
void BtAtSppDisconnect()
{
	SendBtAtCommand("AT+ZV SPPDisconnect");
}

// Enters bypass mode
void BtAtBypass()
{
	SendBtAtCommand("AT+ZV Bypass");
}

//
// Bluetooth Communication Wrapper Functions
//

// Sends a specially formatted AT-ZV command to the Bluetooth module.
int SendBtAtCommand(const char *ATCommand)
{
	char CommandBuffer[64] = "";
	
	// Append Carriage Return and Newline
	strcat(CommandBuffer, ATCommand);
	strcat(CommandBuffer, CRNL);
	
	// Debug AT-ZV Command
	SendToMonitor("SndCmd");
	
	// Send Command
	return SendToBt(CommandBuffer, (int)strlen(CommandBuffer)); 
}

// Sends a CAN message to a neighboring external module via BT. 
//		If the current node does not have a BT connection, the message is 
//		forwarded via CAN to a local node with a BT connection. 
//		- dest_id is the external Node ID (i.e., the true destination).
//		- a0 to a5 is the CAN message.
void SendToNeighboringModule(unsigned char dest_id, 
	unsigned char a0, unsigned char a1, unsigned char a2,
	unsigned char a3, unsigned char a4, unsigned char a5) 
{
	char msg[8];
	
	msg[SenderID] = dest_id;		// msg[SenderID] is the true destination (because snd2BT (called from ReceiveCANmsg) modifies the message...)
	msg[DstID] = BT_ID;				// msg[DstID] is the temporary (BT) destination (because ReceiveCANmsg immediately forwards all BT messages...)
	msg[Command] = a0;
	msg[ComArgment] = a1;
	msg[Message1] = a2;
	msg[Message2] = a3;
	msg[Message3] = a4;
	msg[Message4] = a5;
	
	// Forward message via CAN to a local BT module
	SendToCANuser(msg);
	
	// If the current module has a BT connection, send the message via BT
	if (BTMode == BT_MODE_BYPASS)
	{
		snd2BT(msg[SenderID], &msg[Command]); // Hack: "src" parameter is treated as "DstID" on the receiving end.
	}
}

// Sends a CAN message to any external module via BT (regardless of whether a direct
//		BT connection exists between the current module and the destination module). 
//		The message will automatically be forwarded to every reachable node. 
//		- dest_id is the external Node ID (i.e., the true destination).
//		- a0 to a3 is the CAN message.
void SendToAnyModule(unsigned char dest_id, unsigned char a0, 
	unsigned char a1, unsigned char a2, unsigned char a3)
{
	char msg[8];
	
	msg[SenderID]	= Variable.Name.HardID;
	msg[DstID]		= ALL_ID;				// (The message wrapper will be received by every reachable node)
	msg[Command]	= BT_FORWARD;
	msg[ComArgment]	= dest_id;				// msg[ComArgment] is the true destination
	msg[Message1]	= a0;
	msg[Message2]	= a1;
	msg[Message3]	= a2;
	msg[Message4]	= a3;
	
	// Forward message via CAN to all local modules
	SendToCANuser(msg);
	
	// If the current module has a BT connection, send the message via BT
	if (BTMode == BT_MODE_BYPASS)
	{
		snd2BT(msg[DstID], &msg[Command]); // Hack: "src" parameter is treated as "DstID" on the receiving end.
	}
}

// Forwards a CAN message to any external module via BT (regardless of whether a direct
//		BT connection exists between the current module and the destination module). 
//		The message will automatically be forwarded to every reachable node. 
void ForwardToAnyModule(unsigned char *msg)
{
	// If current module has a BT connection (but did not receive this message via BT), forward via BT
	if (BTMode == BT_MODE_BYPASS && !GetCTS())
	{
		snd2BT(msg[DstID], &msg[Command]);	// Hack: "src" parameter is treated as "DstID" on the receiving end.
	}
	
	// If message has arrived at the destination module, process it (the true destination is stored in msg[ComArgment])
	if(msg[ComArgment] == Variable.Name.HardID || msg[ComArgment] == ALL_ID)
	{	
		ExcuteCANmsg(&msg[Command]); // true message begins at msg[Command] (i.e., Command simulates SenderID)
	}
}

//
// Helper functions
//

// Turns on all active and passive IR LEDs for the 6 primary faces
void TurnOnAllIRLeds()
{
	// Turn on all IR LEDs
	TurnOnAllActiveIRLeds();
	TurnOnAllPassiveIRLeds();
}

// Turns on all active IR LEDs for the 3 primary faces
void TurnOnAllActiveIRLeds()
{
	// Retreive all Active LED IDs (Face 0, Face 1, and Face 2)
	union led_id_format all_active_leds = {
		FaceNumberToLedIds(0).BYTE | FaceNumberToLedIds(1).BYTE | FaceNumberToLedIds(2).BYTE 
	};
	
	// Turn on all active IR LEDs
	SetActiveIr(all_active_leds.BIT.upper, all_active_leds.BIT.lower);
}

// Turns on all passive IR LEDs for the 3 primary faces
void TurnOnAllPassiveIRLeds()
{
	// Retreive all Passive LED IDs (Face 3, Face 4, and Face 5)
	union led_id_format all_passive_leds = {
		FaceNumberToLedIds(3).BYTE | FaceNumberToLedIds(4).BYTE | FaceNumberToLedIds(5).BYTE
	};
	
	// Turn on all passive IR LEDs
	SetPassiveIr(all_passive_leds.BIT.upper, all_passive_leds.BIT.lower);
}

// Turns on one active IR LED (0-7)
void TurnOnOneActiveIRLed(unsigned char led)
{
	// Retreive active LED ID
	union led_id_format led_id = { LedNumberToLedId(led).BYTE };
	
	// Turn on active IR LED
	SetActiveIr(led_id.BIT.upper, led_id.BIT.lower);
}

// Turns on one passive IR LED (1,2,4,6-7)
void TurnOnOnePassiveIRLed(unsigned char led)
{
	// Retreive passive LED ID
	union led_id_format led_id = { LedNumberToLedId(led).BYTE };
	
	// Turn on passive IR LED
	SetPassiveIr(led_id.BIT.upper, led_id.BIT.lower);
}

// Turns off all active and passive IR LEDs for all faces
void TurnOffAllIRLeds()
{
	SetActiveIr(0, 0);
	SetPassiveIr(0, 0);
}

// Reads the IR for the spcified face (0x00-0x02: Active, 0x03-0x05: Passive, 0x13-0x15: Passive)
//	- Note: 0x0X and 0x1X correspond to the two IRs on the Xth Passive face. 
void ReadOneIR(unsigned char face)
{
	// Get active and passive IR readings
	StartIRdetection(face); 
	while (irDetect.mode != 0x80) WaitH8();
}

// Reads the IR for the specified face, and forwards it to the specified destination for display.
// - if sendtoAnyModule is true, limited IR information is forwarded to any module.
void DisplayOneIR(unsigned char face, unsigned char destID, unsigned char resetIR, unsigned char sendToAnyModule)
{
	ReadOneIR(face);
	
	// Reset IR LEDs
	if (resetIR)
	{
		TurnOnAllIRLeds();
	}
	
	// If no destination is provided, display IR data on the monitor:
	if (destID == 0)
	{
		Send2Monitor('I', 'R', Variable.Name.HardID, irDetect.face,
			irDetect.dark, irDetect.bright);
	}
	// Else forward data to another module for display:
	else if (sendToAnyModule)
	{
		SendToAnyModule(destID, C_MARK, SEND_TO_MONITOR, irDetect.dark, irDetect.bright);
	}
	else
	{
		SendToNeighboringModule(destID, C_MARK, SEND_TO_MONITOR, Variable.Name.HardID, 
			irDetect.face, irDetect.dark, irDetect.bright);
	}
}

// Reads and displays the IR for every face
void DisplayAllIR(unsigned char destID, unsigned char resetIR)
{
	int i;
	
	// Active IRs 
	for (i = 0; i <= 2; ++i)
	{
		DisplayOneIR(i, destID, resetIR, FALSE);
	}
	
	// Passive IRs (two IRs per face)
	for (i = 3; i <= 5; ++i)
	{
		DisplayOneIR(i, destID, resetIR, FALSE);
		DisplayOneIR(i + 0x10, destID, resetIR, FALSE);
	}
}

// Reads (and waits) and returns current IR status for the specified Active IR number 
//    (0-1: non-face IR (0 = battery side); 2-4: face IR)
unsigned char ReturnActiveIRStatus(unsigned char num)
{
	// Read and wait for the IR status to complete
	GetActiveIrStatus(num);
	WaitH8();
	
	// Return the IR data (set in getH8message)
	return Variable.Name.IR_data[0][num];
}

// Reads (and waits) and returns current IR status for the specified Passive IR number
//    (0-1: non-face IR (0 = battery side); 2-7: face IR)
unsigned char ReturnPassiveIRStatus(unsigned char num)
{
	// Read and wait for the IR status to complete
	GetPassiveIrStatus(num);
	WaitH8();
	
	// Return the IR data (set in getH8message)
	return Variable.Name.IR_data[1][num];
}

// Reads (and waits) and returns the maximum ambient IR status from non-face IRs
unsigned char ReturnAmbientIRStatus()
{
	int i;
	unsigned char ir_ambient[4];
	unsigned char ir_ambient_max = 0;
	
	// Receive ambient IR status from non-face IRs
	ir_ambient[0] = ReturnActiveIRStatus(0);
	ir_ambient[1] = ReturnActiveIRStatus(1);
	ir_ambient[2] = ReturnPassiveIRStatus(0);
	ir_ambient[3] = ReturnPassiveIRStatus(1);
	
	// Find maximum ambient IR
	for (i = 0; i < 4; ++i)
	{
		if (ir_ambient[i] > ir_ambient_max)
			ir_ambient_max = ir_ambient[i];
	}
	
	// Return maximum ambient IR value
	return ir_ambient_max;
}

// Sends a complete string to the Monitor
void sendStringToMonitor(char *msg)
{
	int i, length, remaining;
	
	length = strlen(msg);
	for (i = 0; i < length; i += 6)
	{
		remaining = length - i;
		if (remaining < 6)
		{
			char msg_tmp[6] = "      ";
			strcpy(msg_tmp, &msg[i]);
			SendToMonitor(msg_tmp);
		}
		else
		{
			SendToMonitor(&msg[i]);
		}
	}
}

// Initializes the locomotion parameters and starts walking
//		(Assumes that LocID has already been assigned for all modules)
void StartLocomotion()
{
	// Confirms that LocID has been assigned for all modules
	if (Variable.Name.HardID == Variable.Name.LocID)
		return;
		
	// Increase pushing force
	SetConfigLink(27, 40); //motor_push(27) = 400
	waitms(100);
	
	// Start walking
	commandLoc(0xf0, SHAPE_SL4M, 0, 0);			// Initialize shape
	commandLoc(0xd0, FORWARD_DIRECTION, 0, 0);	// Initialize direction
	commandLoc(0xe0, 1, 0, 0);					// Start locomotion
	waitms(500);
}

// Stops walking
void StopLocomotion()
{
	commandLoc(0xe0, 0, 0, 0);	// Stop locomotion
}

// Resumes walking
void ResumeLocomotion()
{
	commandLoc(0xe0, 1, 0, 0);	// Start locomotion
	waitms(500);
}

// Reconfigures the walker into an ideal shape for IR reading
void AssumeThePosition()
{
	// Confirms that LocID has been assigned for all modules
	if (Variable.Name.HardID == Variable.Name.LocID)
		return;
		
	switch (Variable.Name.LocID)
	{
		case 1: // White module on bottom (85 deg)
		case 3:
			SetAngleDir(0, 0x55, 0x55);
			break;
		case 2: // Black module on bottom (35 deg)
		case 4:
			SetAngleDir(0, 0x23, 0x23);
			break;
	}

	// Wait for current movement to finish
	Variable.Name.move_end = 0;
	while (H8waiting());

	// Move to new angle
	MotorOn(3);
	while (Variable.Name.move_end == 0)
	{	
		GetMotorComplete();
		while (H8waiting());
	}

	// Finished moving: Turn off motor
	MotorOn(0);
}

//
// Private Helper Functions
//

// Returns true if currently processing a BT message
int GetCTS()
{
	return PEDRL.BIT.CTS;
}

// Returns the Bluetooth address for the specified Node ID
unsigned long GetBTAddress(int NodeID)
{
	// Calculate the BT Address Index from the Node ID (Nodes 1-39 are not used)
	int index = NodeID - 40;
	
	// Error checking: Confirm a valid index
	int min_index = 0;
	int max_index = (sizeof(BTAddress) / sizeof(BTAddress[0])) - 1;
	if (index < min_index || index > max_index)
	{
		return -1;
	}
	
	// Return the BT Address
	return BTAddress[index];
}

// Returns the formatted LED IDs for the specified Face Number (0-2 = Active, 3-5 = Passive)
union led_id_format FaceNumberToLedIds(unsigned char face)
{
	union led_id_format led_id;
	
	switch (face)
	{
		// Active faces
		case 0  : led_id.BYTE = LedNumberToLedId(0).BYTE | LedNumberToLedId(1).BYTE; break; // Face 0: LED 0 and LED 1
		case 1  : led_id.BYTE = LedNumberToLedId(2).BYTE | LedNumberToLedId(3).BYTE; break; // Face 1: LED 2 and LED 3
		case 2  : led_id.BYTE = LedNumberToLedId(4).BYTE | LedNumberToLedId(5).BYTE; break; // Face 2: LED 4 and LED 5
		// Passive faces
		case 3  : led_id.BYTE = LedNumberToLedId(1).BYTE; break; // Face 3: LED 1
		case 4  : led_id.BYTE = LedNumberToLedId(2).BYTE; break; // Face 4: LED 2
		case 5  : led_id.BYTE = LedNumberToLedId(4).BYTE; break; // Face 5: LED 4
		default : led_id.BYTE = 0x00; break;
	}
	
	return led_id;
}

// Returns the formatted LED ID for the specified LED number (0-7)
union led_id_format LedNumberToLedId(unsigned char led)
{
	union led_id_format led_id;
	
	switch (led)
	{
		case 0  : led_id.BYTE = 0x80; break; // (1)  Face 0 (Active) / 3 (Passive)
		case 1  : led_id.BYTE = 0x10; break; // (1)       0          / 3
		case 2  : led_id.BYTE = 0x08; break; // (2)  Face 1 (Active) / 4 (Passive)
		case 3  : led_id.BYTE = 0x40; break; // (2)       1          / 4
		case 4  : led_id.BYTE = 0x04; break; // (4)  Face 2 (Active) / 5 (Passive)
		case 5  : led_id.BYTE = 0x20; break; // (4)       2          / 5
		case 6  : led_id.BYTE = 0x02; break; // (8)	 (Opposite side as Battery)
		case 7  : led_id.BYTE = 0x01; break; // (16) (Same Side as Battery)
		default : led_id.BYTE = 0x00; break;
	}
	
	return led_id;
}

//
// IR Data Collection Test Functions
//

unsigned char CommanderNodeID;		// Node A
unsigned char WalkerNodeID[4];		// Node B
unsigned char SnakeNodeID;			// Node C

// Initialize Walker Node IDs
void IRDataTest_NodeA_InitWalkerNodeIds(unsigned char NodeID1, 
	unsigned char NodeID2, unsigned char NodeID3, unsigned char NodeID4)
{
	WalkerNodeID[0] = NodeID1;
	WalkerNodeID[1] = NodeID2;
	WalkerNodeID[2] = NodeID3;
	WalkerNodeID[3] = NodeID4;
}

// Initialize Snake Node ID
void IRDataTest_NodeA_InitSnakeNodeId(unsigned char NodeID)
{
	SnakeNodeID = NodeID;
}

// Initialize Commander Node ID (SND)
void IRDataTest_NodeA_InitCommanderNodeId(unsigned char NodeID)
{
	// Send to All Nodes: Initialize Commander Node ID
	SendToAnyModule(ALL_ID, C_MARK, IR_DATA_TEST_ALL_NODES_INIT_COMMANDER_NODE_ID, NodeID, 0);
}

// Initialize Commander Node ID (RCV)
void IRDataTest_AllNodes_InitCommanderNodeId(unsigned char NodeID)
{
	CommanderNodeID = NodeID;
	
	// Debug IDs
	SendToAnyModule(CommanderNodeID, C_MARK, SEND_TO_MONITOR, Variable.Name.HardID, 0); // debug
}

// Begin collecting data
void IRDataTest_NodeA_StartTest()
{
	// Send to Snake Node: "Turn on Face 4 IR LED"
	SendToAnyModule(SnakeNodeID, C_MARK, IR_DATA_TEST_NODEC_TURN_ON_LED, 2, 0); // Only use mode 2 & 3
}

// Turn on/off Snake Node's Face 1 IR LED (Message1: 0 = Off, 1 = Left, 2 = Right, 3 = All) (RCV)
void IRDataTest_NodeC_TurnOnLed(unsigned char ModeIndex)
{
	switch (ModeIndex)
	{
		case 0: // Left IR LED On
			TurnOnOneActiveIRLed(2);
			break;
			
		case 1: // Right IR LED On
			TurnOnOneActiveIRLed(3);
			break;
			
		case 2: // All IR LEDs On
			//lightIRface(1);
			TurnOnAllPassiveIRLeds();
			break;
			
		case 3: // All IR LEDs Off
			TurnOffAllIRLeds();
			break;
	}
	
	// Send to Commander Node: "Turn on Face 1 IR LED" Confirmation
	SendToAnyModule(CommanderNodeID, C_MARK, IR_DATA_TEST_NODEA_TURN_ON_LED_CONFIRM, ModeIndex, 0);
}

// Turn on/off Snake Node's Face 1 IR LED (CNF)
void IRDataTest_NodeA_TurnOnLedConfirm(unsigned char ModeIndex)
{
	// If valid LED mode: Send to each Walker Node, "Read all IR"
	if (ModeIndex >= 0 && ModeIndex <= 3)
	{
		IRDataTest_NodeA_DisplayAllIRConfirm(ModeIndex, -1);
	}
	// Else: done
}

// Measure and display all Walker Nodes' IRs (SND)
void IRDataTest_NodeB_DisplayAllIR(unsigned char ModeIndex, unsigned char WalkerNodeIndex)
{
	// Read and display all active and passive IR
	DisplayAllIR(CommanderNodeID, 0);
	
	// Send to Commander Node: "Read all IR" Confirmation
	SendToNeighboringModule(CommanderNodeID, C_MARK, IR_DATA_TEST_NODEA_DISPLAY_ALL_IR_CONFIRM, ModeIndex, WalkerNodeIndex, 0, 0);
}

// Measure and display all Walker Nodes' IRs (CNF)
void IRDataTest_NodeA_DisplayAllIRConfirm(unsigned char ModeIndex, unsigned char WalkerNodeIndex)
{
	// Process next Walker Node
	++WalkerNodeIndex;
	
	// For each module: Send to Walker Node, read all IR
	if (WalkerNodeIndex >= 0 && WalkerNodeIndex <= 3)
	{
		SendToNeighboringModule(WalkerNodeID[WalkerNodeIndex], C_MARK, IR_DATA_TEST_NODEB_DISPLAY_ALL_IR, ModeIndex, WalkerNodeIndex, 0, 0);
	}
	// Else: Move to the next Snake Node LED mode
	else
	{
		SendToAnyModule(SnakeNodeID, C_MARK, IR_DATA_TEST_NODEC_TURN_ON_LED, ++ModeIndex, 0);
	}
}

//
// IR Docking Test
//

unsigned char IRDockingTestMode = IR_SEARCH_MODE;

// Initialization
unsigned char NodeBID = 0;
unsigned char IRAmbientMaxValue = 0;
unsigned char IRAmbientRcvCount = 0;

// Search mode
unsigned char IRLedMaxValue = 0;
unsigned char IRLedDirection = 0;

// Docking mode
unsigned char DockingNodeLocID = 0;
unsigned char InitIRCollaborationRcvCount = 0;
unsigned char IRCollaborationData[6];


unsigned long DockingManeuverWaitTime = 0;
unsigned long DockingManeuverElapsedTime = 0;

//
// IR Docking Test: Callback Functions
//

// Custom IR callback function for IR Docking Test
void LocIrCheckM()
{
	switch (IRDockingTestMode)
	{
		case IR_SEARCH_MODE:
			LocIrCheckM_SearchMode();
			break;
			
		case IR_DOCKING_MODE:
			LocIrCheckM_DockingMode();
			break;
	}
}

// Search Mode IR callback function: 
// - Reads all non-blocked Active and Passive IRs for each module. 
// - Obstacles (Active) are avoided immediately, and maximum IR values (Passive)
//		are forwarded to the Commander Node for processing.
void LocIrCheckM_SearchMode()
{
	switch (Variable.Name.LocID)
	{
		case 1: // Node 1: Process only faces 2, 3, and 5 (white module on bottom) 
			if (irDetect.mode == 0x00) // IR initialized
			{
				StartIRdetection(2);
			}
			else if (irDetect.mode == 0x80) // IR readings complete
			{
				irDetect.mode = 0x00;
				
				switch (irDetect.face)
				{
					case 0: break; // Attached to Node 2
					case 1: break; // Attached to Node 4
					
					case 2: // Pointing towards LEFT
						IRLedMaxValue = 0;					// Reset maximum detected IR LED value
						IRProcess(LEFT_DIRECTION, FALSE);	// Process IR data
						StartIRdetection(3);				// Move to the next face
						break;
						
					case 3: // Pointing towards LEFT
						IRProcess(LEFT_DIRECTION, FALSE);	// Process IR data
						StartIRdetection(4);				// Move to the next face
						break;
						
					case 4: // Pointing towards FOWARD
						IRProcess(FORWARD_DIRECTION, FALSE);// Process IR data
						StartIRdetection(5);				// Move to the next face
						break;
						
					case 5: // Pointing towards RIGHT (blocked by Node 2)
						IRProcess(RIGHT_DIRECTION, TRUE);	// Process IR data (detect IR LEDs only)
						IRDockingTest_NodeA_IRLedMaxValueSend(IRLedMaxValue, IRLedDirection);	// Notify Commander Node of maximum detected IR LED value and direction
						StartIRdetection(2);				// Move to the next face
						break;
				}
			}
			break;
		
		case 2: // Node 2: Process only faces 0, 1, and 5 (black module on bottom)
			if (irDetect.mode == 0x00) // IR initialized
			{
				StartIRdetection(0);
			}
			else if (irDetect.mode == 0x80) // IR readings complete
			{
				irDetect.mode = 0x00;
				
				switch (irDetect.face)
				{
					case 0: // Pointing towards FORWARD
						IRLedMaxValue = 0;					// Reset maximum detected IR LED value
						IRProcess(FORWARD_DIRECTION, FALSE);// Process IR data
						StartIRdetection(1);				// Move to the next face
						break;
						
					case 1: // Pointing towards RIGHT
						IRProcess(RIGHT_DIRECTION, FALSE);	// Process IR data
						StartIRdetection(2);				// Move to the next face
						break;
						
					case 2: // Pointing towards BACK (blocked by Node 3)					
						IRProcess(BACK_DIRECTION, TRUE);	// Process IR data (detect IR LEDs only)
						StartIRdetection(5);				// Move to the next face
						break;
						
					case 3: break; // Attached to Node 3
					case 4: break; // Attached to Node 1
					
					case 5: // Pointing towards FORWARD
						IRProcess(FORWARD_DIRECTION, FALSE);// Process IR data
						IRDockingTest_NodeA_IRLedMaxValueSend(IRLedMaxValue, IRLedDirection);	// Notify Commander Node of maximum detected IR LED value and direction
						StartIRdetection(0);				// Move to the next face
						break;
				}
			}
			break;
		
		case 3: // Node 3: Process only faces 2, 3, and 5 (white module on bottom)
			if (irDetect.mode == 0x00) // IR initialized
			{
				StartIRdetection(2);
			}
			else if (irDetect.mode == 0x80) // IR readings complete
			{
				irDetect.mode = 0x00;
				
				switch (irDetect.face)
				{
					case 0: break; // Attached to Node 4
					case 1: break; // Attached to Node 2
					
					case 2: // Pointing towards RIGHT
						IRLedMaxValue = 0;					// Reset maximum detected IR LED value
						IRProcess(RIGHT_DIRECTION, FALSE);	// Process IR data
						StartIRdetection(3);				// Move to the next face
						break;
						
					case 3: // Pointing towards RIGHT
						IRProcess(RIGHT_DIRECTION, FALSE);	// Process IR data
						StartIRdetection(4);				// Move to the next face
						break;
						
					case 4: // Pointing towards BACK
						IRProcess(BACK_DIRECTION, FALSE);	// Process IR data
						IRDockingTest_NodeA_IRLedMaxValueSend(IRLedMaxValue, IRLedDirection);	// Notify Commander Node of maximum detected IR LED value and direction
						StartIRdetection(5);				// Move to the next face
						break;
						
					case 5: // Pointing towards RIGHT (blocked by Node 4)
						IRProcess(BACK_DIRECTION, TRUE);	// Process IR data (detect IR LEDs only)
						IRDockingTest_NodeA_IRLedMaxValueSend(IRLedMaxValue, IRLedDirection);	// Notify Commander Node of maximum detected IR LED value and direction
						StartIRdetection(2);				// Move to the next face
						break;
				}
			}
			break;
		
		case 4: // Node 4: Process only faces 0, 1, and 5 (black module on bottom)
			if (irDetect.mode == 0x00) // IR initialized
			{
				StartIRdetection(0);
			}
			else if (irDetect.mode == 0x80) // IR readings complete
			{
				irDetect.mode = 0x00;
				
				switch (irDetect.face)
				{
					case 0: // Pointing towards BACK
						IRLedMaxValue = 0;					// Reset max detected IR value
						IRProcess(BACK_DIRECTION, FALSE);	// Process IR data
						StartIRdetection(1);				// Move to the next face
						break;
						
					case 1: // Pointing towards LEFT
						IRProcess(LEFT_DIRECTION, FALSE);	// Process IR data
						StartIRdetection(2);				// Move to the next face
						break;
						
					case 2: // Pointing towards FORWARD (blocked by Node 1)
						IRProcess(FORWARD_DIRECTION, TRUE);	// Process IR data (detect IR LEDs only)
						StartIRdetection(5);				// Move to the next face
						break;
						
					case 3: break; // Attached to Node 1
					case 4: break; // Attached to Node 3
					
					case 5: // Pointing towards BACK
						IRProcess(BACK_DIRECTION, FALSE);	// Process IR data
						IRDockingTest_NodeA_IRLedMaxValueSend(IRLedMaxValue, IRLedDirection);	// Notify Commander Node of maximum detected IR LED value and direction
						StartIRdetection(0);				// Move to the next face
						break;
				}
			}
			break;
	}
}

// Docking Mode IR callback function: 
// - Performs IR collaboration and pose alignment
void LocIrCheckM_DockingMode()
{
	if (Variable.Name.LocID == DockingNodeLocID)
	{
		// Don't interrupt corrective movements
		if (GetTickCount() < DockingManeuverWaitTime)
			return;
		
		// Begin IR collaboration for pose alignment
		IRDockingTest_NodeADocking_InitIRCollaboration_Send(0);
	}
}

//
// IR Docking Test: Helper Functions
//

// Uses the current IR data to avoid obstacles and locate IR LEDs
// - facing_direction is the orientation of the current IR data (FORWARD_DIRECTION, LEFT_DIRECTION, RIGHT_DIRECTION, BACK_DIRECTION)
// - ir_detect_led_only is used to ignore obstacle detection (0 = Obstacle & LED, 1 = LED)
void IRProcess(unsigned char facing_direction, unsigned char ir_detect_led_only)
{
	int ir_led_value = 0;
	
	// If an obstacle is detected:
	if (!ir_detect_led_only && IRDetectObstacle())
	{
		// debug
		Send2Monitor('O', 'B', 'T', irDetect.face, irDetect.dark, irDetect.bright);
		
		// And we are heading towards it:
		if (LocDirection == facing_direction)
		{
			// Select a new direction (always turning counter-clockwise)
			switch (facing_direction)
			{
				case FORWARD_DIRECTION:
					ChangeLocDir(1, LEFT_DIRECTION); break;
					
				case LEFT_DIRECTION:
					ChangeLocDir(1, BACK_DIRECTION); break;
					
				case RIGHT_DIRECTION:
					ChangeLocDir(1, FORWARD_DIRECTION); break;
					
				case BACK_DIRECTION:
					ChangeLocDir(1, RIGHT_DIRECTION); break;
			}
		}
	}
	// Otherwise, If an IR LED is detected:
	else if ((ir_led_value = IRDetectLED()) > IRLedMaxValue)
	{
		// debug
		Send2Monitor('I', 'R', 'T', irDetect.face, irDetect.dark, irDetect.bright);
		
		// Update maximum IR LED value and direction
		IRLedMaxValue = ir_led_value;
		IRLedDirection = facing_direction;
	}
}

// Returns the IR value (true) if an LED is detected, and 0 (false) otherwise
int IRDetectLED()
{
	// If an LED is detected:
	if ((irDetect.dark - IRAmbientMaxValue) >= IR_THRESHOLD) // Normalize IR reading
	{
		return irDetect.dark;
	}
	
	return FALSE;
}

// Returns true if an obstacle is detected, false otherwise
int IRDetectObstacle()
{
	int isObstacleDetected = FALSE;
	
	// If an LED is detected, use a larger obstacle threshold:
	if (IRDetectLED())
	{
		isObstacleDetected = (irDetect.bright - irDetect.dark) >= OBSTACLE_THRESHOLD_LED;
	}
	// Otherwise, use a smaller obstacle threshold: an obstacle is detected:
	else
	{
		isObstacleDetected = (irDetect.bright - irDetect.dark) >= OBSTACLE_THRESHOLD;
	}
	
	return isObstacleDetected;
}

//
// IR Docking Test: Application Functions
//

// Initializes and begins the IR Docking Test
// 	- (Assumes that LocID has been assigned for all modules, 
//			and all BT connections have been established)
void IRDockingTest_NodeACommander_StartTest(unsigned char commander_node_id, unsigned char node_b_id)
{
	// Intialize the Commander Node ID for all modules
	Send2All('Z', IR_DOCKING_TEST_NODEA_INIT_COMMANDER_NODE_ID, commander_node_id, 0, 0, 0);
	IRDockingTest_NodeA_InitCommanderNodeId(commander_node_id); // for me too...
	
	// Intialize the Node B ID for all modules
	Send2All('Z', IR_DOCKING_TEST_NODEA_INIT_NODE_B_ID, node_b_id, 0, 0, 0);
	IRDockingTest_NodeA_InitNodeBID(node_b_id); // for me too...
	
	// Update the ambient IR of the current environment
	IRDockingTest_NodeACommander_IRAmbientMaxValue_Send();
}

// Updates the Commander Node ID
void IRDockingTest_NodeA_InitCommanderNodeId(unsigned char NodeID)
{
	CommanderNodeID = NodeID;
}

// Updates the Node B ID
void IRDockingTest_NodeA_InitNodeBID(unsigned char NodeID)
{
	NodeBID = NodeID;
}

// Requests the maximum ambient IR value from all modules
void IRDockingTest_NodeACommander_IRAmbientMaxValue_Send()
{
	// Select a convenient position for reading IRs
	Send2All('Z', STOP_LOCOMOTION, 0, 0, 0, 0);
	StopLocomotion(); // for me too...
	
	Send2All('Z', ASSUME_THE_POSITION, 0, 0, 0, 0);
	AssumeThePosition(); // for me too...
	
	// Reset the default ambient IR
	IRAmbientMaxValue = 0;
	
	// Reset the number of replied modules
	IRAmbientRcvCount = 0;
	
	// Broadcast a maximum ambient IR value request
	Send2All('Z', IR_DOCKING_TEST_NODEA_IR_AMBIENT_MAX_VALUE_SEND, 0, 0, 0, 0);
	IRDockingTest_NodeA_IRAmbientMaxValue_Send(); // for me too...
}

// Forwards the maximum ambient IR value to the Commander Node
void IRDockingTest_NodeA_IRAmbientMaxValue_Send()
{
	// Retrieve ambient IR status
	unsigned char ir_ambient = ReturnAmbientIRStatus();
	
	// Forward ambient IR status to Commander Node
	if (Variable.Name.HardID == CommanderNodeID)
		IRDockingTest_NodeACommander_IRAmbientMaxValue_Receive(ir_ambient);
	else
		Send2CAN(CommanderNodeID, 'Z', IR_DOCKING_TEST_NODEA_COMMANDER_IR_AMBIENT_MAX_VALUE_RECEIVE, ir_ambient, 0, 0, 0);
}

// Finds and initializes the maximum ambient IR value for all modules
void IRDockingTest_NodeACommander_IRAmbientMaxValue_Receive(unsigned char ir_ambient)
{
	// Update the number of replied modules
	++IRAmbientRcvCount;
	
	// Find maximum ambient IR status
	if (ir_ambient > IRAmbientMaxValue)
		IRAmbientMaxValue = ir_ambient;
	
	// If all modules have responded:
	if (IRAmbientRcvCount == 4)
	{
		// debug
		Send2Monitor('A', 'M', 'B', 0, 0, IRAmbientMaxValue);
		
		// Update ambient IR status for all modules
		Send2All('Z', IR_DOCKING_TEST_NODEA_IR_AMBIENT_MAX_VALUE_RECEIVE, IRAmbientMaxValue, 0, 0, 0);
		
		// Enter IR Search Mode
		IRDockingTest_NodeADocking_IRSearchMode_Send();
	}
}

// Updates the maximum ambient IR value
void IRDockingTest_NodeA_IRAmbientMaxValue_Receive(unsigned char ir_ambient)
{
	IRAmbientMaxValue = ir_ambient;
}

// Requests all modules to enter IR Search Mode
void IRDockingTest_NodeADocking_IRSearchMode_Send()
{
	// Update IR Docking Test mode to IR Search Mode
	Send2All('Z', IR_DOCKING_TEST_NODEA_IR_SEARCH_MODE_RECEIVE, 0, 0, 0, 0);
	IRDockingTest_NodeA_IRSearchMode_Receive(); // for me too...
}

// Updates the IR Docking Test mode to IR Search Mode
void IRDockingTest_NodeA_IRSearchMode_Receive()
{
	IRDockingTestMode = IR_SEARCH_MODE;
	
	// Resume IR Docking Test
	Send2All('Z', START_LOCOMOTION, 0, 0, 0, 0); // walk again
	StartLocomotion(); // for me too...
}

// Requests all modules to enter IR Docking Mode
void IRDockingTest_NodeACommander_IRDockingMode_Send(unsigned char loc_id)
{
	// Update IR Docking Test mode to IR Docking Mode
	Send2All('Z', IR_DOCKING_TEST_NODEA_IR_DOCKING_MODE_RECEIVE, loc_id, 0, 0, 0);
	IRDockingTest_NodeA_IRDockingMode_Receive(loc_id); // for me too...
}

// Updates the IR Docking Test mode to IR Docking Mode
void IRDockingTest_NodeA_IRDockingMode_Receive(unsigned char loc_id)
{
	IRDockingTestMode = IR_DOCKING_MODE;
	DockingNodeLocID = loc_id;
	
	if (Variable.Name.LocID == DockingNodeLocID)
	{
		// Update docking maneuver start time
		DockingManeuverElapsedTime = 0;
		
		// Initialize the Particle Filter
		InitializePoseParticles();
	}
}

// Requests all modules to enter IR Docking Complete Mode
void IRDockingTest_NodeADocking_IRDockingCompleteMode_Send()
{
	// Update IR Docking Test mode to IR Docking Complete Mode
	Send2All('Z', IR_DOCKING_TEST_NODEA_IR_DOCKING_COMPLETE_MODE_RECEIVE, 0, 0, 0, 0);
	IRDockingTest_NodeA_IRDockingCompleteMode_Receive(); // for me too...
}

// Updates the IR Docking Test mode to IR Docking Complete Mode
void IRDockingTest_NodeA_IRDockingCompleteMode_Receive()
{
	IRDockingTestMode = IR_DOCKING_COMPLETE_MODE;
}

// Forwards the maximum IR value (Passive) and the direction to the Commander Node
void IRDockingTest_NodeA_IRLedMaxValueSend(unsigned char ir_value, unsigned char ir_direction)
{
	static unsigned char message_counter = 0;
	static unsigned char last_ir_value = -1;
	static unsigned char last_ir_direction = -1;
	
	// Avoid sending duplicate messages
	if (ir_value == last_ir_value && ir_direction == last_ir_direction)
		return;
	
	// Update the last IR value and direction
	last_ir_value = ir_value;
	last_ir_direction = ir_direction;
	
	// Forward the maximum IR value to the Commander Node
	if (Variable.Name.HardID == CommanderNodeID)
		IRDockingTest_NodeACommander_IRLedMaxValueReceive(Variable.Name.LocID, ir_value, ir_direction);
	else
		Send2CAN(CommanderNodeID, 'Z', IR_DOCKING_TEST_NODEA_COMMANDER_IR_LED_MAX_VALUE_RECEIVE,
			Variable.Name.LocID, ir_value, ir_direction, ++message_counter);
}

// Finds the maximum IR value (Passive) the direction and attempts to:
//		1) Maximize the IR value on any module face
//		2) Maximize the IR value on a docking face
void IRDockingTest_NodeACommander_IRLedMaxValueReceive(unsigned char loc_id, unsigned char ir_value, unsigned char ir_direction)
{
	static unsigned long maneuver_timeout = 0;
	static unsigned char all_ir_led_max_value[4] = {0, 0, 0, 0};
	static unsigned char all_ir_led_direction[4] = {0, 0, 0, 0};
	
	unsigned char ir_led_loc_id = 0;
	unsigned char ir_led_max_value = 0;
	unsigned char ir_led_direction = 0;
	int i;
	
	// Update maximum IR LED value and direction
	all_ir_led_max_value[loc_id - 1] = ir_value;
	all_ir_led_direction[loc_id - 1] = ir_direction;
	
	// Find maximum LED value
	for (i = 0; i < 4; ++i)
	{
		if (all_ir_led_max_value[i] > ir_led_max_value)
		{
			ir_led_loc_id = i + 1;
			ir_led_max_value = all_ir_led_max_value[i];
			ir_led_direction = all_ir_led_direction[i];
		}
	}
	
	// debug
	//Send2Monitor('M', 'A', 'X', ir_led_loc_id, ir_led_max_value, ir_led_direction);
	
	// Orienting the docking module, don't interrupt this maneuver!
	if (maneuver_timeout > GetTickCount() &&
		// (unless the docking module is already oriented!)
		!((ir_led_max_value >= IR_THRESHOLD_MAX) && 
				((ir_led_loc_id == 2 && ir_led_direction == RIGHT_DIRECTION) ||
				 (ir_led_loc_id == 4 && ir_led_direction == LEFT_DIRECTION))))
		return;
	
	// If the maximum IR value is detected:
	if (ir_led_max_value > IR_THRESHOLD_MAX)
	{
		// Maneuver a docking module to the IR LED (black module on bottom)
		switch (ir_led_loc_id)
		{
			case 1: // Node 1: (white module on bottom)
				if (ir_led_direction == FORWARD_DIRECTION)
				{
					maneuver_timeout = GetTickCount() + 5000;
					ChangeLocDir(1, ROTATE_RIGHT);
				}
				else if (ir_led_direction == LEFT_DIRECTION || 
						 ir_led_direction == RIGHT_DIRECTION)
				{
					maneuver_timeout = GetTickCount() + 3000;
					ChangeLocDir(1, FORWARD_DIRECTION);
				}
				break;
				
			case 2: // Node 2: (black module on bottom)
				if (ir_led_direction == FORWARD_DIRECTION)
				{
					maneuver_timeout = GetTickCount() + 5000;
					ChangeLocDir(1, ROTATE_LEFT);
				}
				else if (ir_led_direction == RIGHT_DIRECTION)
				{
					// IR LED already on docking face! Done!!
					IRDockingTest_NodeACommander_IRDockingMode_Send(ir_led_loc_id);
				}
				else if (ir_led_direction == BACK_DIRECTION)
				{
					maneuver_timeout = GetTickCount() + 5000;
					ChangeLocDir(1, ROTATE_RIGHT);
				}
				break;
				
			case 3: // Node 3: (white module on bottom)
				if (ir_led_direction == LEFT_DIRECTION ||
					ir_led_direction == RIGHT_DIRECTION)
				{
					maneuver_timeout = GetTickCount() + 3000;
					ChangeLocDir(1, BACK_DIRECTION);
				}
				else if (ir_led_direction == BACK_DIRECTION)
				{
					maneuver_timeout = GetTickCount() + 5000;
					ChangeLocDir(1, ROTATE_RIGHT);
				}
				break;
				
			case 4: // Node 4: (black module on bottom)
				if (ir_led_direction == FORWARD_DIRECTION)
				{
					maneuver_timeout = GetTickCount() + 5000;
					ChangeLocDir(1, ROTATE_RIGHT);
				} 
				else if (ir_led_direction == LEFT_DIRECTION)
				{
					// IR LED already on docking face! Done!!
					IRDockingTest_NodeACommander_IRDockingMode_Send(ir_led_loc_id);
				}
				else if (ir_led_direction == BACK_DIRECTION)
				{
					maneuver_timeout = GetTickCount() + 5000;
					ChangeLocDir(1, ROTATE_LEFT);
				}
				break;
		}
	}	
	// Otherwise, Head towards the highest LED value (if not already headed there)
	else if ((ir_led_max_value > IR_THRESHOLD) && (LocDirection != ir_led_direction))
	{
		ChangeLocDir(1, ir_led_direction);
	}
}

// Requests all modules to select a convenient pose for reading IRs
void IRDockingTest_NodeADocking_InitIRCollaboration_Send()
{
	// Disable automatic IR readings
	Variable.Name.inhibitIR = TRUE;
			
	// Reset the number of replied modules
	InitIRCollaborationRcvCount = 0;
	
	// Broadcast a request to pose for IR Collaboration
	Send2All('Z', IR_DOCKING_TEST_NODEA_INIT_IR_COLLABORATION_SEND, Variable.Name.HardID, 0, 0, 0);
	IRDockingTest_NodeA_InitIRCollaboration_Send(Variable.Name.HardID); // for me too...
}

// Select a convenient pose for reading IRs
void IRDockingTest_NodeA_InitIRCollaboration_Send(unsigned char docking_node_id)
{
	// Update elapsed movement time
	if (Variable.Name.LocID == DockingNodeLocID)
	{
		if (DockingManeuverElapsedTime != 0)
			DockingManeuverElapsedTime = GetTickCount() - DockingManeuverElapsedTime;
	}
	
	// Initialize pose for IR Collaboration
	StopLocomotion();
	AssumeThePosition();
	
	// Forward response to Docking Node
	if (Variable.Name.LocID == DockingNodeLocID)
		IRDockingTest_NodeADocking_InitIRCollaboration_Receive();
	else
		Send2CAN(docking_node_id, 'Z', IR_DOCKING_TEST_NODEA_DOCKING_INIT_IR_COLLABORATION_RECEIVE, 0, 0, 0, 0);
}

// Confirms that all modules have selected a convenient pose for reading IRs and begins IR Collaboration
void IRDockingTest_NodeADocking_InitIRCollaboration_Receive()
{
	// Update the number of replied modules
	++InitIRCollaborationRcvCount;
	
	// If all modules have responded:
	if (InitIRCollaborationRcvCount == 4)
	{
		// Begin IR Collaboration
		IRDockingTest_NodeADocking_IRCollaboration_Send(0);
	}
}

// Collaborates with an external module to read many IR configurations
void IRDockingTest_NodeADocking_IRCollaboration_Send(unsigned char ModeIndex)
{
	switch (ModeIndex)
	{
		case 0:
			// Get a new IR reading
			ReadOneIR(1);
			
			// Save a copy of the current IR reading
			IRCollaborationData[NODE_A_IR_WITH_NODE_B_LED_ON] = irDetect.dark;
			
			//  Send to Node B: "Turn off IR LED"
			SendToAnyModule(NodeBID, C_MARK, IR_DOCKING_TEST_NODEB_IR_COLLABORATION_RECEIVE, ModeIndex, Variable.Name.HardID);
			break;
			
		case 1:
			// Get a new IR reading 
			ReadOneIR(1);
		
			// Save a copy of the current IR reading
			IRCollaborationData[NODE_A_IR_WITH_NODE_B_LED_OFF] = irDetect.bright;
		
			// Turn Left IR LED On
			TurnOnOneActiveIRLed(2);
			
			//  Send to Node B: "Return Passive IR Top"
			SendToAnyModule(NodeBID, C_MARK, IR_DOCKING_TEST_NODEB_IR_COLLABORATION_RECEIVE, ModeIndex, 0);
			break;
			
		case 2:
			//  Send to Node B: "Return Passive IR Left"
			SendToAnyModule(NodeBID, C_MARK, IR_DOCKING_TEST_NODEB_IR_COLLABORATION_RECEIVE, ModeIndex, 0);
			break;
			
		case 3:
			// Turn Right IR LED On
			TurnOnOneActiveIRLed(3);
			
			//  Send to Node B: "Return Passive IR Top"
			SendToAnyModule(NodeBID, C_MARK, IR_DOCKING_TEST_NODEB_IR_COLLABORATION_RECEIVE, ModeIndex, 0);
			break;

		case 4:
			//  Send to Node B: "Return Passive IR Left"
			SendToAnyModule(NodeBID, C_MARK, IR_DOCKING_TEST_NODEB_IR_COLLABORATION_RECEIVE, ModeIndex, 0);
			break;
	}
}

// Collaborates with an external module to read many IR configurations
void IRDockingTest_NodeB_IRCollaboration_Receive(unsigned char ModeIndex, unsigned char docking_node_id)
{
	switch (ModeIndex)
	{
		case 0:
			//  Turn off IR LEDs
			TurnOffAllIRLeds();
			
			SendToAnyModule(docking_node_id, C_MARK, IR_DOCKING_TEST_NODEA_DOCKING_IR_COLLABORATION_RECEIVE, ModeIndex, 0);
			break;
			
		case 1:
			// Get a new Top IR reading
			ReadOneIR(0x04);
		
			SendToAnyModule(docking_node_id, C_MARK, IR_DOCKING_TEST_NODEA_DOCKING_IR_COLLABORATION_RECEIVE, ModeIndex, irDetect.dark);
			break;
			
		case 2:
			// Get a new Left IR reading
			ReadOneIR(0x14);
			
			SendToAnyModule(docking_node_id, C_MARK, IR_DOCKING_TEST_NODEA_DOCKING_IR_COLLABORATION_RECEIVE, ModeIndex, irDetect.dark);
			break;
			
		case 3:
			// Get a new Top IR reading
			ReadOneIR(0x04);
		
			SendToAnyModule(docking_node_id, C_MARK, IR_DOCKING_TEST_NODEA_DOCKING_IR_COLLABORATION_RECEIVE, ModeIndex, irDetect.dark);
			break;
			
		case 4:
			// Get a new Left IR reading
			ReadOneIR(0x14);
			
			SendToAnyModule(docking_node_id, C_MARK, IR_DOCKING_TEST_NODEA_DOCKING_IR_COLLABORATION_RECEIVE, ModeIndex, irDetect.dark);
			break;
	}
}

// Updates the Active and Passive IR values from the IR collaboration
void IRDockingTest_NodeADocking_IRCollaboration_Receive(unsigned char ModeIndex, unsigned char ir_value)
{
	switch (ModeIndex)
	{
		case 0:
			IRDockingTest_NodeADocking_IRCollaboration_Send(++ModeIndex);
			break;
			
		case 1:
			// Left IR LED On
			IRCollaborationData[NODE_B_IR_TOP_WITH_NODE_A_LEFT_LED_ON] = ir_value;
			
			IRDockingTest_NodeADocking_IRCollaboration_Send(++ModeIndex);
			break;
			
		case 2:
			// Left IR LED On
			IRCollaborationData[NODE_B_IR_RIGHT_WITH_NODE_A_LEFT_LED_ON] = ir_value;
			
			IRDockingTest_NodeADocking_IRCollaboration_Send(++ModeIndex);
			break;
			
		case 3:
			// Right IR LED On
			IRCollaborationData[NODE_B_IR_TOP_WITH_NODE_A_RIGHT_LED_ON] = ir_value;
			
			IRDockingTest_NodeADocking_IRCollaboration_Send(++ModeIndex);
			break;
			
		case 4:
			// Right IR LED On
			IRCollaborationData[NODE_B_IR_RIGHT_WITH_NODE_A_RIGHT_LED_ON] = ir_value;
			
			// Enable automatic IR readings
			Variable.Name.inhibitIR = FALSE;
			
			// Update position and locomotion
			IRDockingTest_NodeADocking_UpdateLocomotion();
			break;
	}
}

// Updates a Particle Filter to estimate the relative pose and maneuvers to correct any misalignments
void IRDockingTest_NodeADocking_UpdateLocomotion()
{
	extern PoseParticle BestPoseParticle;
	extern MotionModelXMean[], MotionModelYMean[], MotionModelThetaMean[];
	
	static unsigned char docking_correction_count = 0;
	
	float x_error, y_error, theta_error;
	unsigned long maneuver_wait_time;
	unsigned char relative_direction = LocDirection;
	
	// If Passive IRs are maximized and Active IR is minimized, modules are ready to dock
	if (IRCollaborationData[NODE_A_IR_WITH_NODE_B_LED_ON] >= IR_THRESHOLD_MAX && 
		IRCollaborationData[NODE_B_IR_RIGHT_WITH_NODE_A_LEFT_LED_ON] >= IR_THRESHOLD_MAX &&
		IRCollaborationData[NODE_A_IR_WITH_NODE_B_LED_OFF] < IR_THRESHOLD_MIN)
	{
		// Attempt to dock with the external module
		InhibitDockingRetry = 1;
		
		ConnectDirAP(2, 1);
		SysWait();
		
		InhibitDockingRetry = 0;
		
		// If docking is successful, request all modules to enter IR Docking Complete Mode
		if (NeighborInfo[1].id != 0)
		{
			IRDockingTest_NodeADocking_IRDockingCompleteMode_Send();
			return;
		}
	}
	
	// If neither Passive IR is maximized:
	if (IRCollaborationData[NODE_A_IR_WITH_NODE_B_LED_ON] < IR_THRESHOLD_MAX &&
		IRCollaborationData[NODE_B_IR_RIGHT_WITH_NODE_A_LEFT_LED_ON] < IR_THRESHOLD_MAX)
	{
		// Attempt a limited number of corrective movements
		if (docking_correction_count++ >= MAX_DOCKING_CORRECTION_COUNT)
		{
			docking_correction_count = 0;
			
			// Otherwise, resume IR search
			IRDockingTest_NodeADocking_IRSearchMode_Send();
			return;
		}
	}
	
	// Calculate locomotion direction relative to docking module
	switch (DockingNodeLocID)
	{
		case 2: // Right Node
			switch (LocDirection)
			{
				case FORWARD_DIRECTION: relative_direction = LEFT_DIRECTION; break;
				case LEFT_DIRECTION: relative_direction = BACK_DIRECTION; break;
				case RIGHT_DIRECTION: relative_direction = FORWARD_DIRECTION; break;
				case BACK_DIRECTION: relative_direction = RIGHT_DIRECTION; break;
			}
			break;
		
		case 4: // Left Node
			switch (LocDirection)
			{
				case FORWARD_DIRECTION: relative_direction = RIGHT_DIRECTION; break;
				case LEFT_DIRECTION: relative_direction = FORWARD_DIRECTION; break;
				case RIGHT_DIRECTION: relative_direction = BACK_DIRECTION; break;
				case BACK_DIRECTION: relative_direction = LEFT_DIRECTION; break;
			}
			break;
	}
	
	// Update Particle Filter
	PoseParticlesIteration(relative_direction, DockingManeuverElapsedTime);

	// Calculate alignment error
	x_error = abs(BestPoseParticle.state.x); // in cm
	y_error = abs(BestPoseParticle.state.y); // in cm
	theta_error = abs(BestPoseParticle.state.theta / PI * (180 / 10)); // in deg/10
	
	// If modules are significantly misaligned, calculate best corrective movement
	if ((x_error + theta_error) / y_error > MAX_DOCKING_ALIGNMENT_ERROR)
	{
		if (x_error > theta_error)
		{
			// Correct X error
			if (BestPoseParticle.state.x > 0)
				relative_direction = LEFT_DIRECTION;
			else
				relative_direction = RIGHT_DIRECTION;
		}
		else
		{
			// Correct Theta error
			if (BestPoseParticle.state.theta > 0)
				relative_direction = ROTATE_RIGHT;
			else
				relative_direction = ROTATE_LEFT;
		}
	}
	
	// Calculate global locomotion direction
	switch (DockingNodeLocID)
	{
		case 2: // Right Node
			switch (relative_direction)
			{
				case FORWARD_DIRECTION: LocDirection = RIGHT_DIRECTION; break;
				case LEFT_DIRECTION: LocDirection = FORWARD_DIRECTION; break;
				case RIGHT_DIRECTION: LocDirection = BACK_DIRECTION; break;
				case BACK_DIRECTION: LocDirection = LEFT_DIRECTION; break;
			}
			break;
		
		case 4: // Left Node
			switch (relative_direction)
			{
				case FORWARD_DIRECTION: LocDirection = LEFT_DIRECTION; break;
				case LEFT_DIRECTION: LocDirection = BACK_DIRECTION; break;
				case RIGHT_DIRECTION: LocDirection = FORWARD_DIRECTION; break;
				case BACK_DIRECTION: LocDirection = RIGHT_DIRECTION; break;
			}
			break;
	}
	
	// Calculate the approximate time require to correct misalignment (in seconds)
	switch (LocDirection)
	{
		case FORWARD_DIRECTION:
		case BACK_DIRECTION: 
			maneuver_wait_time = x_error / MotionModelXMean[LocDirection];
			break;
			
		case LEFT_DIRECTION:
		case RIGHT_DIRECTION: 
			maneuver_wait_time = y_error / MotionModelYMean[LocDirection];
			break;
			
		case ROTATE_LEFT:
		case ROTATE_RIGHT: 
			maneuver_wait_time = theta_error / MotionModelThetaMean[LocDirection];
			break;
	}
	
	// Convert wait time to ms
	maneuver_wait_time = abs(maneuver_wait_time * 1000);
	
	// Threshold the maneuver time
	if (maneuver_wait_time > MAX_DOCKING_MANEUVER_WAIT_TIME)
		maneuver_wait_time = MAX_DOCKING_MANEUVER_WAIT_TIME;
	
	// Update direction
	ChangeLocDir(1, LocDirection);
	
	// Start timing
	DockingManeuverElapsedTime = GetTickCount();
	DockingManeuverWaitTime = DockingManeuverElapsedTime + maneuver_wait_time;
	
	// Resume walking
	Send2All('Z', RESUME_LOCOMOTION, 0, 0, 0, 0);
	ResumeLocomotion(); // for me too...	
}

//
// Motion Model Test:
//	

void MotionModelTest_NodeA_Test(unsigned char onoff)
{
	static unsigned long start_motion_time = 0;

	// Resumes walking (and timing)	
	if (onoff)
	{
		Variable.Name.inhibitIR = TRUE;
		start_motion_time = GetTickCount();
		ResumeLocomotion();
	}
	// Stops walking (and timing)
	else
	{
		unsigned long duration_motion_time;
		
		StopLocomotion();
		duration_motion_time = GetTickCount() - start_motion_time;
		AssumeThePosition();
		
		// Output timing
		if (Variable.Name.LocID == 1)
		{
			char buffer[32];
			sprintf(buffer, "Time = %.10fs", ((double)duration_motion_time) / 1000);
			sendStringToMonitor(buffer);
		}
		
		Variable.Name.inhibitIR = FALSE;
	}	
}
