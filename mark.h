/***********************************************************************/
/*                                                                     */
/*  FILE        :mark.h                                                */
/*  DATE        :June, 30, 2009                                    	   */
/*  DESCRIPTION :Autonomous Docking Program by Mark Calnon             */
/*  CPU TYPE    :SH7047                                                */
/*                                                                     */
/***********************************************************************/

//
// Bluetooth AT-ZV (command mode) commands
//

#define BT_AT_ESCAPE				0
#define BT_AT_VERSION				1
#define BT_AT_DISCOVERY				2
#define BT_AT_SPPCONNECT			3
#define BT_AT_SPPDISCONNECT			4
#define BT_AT_BYPASS				5
	
//
// Helper commands
//

// Output commands
#define SEND_TO_MONITOR				10

// LED commands
#define TURN_ON_LINK_LED			20
#define TURN_OFF_LINK_LED			21

// IR LED commands
#define TURN_ON_ALL_IR_LEDS			22
#define TURN_ON_ONE_ACTIVE_IR_LED	23
#define TURN_ON_ONE_PASSIVE_IR_LED	24
#define TURN_OFF_ALL_IR_LEDS		25
#define TURN_ON_ALL_ACTIVE_IR_LEDS	26
#define TURN_ON_ALL_PASSIVE_IR_LEDS	27

// IR commands
#define DISPLAY_ONE_IR_ANYWHERE		29	// Used to display IR data from non-neighboring nodes 
#define DISPLAY_ONE_IR				30
#define DISPLAY_ALL_IR				31

// Movement Commands
#define START_LOCOMOTION			120
#define STOP_LOCOMOTION				121
#define RESUME_LOCOMOTION			122
#define ASSUME_THE_POSITION			123

//
// IR Data Collection Test commands
//

#define IR_DATA_TEST_NODEA_INIT_WALKER_NODE_IDS			100		// Initialize Walker Node IDs
#define IR_DATA_TEST_NODEA_INIT_SNAKE_NODE_ID			101		// Initialize Snake Node ID
#define IR_DATA_TEST_NODEA_INIT_COMMANDER_NODE_ID		102		// Initialize Commander Node ID (SND)
#define IR_DATA_TEST_ALL_NODES_INIT_COMMANDER_NODE_ID	103		// Initialize Commander Node ID (RCV)
#define IR_DATA_TEST_NODEA_START_TEST					104		// Begin collecting data
#define IR_DATA_TEST_NODEC_TURN_ON_LED					105		// Turn on/off Snake Node's Face 2 IR LED (0 = Off, 1 = Left, 2 = Right, 3 = All) (RCV)
#define IR_DATA_TEST_NODEA_TURN_ON_LED_CONFIRM			106		// Turn on/off Snake Node's Face 2 IR LED (CNF)
#define IR_DATA_TEST_NODEB_DISPLAY_ALL_IR				107		// Measure and display all Walker Nodes' IRs (RCV)
#define IR_DATA_TEST_NODEA_DISPLAY_ALL_IR_CONFIRM		108		// Measure and display all Walker Nodes' IRs (CNF)

//
// IR Docking Test commands
//

// Initialization
#define IR_DOCKING_TEST_NODEA_COMMANDER_START_TEST						130
#define IR_DOCKING_TEST_NODEA_INIT_COMMANDER_NODE_ID					131
#define IR_DOCKING_TEST_NODEA_INIT_NODE_B_ID							132
#define IR_DOCKING_TEST_NODEA_COMMANDER_IR_AMBIENT_MAX_VALUE_SEND		133
#define IR_DOCKING_TEST_NODEA_IR_AMBIENT_MAX_VALUE_SEND					134
#define IR_DOCKING_TEST_NODEA_COMMANDER_IR_AMBIENT_MAX_VALUE_RECEIVE	135
#define IR_DOCKING_TEST_NODEA_IR_AMBIENT_MAX_VALUE_RECEIVE				136

#define IR_DOCKING_TEST_NODEA_DOCKING_IR_SEARCH_MODE_SEND				137
#define IR_DOCKING_TEST_NODEA_IR_SEARCH_MODE_RECEIVE					138
#define IR_DOCKING_TEST_NODEA_COMMANDER_IR_DOCKING_MODE_SEND			139
#define IR_DOCKING_TEST_NODEA_IR_DOCKING_MODE_RECEIVE					140
#define IR_DOCKING_TEST_NODEA_DOCKING_IR_DOCKING_COMPLETE_MODE_SEND		141
#define IR_DOCKING_TEST_NODEA_IR_DOCKING_COMPLETE_MODE_RECEIVE			142

// Search Mode
#define IR_DOCKING_TEST_NODEA_IR_LED_MAX_VALUE_SEND						143
#define IR_DOCKING_TEST_NODEA_COMMANDER_IR_LED_MAX_VALUE_RECEIVE		144

// Docking Mode
#define IR_DOCKING_TEST_NODEA_DOCKING_INIT_IR_COLLABORATION_SEND		145
#define IR_DOCKING_TEST_NODEA_INIT_IR_COLLABORATION_SEND				146
#define IR_DOCKING_TEST_NODEA_DOCKING_INIT_IR_COLLABORATION_RECEIVE		147
#define IR_DOCKING_TEST_NODEA_DOCKING_IR_COLLABORATION_SEND				148
#define IR_DOCKING_TEST_NODEB_IR_COLLABORATION_RECEIVE				149
#define IR_DOCKING_TEST_NODEA_DOCKING_IR_COLLABORATION_RECEIVE			150
#define IR_DOCKING_TEST_NODEA_DOCKING_UPDATE_LOCOMOTION					151

//
// Bluetooth Modes
//

#define BT_MODE_BYPASS		0	// Normal Operating Mode
#define BT_MODE_COMMAND		1	// Command Mode

//
// Locomotion Directions
//

#define FORWARD_DIRECTION	0
#define LEFT_DIRECTION		1
#define RIGHT_DIRECTION		2
#define BACK_DIRECTION		3
#define ROTATE_LEFT			4
#define ROTATE_RIGHT		5

//
// IR Thresholds
//

#define IR_THRESHOLD			25	// Passive (dark) IR threshold
#define IR_THRESHOLD_MAX		192	// Passive (dark) Maximum IR threshold (for docking)
#define IR_THRESHOLD_MIN		10	// Active (bright) Minimum IR threshold (for docking)
#define OBSTACLE_THRESHOLD		25	// Active (bright) IR threshold
#define OBSTACLE_THRESHOLD_LED	60	// Active (bright) IR threshold (with high LED reading)

//
// IR Docking Modes
//

#define IR_SEARCH_MODE				0		// Searching for an IR LED
#define IR_DOCKING_MODE				1		// Correcting module alignment for docking
#define IR_DOCKING_COMPLETE_MODE	2		// Successfully docked

//
// IR Collaboration Modes
//

#define NODE_A_IR_WITH_NODE_B_LED_ON				0
#define NODE_A_IR_WITH_NODE_B_LED_OFF				1
#define NODE_B_IR_TOP_WITH_NODE_A_LEFT_LED_ON		2
#define NODE_B_IR_RIGHT_WITH_NODE_A_LEFT_LED_ON		3
#define NODE_B_IR_TOP_WITH_NODE_A_RIGHT_LED_ON		4
#define NODE_B_IR_RIGHT_WITH_NODE_A_RIGHT_LED_ON	5

//
// Other Defines
//

#define MAX_DOCKING_CORRECTION_COUNT	8
#define MAX_DOCKING_ALIGNMENT_ERROR		1
#define MAX_DOCKING_MANEUVER_WAIT_TIME	2000 // in ms

//
// Useful Structures
//

union led_id_format {               // Required LED ID format for certain functions
    unsigned char BYTE;             //  Byte Access 
    struct {                    	//  Bit  Access 
        unsigned char upper  :4;  	//    Upper 4 bits
        unsigned char lower  :4;   	//    Lower 4 bits
    } BIT;
};

//
// Global Variables
//

extern unsigned char BTMode;
extern char LocDirection;			// LocomotionÇÃà⁄ìÆï˚å¸ 0:ëO 1:ç∂ 2:âE 3:å„

//
// Callback Functions
//

void ExecuteCANmsgMark(unsigned char *RingData);			// Executes a CAN Message
void ProcessBtAtResponse(unsigned char *ATResponse);		// Processes a Bluetooth AT-ZV response (command mode)

//
// Bluetooth AT-ZV (Command Mode) Commands
//

void BtAtEscape();					// Enters command mode
void BtAtVersion();					// Displays the Zerial interface version number
void BtAtDiscovery();				// Finds and displays all available BT devices
void BtAtSppConnect(int NodeId);	// Connects to an available BT device (by Node ID)
void BtAtSppDisconnect();			// Disconnects from the current BT device
void BtAtBypass();					// Enters bypass mode

//
// Bluetooth Communication Wrapper Functions
//

int SendBtAtCommand(const char *ATCommand);						// Sends a specially formatted AT-ZV command to the Bluetooth module.
void SendToNeighboringModule(unsigned char dest_id, 			// Sends a CAN message to a neighboring external module via BT
	unsigned char a0, unsigned char a1, unsigned char a2,
	unsigned char a3, unsigned char a4, unsigned char a5);
void SendToAnyModule(unsigned char dest_id, unsigned char a0, 	// Sends a CAN message to any external module via BT
	unsigned char a1, unsigned char a2, unsigned char a3);
void ForwardToAnyModule(unsigned char *msg);					// Forwards a CAN message to any external module via BT

//
// Helper functions
//

// IR LED functions
void TurnOnAllIRLeds();							// Turns on all active and passive IR LEDs for the 6 primary faces
void TurnOnAllActiveIRLeds();					// Turns on all active IR LEDs for the 3 primary faces
void TurnOnAllPassiveIRLeds();					// Turns on all passive IR LEDs for the 3 primary faces
void TurnOnOneActiveIRLed(unsigned char led);	// Turns on one active IR LED (0-7)
void TurnOnOnePassiveIRLed(unsigned char led);	// Turns on one passive IR LED (1,2,4,6-7)
void TurnOffAllIRLeds();						// Turns off all active and passive IR LEDs for all faces

// IR Sensor functions
void ReadOneIR(unsigned char face);				// Reads the active (IR LED on) and passive (IR LED off) IR for the specified face
void DisplayOneIR(unsigned char face,			// Reads and displays the IR for the specified face
	unsigned char destID, unsigned char resetIR, unsigned char sendToAnyModule);
void DisplayAllIR(unsigned char destID, 		// Reads and displays the IR for all faces
	unsigned char resetIR);

unsigned char ReturnActiveIRStatus(unsigned char num);	// Reads (and waits) and returns current IR status for the specified Active IR number (0-1: non-face IR; 2-4: face IR)
unsigned char ReturnPassiveIRStatus(unsigned char num);	// Reads (and waits) and returns current IR status for the specified Passive IR number (0-1: non-face IR; 2-7: face IR)
unsigned char ReturnAmbientIRStatus();					// Reads (and waits) and returns the maximum ambient IR status from non-face IRs

// Output Functions
void sendStringToMonitor(char *msg);			// Sends a complete string to the Monitor

// Movement Functions
void StartLocomotion();							// Initializes the locomotion parameters and starts walking (Assumes that LocId has already been assigned for all modules)
void StopLocomotion();							// Stops walking
void ResumeLocomotion();						// Resumes walking
void AssumeThePosition();						// Reconfigures the walker into an ideal shape for IR reading

//
// Private Helper Functions
//

int GetCTS();									// Returns true if currently processing a BT message
unsigned long GetBTAddress(int NodeID);			// Returns the Bluetooth address for the specified Node ID

union led_id_format FaceNumberToLedIds(unsigned char face);	// Returns the formatted LED IDs for the specified Face Number (0-2 = Active, 3-5 = Passive)
union led_id_format LedNumberToLedId(unsigned char led);	// Returns the formatted LED ID for the specified LED number (0-7)

//
// IR Data Collection Test Functions
//

// Application functions:

void IRDataTest_NodeA_InitWalkerNodeIds(unsigned char NodeID1, 					// Initialize Walker Node IDs
	unsigned char NodeID2, unsigned char NodeID3, unsigned char NodeID4);
void IRDataTest_NodeA_InitSnakeNodeId(unsigned char NodeID);					// Initialize Snake Node ID
void IRDataTest_NodeA_InitCommanderNodeId(unsigned char NodeID);				// Initialize Commander Node ID (SND)
void IRDataTest_AllNodes_InitCommanderNodeId(unsigned char NodeID);				// Initialize Commander Node ID (RCV)
void IRDataTest_NodeA_StartTest();												// Begin collecting data
void IRDataTest_NodeC_TurnOnLed(unsigned char ModeIndex);						// Turn on/off Snake Node's Face 1 IR LED (Message1: 0 = Off, 1 = Left, 2 = Right, 3 = All) (RCV)
void IRDataTest_NodeA_TurnOnLedConfirm(unsigned char ModeIndex);				// Turn on/off Snake Node's Face 1 IR LED (CNF)
void IRDataTest_NodeB_DisplayAllIR(unsigned char ModeIndex, 					// Measure and display all Walker Nodes' IRs (SND)
	unsigned char WalkerNodeIndex);
void IRDataTest_NodeA_DisplayAllIRConfirm(unsigned char ModeIndex, 				// Measure and display all Walker Nodes' IRs (CNF)
	unsigned char WalkerNodeIndex);

//
// IR Docking Test Functions
//

// Callback Functions
void LocIrCheckM();						// Custom IR callback function for IR Docking Test
void LocIrCheckM_SearchMode();			// Search Mode IR callback function
void LocIrCheckM_DockingMode();			// Docking Mode IR callback function

// Helper Functions
void IRProcess(unsigned char facing_direction, 		// Uses the current IR data to avoid obstacles and locate IR LEDs
	unsigned char ir_detect_led_only);
int IRDetectLED();									// Returns the IR value (true) if an LED is detected, and 0 (false) otherwise
int IRDetectObstacle();								// Returns true if an obstacle is detected, false otherwise

// Application Functions:

// Initialization
void IRDockingTest_NodeACommander_StartTest(unsigned char commander_node_id, 			// Initializes and begins the IR Docking Test
	unsigned char node_b_id);
void IRDockingTest_NodeA_InitCommanderNodeId(unsigned char NodeID);						// Updates the Commander Node ID
void IRDockingTest_NodeA_InitNodeBID(unsigned char NodeID); 							// Updates the Node B ID
void IRDockingTest_NodeACommander_IRAmbientMaxValue_Send();								// Requests the maximum ambient IR value from all modules
void IRDockingTest_NodeA_IRAmbientMaxValue_Send();										// Forwards the maximum ambient IR value to the Commander Node
void IRDockingTest_NodeACommander_IRAmbientMaxValue_Receive(unsigned char ir_ambient);	// Finds and initializes the maximum ambient IR value for all modules
void IRDockingTest_NodeA_IRAmbientMaxValue_Receive(unsigned char ir_ambient);			// Updates the maximum ambient IR value

void IRDockingTest_NodeADocking_IRSearchMode_Send();									// Requests all modules to enter IR Search Mode
void IRDockingTest_NodeA_IRSearchMode_Receive();										// Updates the IR Docking Test mode to IR Search Mode
void IRDockingTest_NodeACommander_IRDockingMode_Send(unsigned char loc_id);				// Requests all modules to enter IR Docking Mode
void IRDockingTest_NodeA_IRDockingMode_Receive(unsigned char loc_id);					// Updates the IR Docking Test mode to IR Docking Mode
void IRDockingTest_NodeADocking_IRDockingCompleteMode_Send();							// Requests all modules to enter IR Docking Complete Mode
void IRDockingTest_NodeA_IRDockingCompleteMode_Receive();								// Updates the IR Docking Test mode to IR Docking Complete Mode

// Search Mode
void IRDockingTest_NodeA_IRLedMaxValueSend(unsigned char ir_value, 						// Forwards the maximum IR value (Passive) and the direction to the Commander Node
	unsigned char ir_direction);
void IRDockingTest_NodeACommander_IRLedMaxValueReceive(unsigned char loc_id,			// Finds the maximum IR value (Passive) the direction and attempts to maximize the IR value
	unsigned char ir_value, unsigned char ir_direction);

// Docking Mode
void IRDockingTest_NodeADocking_InitIRCollaboration_Send();								// Requests all modules to select a convenient pose for reading IRs
void IRDockingTest_NodeA_InitIRCollaboration_Send(unsigned char docking_node_id);		// Select a convenient pose for reading IRs
void IRDockingTest_NodeADocking_InitIRCollaboration_Receive();							// Confirms that all modules have selected a convenient pose for reading IRs and begins IR Collaboration
void IRDockingTest_NodeADocking_IRCollaboration_Send(unsigned char ModeIndex);			// Collaborates with an external module to read many IR configurations
void IRDockingTest_NodeB_IRCollaboration_Receive(unsigned char ModeIndex, 				// Collaborates with an external module to read many IR configurations
	unsigned char docking_node_id);
void IRDockingTest_NodeADocking_IRCollaboration_Receive(unsigned char ModeIndex, 		// Updates the Active and Passive IR values from the IR collaboration
	unsigned char ir_value);
void IRDockingTest_NodeADocking_UpdateLocomotion();										// Updates a Particle Filter to estimate the relative pose and maneuvers to correct any misalignments

//
// Motion Model Test Functions
//

// Application functions:

void MotionModelTest_NodeA_Test(unsigned char onoff);
