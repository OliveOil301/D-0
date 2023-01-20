#include <Arduino.h>
#include <RadioLib.h>

//libraries for the OLED display
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


// Setting up the NRF24L01+ radio transceiver module:
// nRF24 has the following connections:
// CS pin:    10
// IRQ pin:   2
// CE pin:    3
//TODO: Make sure these connections are right
nRF24 radio = new Module(10, 2, 3);
int radioState = 0; //This is used to store the state of the transceiver during various funciton calls - for debugging


//Setting up the OLED Display module:
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

#define OLED_DC     6
#define OLED_CS     7
#define OLED_RESET  8
//TODO: Make sure the connections are right
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT,
  &SPI, OLED_DC, OLED_RESET, OLED_CS);

static const unsigned char PROGMEM D0_bmp[] =
{
0b00000000, 0b00000000, 0b00000000, 0b00000000,
0b00000000, 0b00000000, 0b00000000, 0b00000000,
0b00000000, 0b01111100, 0b00000000, 0b00000000,
0b00111000, 0b11000011, 0b11111000, 0b00000000,
0b00000111, 0b10000000, 0b00000111, 0b11000000,
0b00000000, 0b10000111, 0b11100010, 0b00100000,
0b00000000, 0b10000000, 0b00000100, 0b00100000,
0b00000000, 0b10000000, 0b00000010, 0b00100000,
0b00000111, 0b10000000, 0b00000011, 0b11000000,
0b00111000, 0b11000011, 0b11111100, 0b00000000,
0b00000000, 0b01111101, 0b11110000, 0b00000000,
0b00000000, 0b00000001, 0b11110000, 0b00000000,
0b00000000, 0b00000011, 0b11111000, 0b00000000,
0b00000000, 0b00000011, 0b11111000, 0b00000000,
0b00000000, 0b00000001, 0b10010000, 0b00000000,
0b00000000, 0b00000001, 0b10010000, 0b00000000,
0b00000000, 0b00000011, 0b11110000, 0b00000000,
0b00000000, 0b00001101, 0b10011100, 0b00000000,
0b00000000, 0b00010001, 0b10010010, 0b00000000,
0b00000000, 0b00100001, 0b10010001, 0b00000000,
0b00000000, 0b00100001, 0b10010001, 0b00000000,
0b00000000, 0b01000001, 0b11010000, 0b10000000,
0b00000000, 0b01000011, 0b11110000, 0b10000000,
0b00000000, 0b01000011, 0b11110000, 0b10000000,
0b00000000, 0b01000011, 0b11100000, 0b10000000,
0b00000000, 0b01000001, 0b11000000, 0b10000000,
0b00000000, 0b01000000, 0b00000000, 0b10000000,
0b00000000, 0b00100000, 0b00000001, 0b00000000,
0b00000000, 0b00100000, 0b00000001, 0b00000000,
0b00000000, 0b00010000, 0b00000010, 0b00000000,
0b00000000, 0b00001100, 0b00001100, 0b00000000,
0b00000000, 0b00000011, 0b11110000, 0b00000000
};



//*____________________________________________________________________
//*Controller variables------------------------------------------------
/**
 * These variables store the most recent state of each button, joystick, or potentiometer
 * each loop of the controller, the inputs are checked against these numbers.
 * each loop of the controller, the inputs are checked against these numbers.
 * If there is a difference (if the user has changed something), the 
 * controller will send an updated data packet to the robot
**/

//_______________________
// Left Side------------

// Left Joystick
int LJoystickx = 512;
int LJoysticky = 512;
#define LJoystickDeadzone 10 //The deadzone on the x and y axis
#define LJoystickCenterx 512 //The center position on the x axis
#define LJoystickCentery 512 //The center position on the y axis
bool LJoystickButtonPressed = false;
int LScroll = 0;// Left Scroll Wheel
bool LTriggerPressed = false;// Left Trigger
bool LBumperPressed = false;// Left Bumper
bool LOtherButton = false;// Left button above joystick


//_______________________
// Right Side------------

// Right joystick
int RJoystickx = 512;
int RJoysticky = 512;
#define RJoystickDeadzone 10 //The deadzone on the x and y axis
#define RJoystickCenterx 512 //The center position on the x axis
#define RJoystickCentery 512 //The center position on the y axis
bool RJoystickButtonPressed = false;
int RScroll = 0;// Right Scroll Wheel
bool RTriggerPressed = false;// Right Trigger
bool RBumperPressed = false;// Right Bumper
bool ROtherButton = false;// Right button above joystick

//_______________________
// Other ----------------
bool centerButtonPressed = false;
int controllerState = 0;
// 0 = Driving Mode
// 1 = Pose Mode

//*____________________________________________________________________
//*Screen and Menu variables------------------------------------------------
/**
 * These variables store the states of the screen, battery, and other UI things
**/
enum screenState{
  MAIN_MENU,
  BATTERY_LOW,
  SOUND_SELECTION
};

//*____________________________________________________________________
//*Radio Communication stuff & variables-------------------------------
/**
 * These variables are set up to do the radio communication
**/
struct Remote_Data_Packet
{
  byte LJx;
  byte LJy;
  byte RJx;
  byte RJy;

  byte scrollWheels;// first 4 bits are devoted to the left wheel selected value,
  // last 4 are devoted to the right scroll wheel

  byte buttonsA;
  //The button presses are stored in the following order:
  // Bit 0: LJoystick, 
  // Bit 1: LOther, 
  // Bit 2: LTrigger, 
  // Bit 3: LBumper, 
  // Bit 4: RJoystick, 
  // Bit 5: ROther, 
  // Bit 6: RTrigger, 
  // Bit 7: RBumper
  // Example: 10010110 would mean the following:
  // left joystick, left bumper, right other and right trigger buttons are pressed
  
  byte buttonsB;
  //This stores other buttons and any other functionality that may be added
  // Bit 0: center button
  // Bit 1: n/a
  // Bit 2: n/a
  // Bit 3: n/a
  // Bit 4: n/a 
  // Bit 5: n/a 
  // Bit 6: n/a 
  // Bit 7: n/a


  byte other; 
  // 8 bits devoted to other functionality:
  // Bit 0: Request robot battery level
  // Bit 1: n/a 
  // Bit 2: n/a 
  // Bit 3: n/a 
  // Bit 4: n/a 
  // Bit 5: n/a 
  // Bit 6: n/a 
  // Bit 7: n/a
};

Remote_Data_Packet dataPacket; // The data package that we're going to send



/** Checks if the inputs have changed and updates them if they have
 * Takes no input
 * @return true if something has changed, false if everything is same as before
*/
bool inputsHaveChanged(){

  //Left Joystick:
  if(!(abs(LJoystickx-LJoystickCenterx)<= LJoystickDeadzone) && LJoystickx != dataPacket.LJx){//If the Left joystick x axis isn't within the deadzone and has changed position
    return true;
  }
  if(!(abs(LJoysticky-LJoystickCentery)<= LJoystickDeadzone) && LJoysticky != dataPacket.LJy){//If the Left joystick x axis isn't within the deadzone and has changed position
    return true;
  }

  //Right Joystick:
  if(!(abs(RJoystickx-RJoystickCenterx) <= RJoystickDeadzone) && RJoystickx != dataPacket.RJx){//If the Left joystick x axis isn't within the deadzone and has changed position
    return true;
  }
  if(!(abs(RJoysticky-RJoystickCentery) <= RJoystickDeadzone) && RJoysticky != dataPacket.RJy){//If the Left joystick x axis isn't within the deadzone and has changed position
    return true;
  }

  //Buttons:
  
  //button A byte
  byte currentButtonAPresses = (byte)LJoystickButtonPressed<<7 + (byte)LOtherButton<<6 
  + (byte)LTriggerPressed<<5 + (byte)LBumperPressed<<4 + (byte)RJoystickButtonPressed<<3
  + (byte)ROtherButton<<2 + (byte)RTriggerPressed<<1 + (byte)RBumperPressed;

  if(currentButtonAPresses != dataPacket.buttonsA){
    return true;
  }

  //buttonB byte
  byte currentButtonBPresses = (byte)centerButtonPressed;

  if(currentButtonBPresses != dataPacket.buttonsB){
    return true;
  }

  //Otherwise, everything is the same so we
  return false;
  //because no inputs have changed

}

/** fills the dataPacket object with the cuttent controller inputs so it is ready to send
 * replaces all inputs
 * should only be used if inputsHaveChanged() returns true so the robot isn't spammed with
 * data
 * @return none - the dataPacket object that is filled is a global variable
*/
void fillDataPacket(){
  //Fill Left joystick x,y
  if(abs(LJoystickx-LJoystickCenterx)<= LJoystickDeadzone){//if the joystick is within the deadzone
    dataPacket.LJx = LJoystickCenterx; //just send the center point for that axis
  } else{//if it's not within the deadzone
    dataPacket.LJx = LJoystickx;
  }
  if(abs(LJoysticky-LJoystickCentery)<= LJoystickDeadzone){//if the joystick is within the deadzone
    dataPacket.LJy = LJoystickCentery; //just send the center point for that axis
  } else{//if it's not within the deadzone
    dataPacket.LJy = LJoysticky;
  }

  //Fill Right joystick x,y
  if(abs(RJoystickx-RJoystickCenterx)<= RJoystickDeadzone){//if the joystick is within the deadzone
    dataPacket.RJx = RJoystickCenterx; //just send the center point for that axis
  } else{//if it's not within the deadzone
    dataPacket.RJx = RJoystickx;
  }
  if(abs(RJoysticky-RJoystickCentery)<= RJoystickDeadzone){//if the joystick is within the deadzone
    dataPacket.RJy = RJoystickCentery; //just send the center point for that axis
  } else{//if it's not within the deadzone
    dataPacket.RJy = RJoysticky;
  }

  //Fill buttonsA byte
  dataPacket.buttonsA = (byte)LJoystickButtonPressed<<7 + (byte)LOtherButton<<6 
  + (byte)LTriggerPressed<<5 + (byte)LBumperPressed<<4 + (byte)RJoystickButtonPressed<<3
  + (byte)ROtherButton<<2 + (byte)RTriggerPressed<<1 + (byte)RBumperPressed;

  //Fill buttonsB byte
  dataPacket.buttonsB = (byte)centerButtonPressed;

}

/** sends the data packet to the reciever on the robot using the transciever in the remote
 * @return true if it was sent successfully
*/
bool sendDataPacket(){

}


void setup() {
  // put your setup code here, to run once:


  Serial.begin(9600);

  //*_________________________________________________________________________
  //* Initializing the SSD1306 OLED Display ---------------------------
  Serial.print(F("[SSD1306 OLED] Initializing - Start "));
  if(!display.begin(SSD1306_SWITCHCAPVCC)) {
    Serial.println(F("[SSD1306 OLED] Initializing - Failed"));
    while(true);//Stop the program here since we got an error
  } else{
    Serial.println(F("[SSD1306 OLED] Initializing - Sucessful"));
  }

  display.display();//The display is initialized to display the Adafruit splash screen
  delay(1000);//Show that for 1 second
  display.clearDisplay();
  display.drawBitmap(0, 0, D0_bmp, 32, 32, 1);//Draw D-0
  //Display some text describing the project - a title?
  display.display();

  //*_________________________________________________________________________
  //* Initializing the NRF24L01+ radio module ---------------------------
  Serial.print(F("[NRF24L01+] Initializing - Start "));
  radioState = radio.begin();
  if(radioState == RADIOLIB_ERR_NONE) {//If we don't have an error Initializing
    Serial.println(F("[NRF24L01+] Initializing - Sucessful"));
  } else {
    Serial.print(F("[NRF24L01+] Initializing - Failed: Error code "));
    Serial.println(radioState);
    while(true); //Stop the program here since we got an error
  }

  byte addr[] = {0x01, 0x23, 0x45, 0x67, 0x89};
  Serial.print(F("[nRF24] Setting transmit pipe - Start "));
  radioState = radio.setTransmitPipe(addr);
  if(radioState == RADIOLIB_ERR_NONE) { //If we don't have an error setting the address
    Serial.println(F("[nRF24] Setting transmit pipe - Sucessful"));
  } else {
    Serial.print(F("[nRF24] Setting transmit pipe - Failed: Error code "));
    Serial.println(radioState);
    while(true);//Stop the program here since we got an error
  }

}

void loop() {
  // put your main code here, to run repeatedly:

  //read the inputs and set all our variables

  //Check the inputs against our last packet to see if any have changed

  //update the screen
  //If we're changing the mode, take care of that
  //The sound scroll/button stuff

  
  //if the inputs have changed, send a new packet to the robot




}