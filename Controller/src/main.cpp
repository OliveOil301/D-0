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
int LJoystick1y = 512;
int LJoystickDeadzone = 20;//The deadzone on the x and y axis

int LScroll = 0;// Left Scroll Wheel

bool LTriggerPressed = false;// Left Trigger

bool LOtherButton = false;// Left button above joystick


//_______________________
// Right Side------------

// Right joystick
int RJoystick2x = 512;
int RJoystick2y = 512;
int RJoystickDeadzone = 20;//The deadzone on the x and y axis

int RScroll = 0;// Right Scroll Wheel

bool RTriggerPressed = false;// Right Trigger

bool ROtherButton = false;// Right button above joystick

//_______________________
// Other ----------------
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




}