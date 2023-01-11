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
int radioState = 0; //This is used to store the state of the transceiver during various funciton calls


//Setting up the OLED Display module:
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

#define OLED_DC     6
#define OLED_CS     7
#define OLED_RESET  8
//TODO: Make sure the connections are right
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT,
  &SPI, OLED_DC, OLED_RESET, OLED_CS);

static const unsigned char PROGMEM logo_bmp[] =
{

};



void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);

  //_________________________________________________________________________
  // initializing the SSD1306 OLED Display ---------------------------

  Serial.print(F("[SSD1306 OLED] Initializing - Start "));
  if(!display.begin(SSD1306_SWITCHCAPVCC)) {
    Serial.println(F("[SSD1306 OLED] Initializing - Failed"));
    while(true);//Stop the program here
  } else{
    Serial.println(F("[SSD1306 OLED] Initializing - Sucessful"));
  }

  display.display();//The display is initialized to display the Adafruit splash screen
  delay(1000);//Show that for 2 seconds
  display.clearDisplay();

  //_________________________________________________________________________
  // Initializing the NRF24L01+ radio module ---------------------------
  Serial.print(F("[NRF24L01+] Initializing - Start "));
  radioState = radio.begin();
  if(radioState == RADIOLIB_ERR_NONE) {//If we don't have an error Initializing
    Serial.println(F("[NRF24L01+] Initializing - Sucessful"));
  } else {
    Serial.print(F("[NRF24L01+] Initializing - Failed: Error code "));
    Serial.println(radioState);
    while(true); //Stop the program here
  }

  byte addr[] = {0x01, 0x23, 0x45, 0x67, 0x89};
  Serial.print(F("[nRF24] Setting transmit pipe - Start "));
  radioState = radio.setTransmitPipe(addr);
  if(radioState == RADIOLIB_ERR_NONE) { //If we don't have an error setting the address
    Serial.println(F("[nRF24] Setting transmit pipe - Sucessful"));
  } else {
    Serial.print(F("[nRF24] Setting transmit pipe - Failed: Error code "));
    Serial.println(radioState);
    while(true);//Stop the program here
  }

}

void loop() {
  // put your main code here, to run repeatedly:
}