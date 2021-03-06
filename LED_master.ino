#include "LPD8806.h"
#include "SPI.h"
#include <Keypad.h>

//*****************************************************************************
// DECLARING GLOBAL VARIABLES
//*****************************************************************************
boolean audio_sensor = true;
//---------------------------------------------------------------------------------
// SPECTRUM ANALZER VARIABLES
//---------------------------------------------------------------------------------
int SpectrumLeft[7];
int SpectrumRight[7];
byte scaleLow;
byte scaleMid;
byte scaleHigh;
//---------------------------------------------------------------------------------
// LED BOARD DIMENSIONS
//---------------------------------------------------------------------------------
const int numPixels = 1536; // strip.numPixels();
const int cols = 56;
const int rows = 32;
const int NumWindows = 6;
//---------------------------------------------------------------------------------
// MICROCONTROLLER PINS
//---------------------------------------------------------------------------------
//int dataPin = 2;
//int clockPin = 3;
int dataPin = 51;
int clockPin = 52;
int LEDPin = 13;//LED Pin
int capSensePin = 7; //Sensor Pin #1
int capSensePin2 = 6; //Sensor Pin #2
//---------------------------------------------------------------------------------
// LED STRIP OBJECT
//---------------------------------------------------------------------------------
//LPD8806 strip = LPD8806(numPixels, dataPin, clockPin);
LPD8806 strip = LPD8806(numPixels);
//---------------------------------------------------------------------------------
// DEFAULT WINDOW CONTROL ARRAY
//---------------------------------------------------------------------------------
int window_Control[7][3] = {
  0};
int send_first;
int send_last;
int window;
boolean second_input = false;
//---------------------------------------------------------------------------------
// LED GENERAL BEHAVIOR VARIABLES
//---------------------------------------------------------------------------------
int touchedCutoff = 60; //Activation threshold for sensor
int switch_state = 1; // Behavior selection variable
uint8_t audio[3]; //"audio" array. Able to hold 3 audio channel levels.
//---------------------------------------------------------------------------------
// COMMUNICATION VARIABLES
//---------------------------------------------------------------------------------
int incomingByte;
char inChar;
char lastSerialChar; // Holds the last serial character received.
char packet[4];
//---------------------------------------------------------------------------------
// ETCH A SKETCH VARIABLES
//---------------------------------------------------------------------------------
int sensorMax = 1023; // Analog sensors read from 0 - 1023.
long col = 0;
int colPrev = 0;
int colsTot = cols - 1; // Total columns - 1
int row = 0;
int rowPrev = 0;
int rowsTot = rows - 1; // Total rows - 1
int pixelChange = 0; // Tracks if pixelFocus location has changed since last loop.
int pixelPrev = 0;
uint32_t etch_color; // Pixel color.
uint32_t cPrev; // Pixel color during previous loop.`
int pixelFocus = 0; // For tracking location of etch-a-sketch "cursor"
int cursor_location;
boolean etch_interaction_mode = true;
long counter = 0;
boolean pick_etch_color = false;
//---------------------------------------------------------------------------------
// DROPS VARIABLES
//---------------------------------------------------------------------------------
const int Maxdrops = 40;
int drops[Maxdrops][3];// Declare 2D array
int dropNum = 30;
int blank_screen = 0;
int pixel;
int pixel_above;
int blue_true;
int gradual_counter = 0;
int incline = 1;
int decline = 0;
//---------------------------------------------------------------------------------
// STROBE VARIABLES
//---------------------------------------------------------------------------------
boolean is_strobe = false;
int stobe_height = 100;
float strobe_value = 0;
int strobe_length = 1000;
float strobe_time;
float strobe_increment = 2 - 0.5*(strobe_length/1000);
//---------------------------------------------------------------------------------
//AUDIO INPUT VARIABLES
//---------------------------------------------------------------------------------
int readA0,readA1,readA2;
//---------------------------------------------------------------------------------
//KEYPAD VARIABLES
//---------------------------------------------------------------------------------
const byte ROWS = 4; //four rows
const byte COLS = 3; //three columns
//define the symbols on the buttons of the keypads
char hexaKeys[ROWS][COLS] =
{
  {
    '1','2','3'                        }
  ,
  {
    '4','5','6'                        }
  ,
  {
    '7','8','9'                        }
  ,
  {
    '*','0','#'                        }
};
byte rowPins[ROWS] = {
  15, 16, 17, 18}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {
  19, 20, 21}; //connect to the column pinouts of the keypad
//Create the Keypad object
Keypad customKeypad = Keypad( makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);
//Variables used in the keypad to assign individual window behaviors
boolean window_selection_mode; //Select the window
boolean window_behavior_mode; //Select the behavior for that window
//---------------------------------------------------------------------------------
//GEIGER VARIABLES
//---------------------------------------------------------------------------------
boolean geiger_mode = false;
//---------------------------------------------------------------------------------
//MAKER FAIRE HOMAGE VARIABLES
//---------------------------------------------------------------------------------
byte colorSet[numPixels];
//---------------------------------------------------------------------------------
//BUTTON VARIABLES 
//---------------------------------------------------------------------------------
byte pin_button_up = 7;
byte pin_button_down = 8;
byte pin_button_left = 9;
byte pin_button_right = 10;
byte pin_button_player1 = 11;
byte pin_button_player2 = 12;

byte state_button_up = LOW;
byte state_button_down = LOW;
byte state_button_left = LOW;
byte state_button_right = LOW;
byte state_button_player1 = LOW;
byte state_button_player2 = LOW;

//---------------------------------------------------------------------------------
//REFLEX GAME VARIABLES
//---------------------------------------------------------------------------------
byte player_1_score = 0;
byte player_2_score = 0;
byte winning_score = 3;
int react_time_range = 5000;
int react_time;
int timer;
//*****************************************************************************
// SETUP FUNCTION
//*****************************************************************************
void setup() {
  pinMode(LEDPin, OUTPUT);// Start up the LED strip
  digitalWrite(LEDPin, LOW); // Set up the LED
  strip.begin();
  strip.show(); // Update the strip. To start they are all 'off'
  Serial.begin(57600); //Open Serial Communication
  //---------------------------------------------------------------------------------
  // SPECTRUM ANALZER PINS
  //---------------------------------------------------------------------------------
  pinMode(5, OUTPUT); //Setup pins to drive the spectrum analyzer. It needs RESET and STROBE pins.
  pinMode(4, OUTPUT);
  digitalWrite(4,LOW); //pin 4 is strobe on shield
  digitalWrite(5,HIGH); //pin 5 is RESET on the shield
  digitalWrite(4,HIGH);
  digitalWrite(4,LOW);
  digitalWrite(5,LOW);
  //---------------------------------------------------------------------------------
  // DEFAULT WINDOW SIZES
  //---------------------------------------------------------------------------------
  window_Control[0][0] = 0;
  window_Control[0][1] = 255;
  window_Control[1][0] = 256;
  window_Control[1][1] = 511;
  window_Control[2][0] = 512;
  window_Control[2][1] = 767;
  window_Control[3][0] = 768;
  window_Control[3][1] = 1023;
  window_Control[4][0] = 1024;
  window_Control[4][1] = 1279;
  window_Control[5][0] = 1280;
  window_Control[5][1] = 1535;
  //window_Control[6][0] = 1536;
  //window_Control[6][1] = 1791;
  //WINDOWS STATES
  window_Control[0][2] = 7;
  window_Control[1][2] = 8;
  window_Control[2][2] = 8;
  window_Control[3][2] = 8;
  window_Control[4][2] = 8;
  window_Control[5][2] = 7;
  //window_Control[6][2] = 7;

  //---------------------------------------------------------------------------------
  // GEIGER COUNTER INTERRUPT SETUP
  //---------------------------------------------------------------------------------
  attachInterrupt(0, geiger, FALLING );
}

//*****************************************************************************
// MAIN FUNCTION
//*****************************************************************************
void loop() {

  //Check the Keypad for input
  char customKey = customKeypad.getKey();

  if (Serial.available() > 0){
    selectColourMode(); // Returns "switch_state" variable
  }
  else if (customKey){
    KeypadSelectColourMode(customKey); // Returns "switch_state" variable
  }
  selectColourMode(); // Returns "switch_state" variable.
  readAudio(audio); // Calls readAudio function to update 3 audio channel readings.
  scaleAudio(audio); //Scales to Audio channel values
  if ((readCapacitivePin(capSensePin) > touchedCutoff ) || (readCapacitivePin(capSensePin2) > touchedCutoff )) {
    displayColourMode(audio);
  }
  if(is_strobe == true) {
    strobe();
  } // Strobe variable decline timer
}

//*****************************************************************************
// AUDIO FUNCTIONS
//*****************************************************************************

//---------------------------------------------------------------------------------
// READ AUDIO LEVEL
//---------------------------------------------------------------------------------
void readAudio(uint8_t audioLev[]) {
  //Reads audio levels at pins A5, A4, A3.
  for(byte Band=0;Band <7; Band++)
  {
    SpectrumRight[Band] = analogRead(1);
    digitalWrite(4,HIGH); //Strobe pin on the shield
    digitalWrite(4,LOW);
  }

  if(audio_sensor == true){
    audioLev[0]=SpectrumRight[0]/10;
    audioLev[1]=SpectrumRight[3]/10;
    audioLev[2]=SpectrumRight[6]/10;
  }
  else{
    audioLev[0]= uint8_t(random(100,150));
    audioLev[1]= uint8_t(random(60,130));
    audioLev[2]= uint8_t(random(120,145));
  }
}
//---------------------------------------------------------------------------------
// SCALE AUDIO LEVEL
//---------------------------------------------------------------------------------
void scaleAudio(uint8_t audioLev[]) {

  scaleLow = analogRead(A5) / 10;
  scaleMid = analogRead(A4) / 10;
  scaleHigh = analogRead(A3) / 10;

  if (audioLev[0] < scaleLow ) {
    audioLev[0] = 0;
  }
  if (audioLev[1] < scaleMid ) {
    audioLev[1] = 0;
  }
  if (audioLev[2] < scaleHigh) {
    audioLev[2] = 0;
  }
}

//*****************************************************************************
// COLOR BEHAVIOR SELECTION FUNCTIONS
//*****************************************************************************

//---------------------------------------------------------------------------------
// SELECT BEHAVIOR
//---------------------------------------------------------------------------------
//Helper function for selectColourMode()
void modeSelect(int switch_state)
{
  if(second_input == true)
  {
    window_Control[window-1][2] = switch_state;
    second_input = false;
  }
  else
  {
    for (byte i=0; i < NumWindows; i++){
      window_Control[i][2] = switch_state;
    }
  }
}

int selectColourMode() {
  incomingByte = Serial.read();
  lastSerialChar = incomingByte; // Record the last serial char entered

  if(incomingByte >= '0' && incomingByte < '8')
  {
    second_input = true;
    window = incomingByte - '0';
  }
  else if (incomingByte == 'a') {
    modeSelect(1);
  } //Triples
  else if (incomingByte == 'b') {
    modeSelect(2);
  } //Vertical Bar #1
  else if (incomingByte == 'c') {
    modeSelect(3);
  } //Vertical Bar #2
  else if (incomingByte == 'd') {
    modeSelect(5);
  } //White Sparkle
  else if (incomingByte == 'e') {
    modeSelect(7);
  } //Equalizer
  else if (incomingByte == 'o') {
    modeSelect(0);
  } //Maker Faire Sparkle
  else if (incomingByte == 'u') {
    modeSelect(4);
  } //Maker Faire Homage
  else if (incomingByte == 's') {
    modeSelect(8); //Etch and Sketch
    blank_screen = 1;
    pick_etch_color = true;
  }
  else if (incomingByte == ']')
  {
    etch_interaction_mode = !etch_interaction_mode;
  } //Toggle Etch Music Interaction Mode
  else if (incomingByte == '[')
  {
    geiger_mode = !geiger_mode;
  } //Toggle Geiger Counter
  else if (incomingByte == 'p')
  {
    blank_screen = 1; 
    modeSelect(6);
  } //Photographic Film
  else if (incomingByte == 'f') { //Drops
    blank_screen = 1;
    for (byte i=0; i < NumWindows; i++){
      window_Control[i][2] = 10;
    }
  }
  else if (incomingByte == 'l') { //Strobe
    strobe_value = stobe_height;
    strobe_time = millis();
    is_strobe = true;
  }
  else if(inChar =='\0') {
  } //Do nothing
  else { 
    switch_state = 0;
  }
  return switch_state;
}
//---------------------------------------------------------------------------------
// SELECT BEHAVIOR USING THE KEYPAD
//---------------------------------------------------------------------------------
//Helper function for KeypadSelectColourMode()
void modeSelectKeypad(int switch_state)
{
  if(window_behavior_mode == true)
  {
    window_Control[window-1][2] = switch_state;
    window_behavior_mode = false;
  }
  else
  {
    for (byte i=0; i < NumWindows; i++)
    {
      window_Control[i][2] = switch_state;
    }
  }
}

int KeypadSelectColourMode(char incomingByte) {

  lastSerialChar = incomingByte; // Record the last serial char entered

  if(incomingByte == '*')
  {
    window_selection_mode = 1;
    Serial.print("Select a Window\n");
  }

  else if(window_selection_mode == true && incomingByte >= '0' && incomingByte < '8')
  {
    window_behavior_mode = true;
    window = incomingByte - '0';
    window_selection_mode = false;
    //Serial.print("Selected window: ");
    //Serial.println(window);
  }
  else if (incomingByte == '1'){
    modeSelectKeypad(1);
  } //Triples
  else if (incomingByte == '8'){
    modeSelectKeypad(2);
  } //Vertical Band
  else if (incomingByte == '6'){
    modeSelectKeypad(5);
  } //RGB Sparkle
  else if (incomingByte == '2'){
    modeSelectKeypad(7);
  } //Equalizer
  else if (incomingByte == '5'){
    modeSelectKeypad(0);
  } //Maker Faire Sparkle
  else if (incomingByte == '7'){
    modeSelectKeypad(9);
  } //Maker Faire Homage
  else if (incomingByte == '9')
  {
    blank_screen = 1; 
    modeSelectKeypad(6);
  } //Photographic Film
  else if (incomingByte == '#')
  {
    etch_interaction_mode = !etch_interaction_mode;
  } //Toggle Etch Music Interaction Mode
  else if (incomingByte == '0')
  {
    geiger_mode = !geiger_mode;
  } //Toggle Geiger Counter
  else if (incomingByte == '4')
  {
    blank_screen = 1; 
    pick_etch_color = true;
    modeSelectKeypad(8);
  } //Etch and Sketch
  else if (incomingByte == '3') { //Drops
    blank_screen = 1;
    for (byte i=0; i < NumWindows; i++){
      window_Control[i][2] = 10;
    }
  }
  else if(inChar =='\0') {
  } //Do nothing
  else {
    switch_state = 0;
  }
  return switch_state;
}
//---------------------------------------------------------------------------------
// WINDOW BEHAVIORS
//---------------------------------------------------------------------------------
void displayColourMode(uint8_t audioLev[])
{
  for (byte i=0; i < NumWindows; i++)
  {
    send_first = window_Control[i][0];
    send_last = window_Control[i][1];
    //Serial.print("First: ");
    //Serial.println(send_first);
    int add = -1;

    for( int j = i; j < NumWindows; j++){
      if(window_Control[i][2] == window_Control[j][2]){
        send_last = window_Control[j][1];
        add ++;
      }
      else{
        break;
      }
    }
    // Serial.print("First: ");
    // Serial.println(send_first);
    // Serial.print("Last: ");
    // Serial.println(send_last);
    // Serial.print("State: ");
    // Serial.println(window_Control[i][2]);
    ColourModes(window_Control[i][2], audioLev , send_first , send_last);
    i= i + add;
    // Serial.print("I: ");
    // Serial.println(i);
  }
  strip.show();
}
//---------------------------------------------------------------------------------
// COLOR BEHAVIORS
//---------------------------------------------------------------------------------
void ColourModes(int switch_state, uint8_t audioLev[], int first, int last ) {
  //---------------------------------------------------------------------------------
  // Triples Behavior
  //---------------------------------------------------------------------------------
  if (switch_state == 1) {//Triples Behavior
    Triples(first,last,audioLev);
  }
  //---------------------------------------------------------------------------------
  // Vertical RGB bands
  //---------------------------------------------------------------------------------
  else if (switch_state == 2) {//Vertical RGB bands
    Vertical_Band(first,last/3, 'R', audioLev);
    Vertical_Band(last/3, (last*2)/3, 'G', audioLev);
    Vertical_Band((last*2)/3, last, 'B', audioLev);
  }
  else if (switch_state == 3) {//Vertical RGB bands
    Vertical_Band(first,last/3, 'B', audioLev);
    Vertical_Band(last/3, (last*2)/3, 'R', audioLev);
    Vertical_Band((last*2)/3, last, 'G', audioLev);
  }
  //---------------------------------------------------------------------------------
  // RANDOM WHITE SPARKLE
  //---------------------------------------------------------------------------------
  else if (switch_state == 5) {
    Sparkle(first,last);
  }
  //---------------------------------------------------------------------------------
  // MAKER FAIRE SPARKLE
  //---------------------------------------------------------------------------------
  else if (switch_state == 0) {
    maker_faire(first,last,audioLev);
  }
  //---------------------------------------------------------------------------------
  // PHOTOGRAPHIC FILM
  //---------------------------------------------------------------------------------
  else if (switch_state == 6) {
    photo_film(first,last,audioLev);
  }
  //---------------------------------------------------------------------------------
  // MAKER FAIRE HOMAGE
  //---------------------------------------------------------------------------------
  else if (switch_state == 4) {
    maker_homage(first,last,audioLev);
  }
  //---------------------------------------------------------------------------------
  // EQUALIZER
  //---------------------------------------------------------------------------------
  else if (switch_state == 7) {
    int first_col = (int)(first/rows) + 1;
    int last_col = last/rows +2;
    for(int i = first_col; i <= last_col; i+=3){
      equalizer(i,'R', audioLev[0]);
      equalizer(i+1,'G', audioLev[1]);
      equalizer(i+2,'B', audioLev[2]);
    }
    //---------------------------------------------------------------------------------
    // ETCH AND SKETCH MODE
    //---------------------------------------------------------------------------------
  }
  else if (switch_state == 8) {
    int first_cols = first/32 + 1;
    int last_cols = (last - first + 1)/32 + first_cols - 1;
    etch_Sketch(first_cols,last_cols,audioLev);
  }
  //---------------------------------------------------------------------------------
  // REFLEX GAME
  //---------------------------------------------------------------------------------
  else if (switch_state == 9) {
    reflex_game(first,last,audioLev);
  }
  //---------------------------------------------------------------------------------
  // DROPS MODE
  //---------------------------------------------------------------------------------
  else if (switch_state == 10) {
    rain_Drops(first,last,audioLev);
  }
}

//*****************************************************************************
// HELPER FUNCTIONS
//*****************************************************************************
//---------------------------------------------------------------------------------
// COLOR INCREPTION
//---------------------------------------------------------------------------------
// strips the most significant bits of three color bytes
// and merges them into one byte
byte encode(byte R, byte G, byte B)
{
  return (R & 0xE0) | ((G & 0xE0)>>3) | (B >> 6);
}

// Extracts the R value from an encoded RGB value.
// Add's a half delta
byte decodeR(byte v)
{
  return (v & 0xE0) + 0x10;
}

// Extracts the G value from an encoded RGB value.
// Add's a half delta
byte decodeG(byte v)
{
  return ((v << 3) & 0x10);
}

// Extracts the B value from an encoded RGB value.
// Add's a half delta (larger as more bits are stripped while encoding)
byte decodeB(byte v)
{
  return (v << 6) + 0x20;
}
//---------------------------------------------------------------------------------
// RE-MAP COORDINATE SPACE
//---------------------------------------------------------------------------------
int map_coord(int location)
{
  int new_location;
  if( ((int)(location/rows)) % 2 == 1)
  {
    new_location = ((int)(location/rows))*rows + (rows - location%rows)-1;
    return new_location;
  }
  return location;
}
//---------------------------------------------------------------------------------
// STROBE INCREMENT FUNCTION
//---------------------------------------------------------------------------------
void strobe()
{
  if(millis() - strobe_time > strobe_length)
  {
    strobe_value = 0;
    is_strobe = false;
  }
  else
  {
    if(strobe_value > 0)
    {
      strobe_value = strobe_value - strobe_increment;
    }
  }
}
//---------------------------------------------------------------------------------
// GEIGER STROBE FUNCTION
//---------------------------------------------------------------------------------
//Function has interupt status so it is constantly monitered for input
void geiger()
{
  if(geiger_mode == true)
  {
    //Initiates the Strobe Behavior
    strobe_value = stobe_height;
    strobe_time = millis();
    is_strobe = true;
  }
}

//---------------------------------------------------------------------------------
// WHEEL COLOR SELECTION FUNCTION
//---------------------------------------------------------------------------------
//Input a value 0 to 384 to get a color value.
//The colours are a transition r - g - b - back to r
uint32_t Wheel(uint16_t WheelPos)
{
  byte r, g, b;
  switch(WheelPos / 128)
  {
  case 0:
    r = 127 - WheelPos % 128; // red down
    g = WheelPos % 128; // green up
    b = 0; // blue off
    break;
  case 1:
    g = 127 - WheelPos % 128; // green down
    b = WheelPos % 128; // blue up
    r = 0; // red off
    break;
  case 2:
    b = 127 - WheelPos % 128; // blue down
    r = WheelPos % 128; // red up
    g = 0; // green off
    break;
  }
  return(strip.Color(r,g,b));
}

//---------------------------------------------------------------------------------
// READ CAPACITIVE PIN FUNCTION
//---------------------------------------------------------------------------------
// readCapacitivePin
// Input: Arduino pin number
// Output: A number, from 0 to 17 expressing
// how much capacitance is on the pin
// When you touch the pin, or whatever you have
// attached to it, the number will get higher
// In order for this to work now,
// The pin should have a 1+Megaohm resistor pulling
// it up to +5v.

uint8_t readCapacitivePin(int pinToMeasure){
  // This is how you declare a variable which
  // will hold the PORT, PIN, and DDR registers
  // on an AVR
  volatile uint8_t* port;
  volatile uint8_t* ddr;
  volatile uint8_t* pin;
  // Here we translate the input pin number from
  // Arduino pin number to the AVR PORT, PIN, DDR,
  // and which bit of those registers we care about.
  byte bitmask;
  if ((pinToMeasure >= 0) && (pinToMeasure <= 7)){
    port = &PORTD;
    ddr = &DDRD;
    bitmask = 1 << pinToMeasure;
    pin = &PIND;
  }
  if ((pinToMeasure > 7) && (pinToMeasure <= 13)){
    port = &PORTB;
    ddr = &DDRB;
    bitmask = 1 << (pinToMeasure - 8);
    pin = &PINB;
  }
  if ((pinToMeasure > 13) && (pinToMeasure <= 19)){
    port = &PORTC;
    ddr = &DDRC;
    bitmask = 1 << (pinToMeasure - 13);
    pin = &PINC;
  }
  // Discharge the pin first by setting it low and output
  *port &= ~(bitmask);
  *ddr |= bitmask;
  delay(1);
  // Make the pin an input WITHOUT the internal pull-up on
  *ddr &= ~(bitmask);
  // Now see how long the pin to get pulled up
  int cycles = 16000;
  for(int i = 0; i < cycles; i++){
    if (*pin & bitmask){
      cycles = i;
      break;
    }
  }
  // Discharge the pin again by setting it low and output
  // It's important to leave the pins low if you want to
  // be able to touch more than 1 sensor at a time - if
  // the sensor is left pulled high, when you touch
  // two sensors, your body will transfer the charge between
  // sensors.
  *port &= ~(bitmask);
  *ddr |= bitmask;

  return cycles;
}

//*****************************************************************************
// COLOR BEHAVIOR FUNCTIONS
//*****************************************************************************

//---------------------------------------------------------------------------------
// Triples Color Behavior Function
//---------------------------------------------------------------------------------

void Triples(int first, int last, uint8_t audioLev[])
{
  for (int i=first; i < last ; i+=24) {
    strip.setPixelColor( i , strobe_value, 0, audioLev[0] + strobe_value);
    strip.setPixelColor( i + 3, 0, 0, audioLev[0] + strobe_value);
    strip.setPixelColor( i + 6, 0, 0, audioLev[0] + strobe_value);
    strip.setPixelColor( i + 9, 0, 0, audioLev[0] + strobe_value);
    strip.setPixelColor( i + 12, 0, 0, audioLev[0] + strobe_value);
    strip.setPixelColor( i + 15, 0, 0, audioLev[0] + strobe_value);
    strip.setPixelColor( i + 18, 0, 0, audioLev[0] + strobe_value);
    strip.setPixelColor( i + 21, 0, 0, audioLev[0] + strobe_value);
  }
  for (int i=first+1; i < last; i+=24) {
    strip.setPixelColor( i , audioLev[1] + strobe_value, 0, 0);
    strip.setPixelColor( i + 3, audioLev[1] + strobe_value, 0, 0);
    strip.setPixelColor( i + 6, audioLev[1] + strobe_value, 0, 0);
    strip.setPixelColor( i + 9, audioLev[1] + strobe_value, 0, 0);
    strip.setPixelColor( i + 12, audioLev[1] + strobe_value, 0, 0);
    strip.setPixelColor( i + 15, audioLev[1] + strobe_value, 0, 0);
    strip.setPixelColor( i + 18, audioLev[1] + strobe_value, 0, 0);
    strip.setPixelColor( i + 21, audioLev[1] + strobe_value, 0, 0);
  }
  for (int i=first+2; i < last; i+=24) {
    strip.setPixelColor( i , 0, audioLev[2] + strobe_value , 0);
    strip.setPixelColor( i + 3, 0, audioLev[2] + strobe_value , 0);
    strip.setPixelColor( i + 6, 0, audioLev[2] + strobe_value , 0);
    strip.setPixelColor( i + 9, 0, audioLev[2] + strobe_value , 0);
    strip.setPixelColor( i + 12, 0, audioLev[2] + strobe_value , 0);
    strip.setPixelColor( i + 15, 0, audioLev[2] + strobe_value , 0);
    strip.setPixelColor( i + 18, 0, audioLev[2] + strobe_value , 0);
    strip.setPixelColor( i + 21, 0, audioLev[2] + strobe_value , 0);
  }
}

//---------------------------------------------------------------------------------
// Random White Sparkle
//---------------------------------------------------------------------------------
void Sparkle(int first, int last)
{
  for(int i=first; i < last/4; i++){
    int color = random(0,65);
    strip.setPixelColor(i, strip.Color(color, color, color));
    strip.setPixelColor(last*1/4 + i, strip.Color(color, color, color));
    strip.setPixelColor(last*2/4 + i, strip.Color(color, color, color));
    strip.setPixelColor(last*3/4 + i, strip.Color(color, color, color));
  }
}
//---------------------------------------------------------------------------------
// Blank Screen
//---------------------------------------------------------------------------------
void blank(int first, int last)
{
  for(int i=first; i < last/4; i++){
    strip.setPixelColor(i, 0,0,0);
    strip.setPixelColor(last*1/4 + i, 0,0,0);
    strip.setPixelColor(last*2/4 + i, 0,0,0);
    strip.setPixelColor(last*3/4 + i, 0,0,0);
  }
}

//---------------------------------------------------------------------------------
// Vertical Band
//---------------------------------------------------------------------------------
void Vertical_Band(int first, int last, char color, uint8_t audioLev[])
{
  if(color == 'R')
  {
    for (int i=first; i < last ; i+=4) {
      strip.setPixelColor( i , audioLev[0], 0, 0);
      strip.setPixelColor( i + 1, audioLev[0], 0, 0);
      strip.setPixelColor( i + 2, audioLev[0], 0, 0);
      strip.setPixelColor( i + 3, audioLev[0], 0, 0);
    }
  }
  else if(color == 'G')
  {
    for (int i=first; i < last ; i+=4) {
      strip.setPixelColor( i , 0, audioLev[1] , 0);
      strip.setPixelColor( i + 1, 0, audioLev[1], 0);
      strip.setPixelColor( i + 2, 0, audioLev[1], 0);
      strip.setPixelColor( i + 3, 0, audioLev[1], 0);
    }
  }
  else if(color == 'B')
  {
    for (int i=first; i < last ; i+=4) {
      strip.setPixelColor( i , 0, 0, audioLev[2]);
      strip.setPixelColor( i + 1, 0, 0, audioLev[2]);
      strip.setPixelColor( i + 2, 0, 0, audioLev[2]);
      strip.setPixelColor( i + 3, 0, 0, audioLev[2]);
    }
  }
}
//---------------------------------------------------------------------------------
// Maker Faire Sparkle
//---------------------------------------------------------------------------------
void maker_faire(int first, int last, uint8_t audioLev[])
{
  for (int i = 0; i < 12; i++) { //Multiple iterations so that more color change at the same time
    strip.setPixelColor(random(first,last),127 + strobe_value, strobe_value, strobe_value); //red
    strip.setPixelColor(random(first,last),strobe_value, 100 + strobe_value, strobe_value); //green
    strip.setPixelColor(random(first,last), 20 + strobe_value, 20 + strobe_value, 127 + strobe_value);
    strip.setPixelColor(random(first,last), 127 + strobe_value, 127 + strobe_value, strobe_value); //yellow
    strip.setPixelColor(random(first,last), 127 + strobe_value, strobe_value, 127 + strobe_value); // magenta
    strip.setPixelColor(random(first,last), strobe_value, 40 + strobe_value, strobe_value ); // dark green
    for (int i = 0; i < 10; i++){
      strip.setPixelColor(random(first,last),0,0,0);
      strip.setPixelColor(random(first,last),0,0,0);
      strip.setPixelColor(random(first,last),0,0,0);
    }
  }
}
//---------------------------------------------------------------------------------
// Maker Faire Homage
//---------------------------------------------------------------------------------
void maker_homage(int first, int last, uint8_t audioLev[])
{
  for (int k=first; k<last; k++) {
    int pickRand = random(0,9);

    if (pickRand >= 6) {
      strip.setPixelColor(k,1,1,1);
    }
    else if (pickRand != colorSet[k]) {
      strip.setPixelColor(k,10,10,10); // Switches coloured pixels back to white.
    }
    else {
      if (pickRand == 0) {
        strip.setPixelColor(k,127,0,0);
      }
      if (pickRand == 1) {
        strip.setPixelColor(k,0,100,0);
      }
      if (pickRand == 2) {
        strip.setPixelColor(k,20,20,127);
      }
      if (pickRand == 3) { // yellow
        strip.setPixelColor(k,127,127,0);
      }
      if (pickRand == 4) { // magenta
        strip.setPixelColor(k,127,0,127);
      }
      if (pickRand == 5) { // dark green
        strip.setPixelColor(k,0,40,0);
      }
    }
    colorSet[k] = pickRand;
  }
  strip.show();
  delay(200);




}
//---------------------------------------------------------------------------------
// Photographic Film
//---------------------------------------------------------------------------------
void photo_film(int first, int last, uint8_t audioLev[])
{
  if (blank_screen == 1)
  {
    for (int i=0; i < numPixels; i+=3)
    {
      strip.setPixelColor(i, 0, 0, 0);
      strip.setPixelColor(i+1, 0, 0, 0);
      strip.setPixelColor(i+2, 0, 0, 0);
    }
    blank_screen = 0;
  }
  for (int i = 0; i < 10; i++)
  {
    strip.setPixelColor(random(first,last),0,strobe_value,0);
    strip.setPixelColor(random(first,last),0,strobe_value,0);
    strip.setPixelColor(random(first,last),0,0,0);
  }
}
//---------------------------------------------------------------------------------
// EQUALIZER
//---------------------------------------------------------------------------------

void equalizer(int cols, char color, int audioLevel)
{
  for (int i=(cols-1)*rows; i < (cols)*rows; i=i+1) {
    strip.setPixelColor(i, 0, 0, 0);
  }

  if(color == 'R'){
    for (int i=(cols-1)*rows; i <= (cols-1)*rows + (int)((audioLevel/(128/rows))+0.5); i++) {
      strip.setPixelColor(map_coord(i), audioLevel+strobe_value,strobe_value,strobe_value);
    }
  }
  else if(color == 'G')
  {
    for (int i=(cols-1)*rows; i <= (cols-1)*rows + (int)((audioLevel/(128/rows))+0.5); i++) {
      strip.setPixelColor(map_coord(i), strobe_value,audioLevel+strobe_value,strobe_value);
    }
  }
  else if(color == 'B')
  {
    for (int i=(cols-1)*rows; i <= (cols-1)*rows + (int)((audioLevel/(128/rows))+0.5); i++) {
      strip.setPixelColor(map_coord(i), strobe_value,strobe_value,audioLevel+strobe_value);
    }
  }
}

//---------------------------------------------------------------------------------
// ETCH AND SKETCH MODE
//---------------------------------------------------------------------------------
void etch_Sketch(int first_col, int last_col, uint8_t audioLev[])
{
  if (blank_screen == 1)
  {
    for (int i=0; i < numPixels; i+=3)
    {
      strip.setPixelColor(i, 0, 0, 0);
      strip.setPixelColor(i+1, 0, 0, 0);
      strip.setPixelColor(i+2, 0, 0, 0);
    }
    blank_screen = 0;
  }
  
  while(state_button_player1 == LOW && pick_etch_color == true)
  {
    readA2 = analogRead(A8)/3; // Reads a value from 0 to 1023. Scale to be less than 384.
    etch_color = Wheel(readA2);

    for(int i=0; i < numPixels; i+=3)
    {
      strip.setPixelColor(i,etch_color);
      strip.setPixelColor(i+1,etch_color);
      strip.setPixelColor(i+2, etch_color);
    }
    strip.show();
    state_button_player1 = digitalRead(pin_button_player1);
  }
  pick_etch_color = false;
  state_button_player1 = LOW;



  //Interact to the Music
  if(etch_interaction_mode == true){
    for (int i = (first_col-1)*32; i <= last_col*32 ; i = i + 2)// This need to be changed to only include the desired region
    {
      if( counter % 2 == 0)
      {
        uint32_t packed_color = strip.getPixelColor(i+1);
        uint8_t green = (packed_color >> 16) & 0x7f7f7f;
        uint8_t red = (packed_color >> 8) & 0x7f7f7f;
        uint8_t blue = (packed_color) & 0x7f7f7f;
        strip.setPixelColor(i+1,red * audioLev[0] * strobe_value, green * audioLev[1] * strobe_value, blue * audioLev[2] * strobe_value);
        strip.setPixelColor(i,packed_color);
      }
      else
      {
        uint32_t packed_color = strip.getPixelColor(i);
        uint8_t green = (packed_color >> 16) & 0x7f7f7f;
        uint8_t red = (packed_color >> 8) & 0x7f7f7f;
        uint8_t blue = (packed_color) & 0x7f7f7f;
        strip.setPixelColor(i,red * audioLev[0] * strobe_value, green * audioLev[1] * strobe_value, blue * audioLev[2] * strobe_value);
        strip.setPixelColor(i+1,packed_color);
      }
    }
    counter = counter + 1;
  }

  //Determine the Boundries
  int col_range = last_col - first_col + 1;
  colsTot = (col_range) - 1;
  rowsTot = (rows/2) - 1;

  // Determine column location.
  readA0 = analogRead(A6);
  col = ((long)readA0*((long)colsTot+1))/sensorMax ;
  if (readA0 == sensorMax) {
    col = colsTot;
  }

  // Determine row location.
  readA1 = analogRead(A7);
  row = (readA1*(rowsTot+1))/sensorMax;
  // if (row == (rowsTot + 1)) { // Fix condition that readA0 = 1023.
  if (readA1 == sensorMax) {
    row = rowsTot;
  }

  // Check if pixelFocus has changed its location.
  if ( (col != colPrev) || (row != rowPrev) ) {
    pixelChange = 1;
  }

  // Determine pixel color
  readA2 = analogRead(A8)/3; // Reads a value from 0 to 1023. Scale to be less than 384.
  etch_color = Wheel(readA2);

  // Display the crusor pixel
  pixelFocus = (rowsTot+1)*col+row;
  strip.setPixelColor(map_coord((first_col-1)*rows + pixelFocus*2),etch_color);
  strip.setPixelColor(map_coord((first_col-1)*rows + pixelFocus*2+1),etch_color);

  //Paint the Previous Pixels
  if (pixelChange == 1) {
    pixelPrev = (rowsTot+1)*colPrev+rowPrev;
    strip.setPixelColor(map_coord((first_col-1)*rows + pixelPrev*2),cPrev);
    strip.setPixelColor(map_coord((first_col-1)*rows + pixelPrev*2+1),cPrev);
  }

  if ( ( (millis() % 600) < 300) && (pixelChange != 1) ) {
    strip.setPixelColor(map_coord((first_col-1)*rows + pixelFocus*2),127,127,127);
    strip.setPixelColor(map_coord((first_col-1)*rows + pixelFocus*2+1),127,127,127);
  }

  //Show the Strip
  strip.show();

  //Reset Variables
  colPrev = col;
  rowPrev = row;
  cPrev = etch_color;
  pixelChange = 0; // Reset to check for pixelFocus changing.
}
//---------------------------------------------------------------------------------
// DROPS MODE
//---------------------------------------------------------------------------------

void rain_Drops(int first, int last, uint8_t audioLev[])
{
  if (blank_screen == 1)
  {
    for (int i=0; i < numPixels; i+=3)
    {
      strip.setPixelColor(i, 0, 0, 1);
      strip.setPixelColor(i+1, 0, 0, 1);
      strip.setPixelColor(i+2, 0, 0, 1);
    }
    //Initial Random Array of Dropes
    for ( int i = 0; i < dropNum ; i++)
    {
      drops[i][0] = (int)random(cols); //Random column
      drops[i][1] = (int)random(rows); //Random row
      drops[i][2] = (int)random(3); //Random color channel
    }
    //Ensure that this section only gets executed during the first run
    blank_screen = 0;
  }

  //Refresh the drops as they fall of the screen
  for ( int i= 0; i < dropNum ; i++)
  {
    if (drops[i][1] < -1)//Drop has left the screen
    {
      drops[i][0] = (int)random(0, cols);
      drops[i][1] = rows-1;
    }
    //println("drop cord: (" + drops[i][0] + "," + drops[i][1] + ")" );
  }

  for ( int i= 0; i < dropNum ; i++)
  {
    //Mapping drop from array to LED coordinate space
    pixel = (drops[i][1])*(cols) + drops[i][0];
    pixel_above = (drops[i][1]+1)*(cols) + drops[i][0];
    //strip.setPixelColor(pixel, 0, 0, (audioLev[1]-50) + 5*drops[i][1]);
    if (drops[i][1] == -1)
    {
      strip.setPixelColor(drops[i][0], (0 + strobe_value), (0 + strobe_value), (1 + audioLev[2]+strobe_value));
    }
    else
    {
      strip.setPixelColor(pixel, (strobe_value + 20 + audioLev[0]), (strobe_value + 1 + gradual_counter), strobe_value );
      strip.setPixelColor(pixel_above, strobe_value, strobe_value, (strobe_value + 1 + audioLev[2]));
    }

  }

  //Translate
  if ((millis() % 100) < 50)
  {
    for ( int i= 0; i < dropNum ; i++)
    {
      drops[i][1]--;
    }
    blue_true = 1;
  }

  //Gradual Change counter
  if ((millis() % 300) < 50)
  {
    if(incline == 1 && decline == 0)
    {
      gradual_counter++;
      if(gradual_counter > 50)
      {
        incline = 0;
        decline = 1;
      }
    }
    else if(incline == 0 && decline == 1)
    {
      gradual_counter--;

      if(gradual_counter < 0)
      {
        incline = 1;
        decline = 0;
      }
    }
  }
}

//---------------------------------------------------------------------------------
// REFLEX GAME
//---------------------------------------------------------------------------------

void reflex_game(int first, int last, uint8_t audioLev[])
{

  //int pin_button_up = 7;
  //int pin_button_down = 8;
  //int pin_button_left = 9;
  //int pin_button_right = 10;
  //int pin_button_player1 = 11;
  //int pin_button_player2 = 12;
  //
  //int state_button_up = HIGH;
  //int state_button_down = HIGH;
  //int state_button_left = HIGH;
  //int state_button_right = HIGH;
  //int state_button_player1 = HIGH;
  //int state_button_player2 = HIGH;

  //int react_time_range = 5000;
  //int react_time;
  //int timer;

  //Reading the button states 
  //  state_button_player1 = digitalRead(pin_button_player1);
  //  state_button_player2 = digitalRead(pin_button_player2);

  Serial.println("ENTERED REACTION GAME");
  byte game_over = 0;

  while (game_over == 0)
  {
    //Reset the Screen
    screen_color(first, last, 'N');

    //Set the Reaction Time
    react_time = random(1000,react_time_range);
    //Can't use screen_color function because strip.show() is part of it 
    //and it may introduce to much delay for a fair reaction game.

    for(int i=first; i < last/4; i++){
      strip.setPixelColor(i, strip.Color(100,0, 0));
      strip.setPixelColor(last*1/4 + i, strip.Color(100,0, 0));
      strip.setPixelColor(last*2/4 + i, strip.Color(100,0, 0));
      strip.setPixelColor(last*3/4 + i, strip.Color(100,0, 0));
    }

    //Reaction Trigger
    delay(react_time);
    strip.show();
    Serial.println("HUMAN REACT NOW");

    while(state_button_player1 == LOW && state_button_player2 == LOW)
    {
      state_button_player1 = digitalRead(pin_button_player1);
      state_button_player2 = digitalRead(pin_button_player2);
    }

    //Determine the Round Winner
    if ((state_button_player1 == HIGH) && (state_button_player2 == HIGH))
    {
      Serial.println("ITS A TIE");
    }
    else if(state_button_player1 == HIGH)
    {
      Serial.println("Player 1 Won This round");
      player_1_score = player_1_score + 1;

      for (int blinks = 0 ; blinks < 1; blinks++)
      {
        screen_color(first, last, 'N');
        delay(300);
        screen_color(first, last, 'G');
        delay(500);
      }
    }
    else if(state_button_player2 == HIGH)
    {
      Serial.println("Player 2 Won This round");
      player_2_score = player_2_score + 1;

      for (int blinks = 0 ; blinks < 1; blinks++)
      {
        screen_color(first, last, 'N');
        delay(300);
        screen_color(first, last, 'B');
        delay(500);
      }
    }

    state_button_player1 = LOW;
    state_button_player2 = LOW;
    //Determine the Game Winner

    if(player_1_score == winning_score)
    {
      Serial.println("Player 1 Won the game");
      player_1_score = 0;
      player_2_score = 0;
      game_over = 1;

      for(int number=0; number < 20; number++){     
        for(int i=first; i < last/4; i++){
          int color = random(0,65);
          strip.setPixelColor(i, strip.Color(0, color, 0));
          strip.setPixelColor(last*1/4 + i, strip.Color(0, color, 0));
          strip.setPixelColor(last*2/4 + i, strip.Color(0, color, 0));
          strip.setPixelColor(last*3/4 + i, strip.Color(0, color, 0));
        }
        strip.show();
      }
    }
    else if(player_2_score == winning_score)
    {
      Serial.println("Player 2 Won the game");
      player_1_score = 0;
      player_2_score = 0;
      game_over = 1;

      for(int number=0; number < 20; number++)    { 
        for(int i=first; i < last/4; i++){
          int color = random(0,65);
          strip.setPixelColor(i, strip.Color(0, 0, color));
          strip.setPixelColor(last*1/4 + i, strip.Color(0, 0, color));
          strip.setPixelColor(last*2/4 + i, strip.Color(0, 0, color));
          strip.setPixelColor(last*3/4 + i, strip.Color(0, 0, color));
        }
        strip.show();
      }
    }
  }
  for (byte i=0; i < NumWindows; i++){
    window_Control[i][2] = 1;
  }
}

void screen_color(int first, int last, char input)
{
  if( input == 'R')
  {
    for(int i=first; i < last/4; i++){
      strip.setPixelColor(i, strip.Color(100,0,0));
      strip.setPixelColor(last*1/4 + i, strip.Color(100,0,0));
      strip.setPixelColor(last*2/4 + i, strip.Color(100,0,0));
      strip.setPixelColor(last*3/4 + i, strip.Color(100,0,0));
    }
  }
  else if(input == 'B')
  {
    for(int i=first; i < last/4; i++){
      strip.setPixelColor(i, strip.Color(0,0,100));
      strip.setPixelColor(last*1/4 + i, strip.Color(0,0,100));
      strip.setPixelColor(last*2/4 + i, strip.Color(0,0,100));
      strip.setPixelColor(last*3/4 + i, strip.Color(0,0,100));
    }
  }
  else if (input == 'G')
  {
    for(int i=first; i < last/4; i++){
      strip.setPixelColor(i, strip.Color(0,100,0));
      strip.setPixelColor(last*1/4 + i, strip.Color(0,100,0));
      strip.setPixelColor(last*2/4 + i, strip.Color(0,100,0));
      strip.setPixelColor(last*3/4 + i, strip.Color(0,100,0));
    }
  }
  else if (input == 'N')
  {
    for(int i=first; i < last/4; i++){
      strip.setPixelColor(i, strip.Color(0,0,0));
      strip.setPixelColor(last*1/4 + i, strip.Color(0,0,0));
      strip.setPixelColor(last*2/4 + i, strip.Color(0,0,0));
      strip.setPixelColor(last*3/4 + i, strip.Color(0,0,0));
    }
  }
  strip.show();
}









