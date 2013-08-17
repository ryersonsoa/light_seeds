

#include "LPD8806.h"
#include "SPI.h"

//*****************************************************************************
// DECLARING GLOBAL VARIABLES
//*****************************************************************************
boolean audio_sensor = true ;
//---------------------------------------------------------------------------------
// SPECTRUM ANALZER VARIABLES
//---------------------------------------------------------------------------------
int SpectrumLeft[7];
int SpectrumRight[7];
//---------------------------------------------------------------------------------
// LED BOARD DIMENSIONS
//---------------------------------------------------------------------------------
const int numPixels = 1792; // strip.numPixels();
const int cols = 56;
const int rows = 32;
const int NumWindows = 7;
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
//uint32_t memory[numPixels/2]; //Stores Color of each pixel
//uint32_t memory[1];
//---------------------------------------------------------------------------------
// IMAGE ARRAY VARIABLES
//---------------------------------------------------------------------------------
char memory[numPixels/2];
//char red_values[numPixels];
//char green_values[numPixels];
//char blue_values[numPixels];
//---------------------------------------------------------------------------------
// COMMUNICATION VARIABLES
//---------------------------------------------------------------------------------
int incomingByte;
char inChar;
char lastSerialChar; // Holds the last serial character received.
char packet[4];
//char *adress;
//byte temp;
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
//---------------------------------------------------------------------------------
// DROPS VARIABLES
//---------------------------------------------------------------------------------
const int Maxdrops = 40;
int drops[Maxdrops][3];// Declare 2D array
int dropNum = 30;
int blank_screen = 1;
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
// DYNAMIC VARIABLES
//---------------------------------------------------------------------------------
//byte *test ;

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
  window_Control[6][0] = 1536;
  window_Control[6][1] = 1791;
  //WINDOWS STATES
  window_Control[0][2] = 7;
  window_Control[1][2] = 8;
  window_Control[2][2] = 8;
  window_Control[3][2] = 8;
  window_Control[4][2] = 8;
  window_Control[5][2] = 8;
  window_Control[6][2] = 7;
}

//*****************************************************************************
// MAIN FUNCTION
//*****************************************************************************
void loop() {

  if (Serial.available() > 0) {
    selectColourMode(); // Returns "switch_state" variable.
    readAudio(audio); // Calls readAudio function to update 3 audio channel readings.
    scaleAudio(audio); //Scales to Audio channel values
    if ((readCapacitivePin(capSensePin) > touchedCutoff ) || (readCapacitivePin(capSensePin2) > touchedCutoff )) {
      displayColourMode(audio);
    }
  }
  else { //If now new color mode inputed there is no need to check Serial Input
    readAudio(audio);
    scaleAudio(audio);
    if ((readCapacitivePin(capSensePin) > touchedCutoff ) || (readCapacitivePin(capSensePin2) > touchedCutoff )) {
      displayColourMode(audio);
    }
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
    // Serial.print("Red:");
    // Serial.print(SpectrumRight[0]/10);
    // Serial.print(" Green:");
    // Serial.print(SpectrumRight[3]/10);
    // Serial.print(" Blue:");
    // Serial.println(SpectrumRight[6]/10);
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
  if (audioLev[0] < 15 ) {
    audioLev[0] = 0;
  }
  if (audioLev[1] < 50 ) {
    audioLev[1] = 0;
  }
  if (audioLev[2] < 15) {
    audioLev[2] = 0;
  }
}

//*****************************************************************************
// COLOR BEHAVIOR SELECTION FUNCTIONS
//*****************************************************************************

//---------------------------------------------------------------------------------
// SELECT BEHAVIOR
//---------------------------------------------------------------------------------
int selectColourMode() {
  incomingByte = Serial.read();
  blank_screen = 1;
  lastSerialChar = incomingByte; // Record the last serial char entered
  // if(second_input == true)
  // {
  // selectWindowColourMode(window,incomingByte);
  // }
  if(incomingByte >= '0' && incomingByte < '8')
  {
    second_input = true;
    window = incomingByte - '0';
    //Serial.print("INPUT: ");
    //Serial.println(window);
  }
  else if (incomingByte == 'a') {
    if(second_input == true)
    {
      window_Control[window-1][2] = 1;
      second_input = false;
    }
    else
    {
      for (byte i=0; i < NumWindows; i++){
        window_Control[i][2] = 1;
      }
    }
  }
  else if (incomingByte == 'b') {
    if(second_input == true)
    {
      window_Control[window-1][2] = 2;
      second_input = false;
    }
    else
    {
      for (byte i=0; i < NumWindows; i++){
        window_Control[i][2] = 2;
      }
    }
  }
  else if (incomingByte == 'c') {
    if(second_input == true)
    {
      window_Control[window-1][2] = 3;
      second_input = false;
    }
    else
    {
      for (byte i=0; i < NumWindows; i++){
        window_Control[i][2] = 3;
      }
    }
  }
  else if (incomingByte == 'd') {
    if(second_input == true)
    {
      window_Control[window-1][2] = 5;
      second_input = false;
    }
    else
    {
      for (byte i=0; i < NumWindows; i++){
        window_Control[i][2] = 5;
      }
    }
  }
  else if (incomingByte == 'e') {
    if(second_input == true)
    {
      window_Control[window-1][2] = 7;
      second_input = false;
    }
    else
    {
      for (byte i=0; i < NumWindows; i++){
        window_Control[i][2] = 7;
      }
    }
  }
  else if (incomingByte == 's') {
    if(second_input == true)
    {
      window_Control[window-1][2] = 8;
      second_input = false;
    }
    else
    {
      for (byte i=0; i < NumWindows; i++){
        window_Control[i][2] = 8;
      }
    }
  }
  else if (incomingByte == 'r') {
    if(second_input == true)
    {
      window_Control[window-1][2] = 9;
      second_input = false;
    }
    else
    {
      for (byte i=0; i < NumWindows; i++){
        window_Control[i][2] = 9;
      }
    }
  }
  else if (incomingByte == 'f') {
    for (byte i=0; i < NumWindows; i++){
      window_Control[i][2] = 10;
    }
  }
  else if (incomingByte == 'o') {
    if(second_input == true)
    {
      window_Control[window-1][2] = 0;
      second_input = false;
    }
    else
    {
      for (byte i=0; i < NumWindows; i++){
        window_Control[i][2] = 0;
      }
    }
  }
  else if (incomingByte == 'l') {
    strobe_value = stobe_height;
    strobe_time = millis();
    is_strobe = true;
  }
  // else if (incomingByte == 'l') {
  // strobe_value = stobe_height;
  // strobe_time = millis();
  // is_strobe = true;
  // }
  else if(inChar =='\0') { // Patryk
    //Do nothing
  }
  else {
    switch_state = 0;
  }
  return switch_state;
}

int selectWindowColourMode(int window, int switch_state)
{

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
  // LOAD IMAGE
  //---------------------------------------------------------------------------------
  else if (switch_state == 6) {
    for (int i= first; i < last;i++) {
      //strip.setPixelColor(i, red_values[i], green_values[i], blue_values[i]);
    }
  }
  //---------------------------------------------------------------------------------
  // RANDOM RGB SPARKLE
  //---------------------------------------------------------------------------------
  else if (switch_state == 0) {//RBG Sparkle
    RGB_Sparkle(first,last,audioLev);
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
    strip.setPixelColor( i , strobe_value, strobe_value, audioLev[0] + strobe_value);
    strip.setPixelColor( i + 3, strobe_value, strobe_value, audioLev[0] + strobe_value);
    strip.setPixelColor( i + 6, strobe_value, strobe_value, audioLev[0] + strobe_value);
    strip.setPixelColor( i + 9, strobe_value, strobe_value, audioLev[0] + strobe_value);
    strip.setPixelColor( i + 12, strobe_value, strobe_value, audioLev[0] + strobe_value);
    strip.setPixelColor( i + 15, strobe_value, strobe_value, audioLev[0] + strobe_value);
    strip.setPixelColor( i + 18, strobe_value, strobe_value, audioLev[0] + strobe_value);
    strip.setPixelColor( i + 21, strobe_value, strobe_value, audioLev[0] + strobe_value);
  }
  for (int i=first+1; i < last; i+=24) {
    strip.setPixelColor( i , audioLev[1] + strobe_value, strobe_value, strobe_value);
    strip.setPixelColor( i + 3, audioLev[1] + strobe_value, strobe_value, strobe_value);
    strip.setPixelColor( i + 6, audioLev[1] + strobe_value, strobe_value, strobe_value);
    strip.setPixelColor( i + 9, audioLev[1] + strobe_value, strobe_value, strobe_value);
    strip.setPixelColor( i + 12, audioLev[1] + strobe_value, strobe_value, strobe_value);
    strip.setPixelColor( i + 15, audioLev[1] + strobe_value, strobe_value, strobe_value);
    strip.setPixelColor( i + 18, audioLev[1] + strobe_value, strobe_value, strobe_value);
    strip.setPixelColor( i + 21, audioLev[1] + strobe_value, strobe_value, strobe_value);
  }
  for (int i=first+2; i < last; i+=24) {
    strip.setPixelColor( i , strobe_value, audioLev[2] + strobe_value , strobe_value);
    strip.setPixelColor( i + 3, strobe_value, audioLev[2] + strobe_value , strobe_value);
    strip.setPixelColor( i + 6, strobe_value, audioLev[2] + strobe_value , strobe_value);
    strip.setPixelColor( i + 9, strobe_value, audioLev[2] + strobe_value , strobe_value);
    strip.setPixelColor( i + 12, strobe_value, audioLev[2] + strobe_value , strobe_value);
    strip.setPixelColor( i + 15, strobe_value, audioLev[2] + strobe_value , strobe_value);
    strip.setPixelColor( i + 18, strobe_value, audioLev[2] + strobe_value , strobe_value);
    strip.setPixelColor( i + 21, strobe_value, audioLev[2] + strobe_value , strobe_value);
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
      strip.setPixelColor( i , audioLev[0] + strobe_value, strobe_value, strobe_value);
      strip.setPixelColor( i + 1, audioLev[0] + strobe_value, strobe_value, strobe_value);
      strip.setPixelColor( i + 2, audioLev[0] + strobe_value, strobe_value, strobe_value);
      strip.setPixelColor( i + 3, audioLev[0] + strobe_value, strobe_value, strobe_value);
    }
  }
  else if(color == 'G')
  {
    for (int i=first; i < last ; i+=4) {
      strip.setPixelColor( i , strobe_value, audioLev[1] + strobe_value, strobe_value);
      strip.setPixelColor( i + 1, strobe_value, audioLev[1] + strobe_value, strobe_value);
      strip.setPixelColor( i + 2, strobe_value, audioLev[1] + strobe_value, strobe_value);
      strip.setPixelColor( i + 3, strobe_value, audioLev[1] + strobe_value, strobe_value);
    }
  }
  else if(color == 'B')
  {
    for (int i=first; i < last ; i+=4) {
      strip.setPixelColor( i , strobe_value, strobe_value, audioLev[2] + strobe_value);
      strip.setPixelColor( i + 1, strobe_value, strobe_value, audioLev[2] + strobe_value);
      strip.setPixelColor( i + 2, strobe_value, strobe_value, audioLev[2] + strobe_value);
      strip.setPixelColor( i + 3, strobe_value, strobe_value, audioLev[2] + strobe_value);
    }
  }
}

//---------------------------------------------------------------------------------
// Random RGB Sparkle
//---------------------------------------------------------------------------------
void RGB_Sparkle(int first, int last, uint8_t audioLev[])
{
  for (int i = 0; i < 6; i++) {
    strip.setPixelColor(random(first,last), audioLev[0] + strobe_value, strobe_value, strobe_value);
    strip.setPixelColor(random(first,last), strobe_value, audioLev[1] + strobe_value, strobe_value);
    strip.setPixelColor(random(first,last), strobe_value, strobe_value, audioLev[2] + strobe_value);
    delay(10);
  }
}

//---------------------------------------------------------------------------------
// EQUALIZER
//---------------------------------------------------------------------------------

void equalizer(int cols, char color, int audioLevel)
{
  for (int i=(cols-1)*rows; i < (cols)*rows; i=i+1) {
    strip.setPixelColor(i, strobe_value, strobe_value, strobe_value);
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
  //Interact to the Music
  // for (int i = 0; i < numPixels; i++){
  // //Extracting individual colors
  // int r= decodeR(memory[i]);
  // int g= decodeG(memory[i]);
  // int b= decodeB(memory[i]);
  // //Displaying the color
  // strip.setPixelColor(i, audioLev[0]*r,audioLev[1]*g,audioLev[2]*b);
  // }
  //Determine the Boundries
  int col_range = last_col - first_col + 1;
  colsTot = (col_range) - 1;
  rowsTot = (rows/2) - 1;
  //Serial.println(colsTot);
  // Determine column location.
  readA0 = analogRead(A4);
  col = ((long)readA0*((long)colsTot+1))/sensorMax ;
  if (readA0 == sensorMax) {
    col = colsTot;
  }

  // Determine row location.
  readA1 = analogRead(A3);
  row = (readA1*(rowsTot+1))/sensorMax;
  // if (row == (rowsTot + 1)) { // Fix condition that readA0 = 1023.
  if (readA1 == sensorMax) {
    row = rowsTot;
  }
  //Serial.println(readA1);

  // Check if pixelFocus has changed its location.
  if ( (col != colPrev) || (row != rowPrev) ) {
    pixelChange = 1;
  }

  // Determine pixel color
  readA2 = analogRead(A5)/3; // Reads a value from 0 to 1023. Scale to be less than 384.
  etch_color = Wheel(readA2);

  // Display the crusor pixel
  pixelFocus = (rowsTot+1)*col+row;
  strip.setPixelColor(map_coord((first_col-1)*rows + pixelFocus*2),etch_color);
  strip.setPixelColor(map_coord((first_col-1)*rows + pixelFocus*2+1),etch_color);
  //Saving to Memory the pixel location and the color
  memory[pixelFocus] = encode(etch_color >> 16 & 0xFF, etch_color >> 8 & 0xFF, etch_color >> 0 & 0xFF);
  // Serial.print("Red: ");
  // Serial.println(etch_color >> 16 & 0xFF);
  // Serial.print("Green: ");
  // Serial.println(etch_color >> 8 & 0xFF);
  // Serial.print("Blue: ");
  // Serial.println(etch_color >> 0 & 0xFF);
  // byte test = encode((byte)etch_color >> 16 & 0xFF, (byte)etch_color >> 8 & 0xFF, (byte)etch_color >> 0 & 0xFF);
  // Serial.print("Red decoded: ");
  // Serial.println(decodeR(test));
  // Serial.print("Green decoded: ");
  // Serial.println(decodeG(test));
  // Serial.print("Blue decoded: ");
  // Serial.println(decodeB(test));

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

