#include <PCF8574.h>

/*  
Arduino Nano script for homebrew multiband SSB/CW transceivers. 
Written by Paul Taylor, VK3HN (https://vk3hn.wordpress.com/) standing on the shoulders of:
  - Przemek Sadowski, SQ9NJE (basic controller script)
  - Jason Mildrum NT7S (si5351 library)
  - too many others to mention (ideas, code snippets). 
  
Targets Ashar Farhan VU2ESE's Arduino Nano/si5351 module (Raduino). 

V2.0  25 Jun 2024 - first version, cloned and stripped down from VK3HN SP_VFO_Controller.ino 

Labels that need to be #define'd for your target radio/rig/project:
  Rotary encoder         {ENCODER_OPTICAL_360, ENCODER_MECHANICAL} 
  Display technology     {DISPLAY_LCD, DISPLAY_OLED}
  Display type           {LCD_20X4, LCD_16X2, LCD_8X2, OLED_128X64}
  Project name           {SP_IV, SP_11, SP_V, SS_EI9GQ, SP_6, SP_7, etc }
  BFO enable             {BFO_ENABLED}
  VSWR meter             {VSWR_METER}
  CW keyer               {CW_KEYER} 
  Straight key           {CW_STRAIGHTKEY}
  Tune mS                {TUNE_MS}            Typical value: 3000mS
  Held button mS         {BUTTON_HELD_MS}     Typical value: 700mS
  Diagnostics on display {DIAGNOSTIC_DISPLAY} 
  VFO/BFO swap between transmit and receive {VFO_BFO_SWAP}
*/

// I2C devices and addresses:
// PCF8574  0x20  // demux
// SD1306   0x3C
// 24LC254T 0x50  // eeprom 
// si5351   0x60


/*
XIAO ESP32-C3 pinouts 
[Function][pin](address in software)
AD0 GPIO2  [1] aka A0 (2) // Encoder-A
AD1 GPIO3  [2] aka A1 (3) // Encoder-B  
AD2 GPIO4  [3] aka A2 (4) // PUSHBUTTONS and KEY
AD3 GPIO5  [4] aka A3 (5) // CONFLICTS, DO NOT USE!
D4  GPIO6  [5]        () // reserve for SDA
D5  GPIO7  [6]        () // reserve for SCL
D6  GPIO21 [7]        () // Reserve for serial
D7  GPIO20 [8]        () // reserve for serial
D8  GPIO8  [9]        () // PDL-DASH
D9  GPIO9  [10]       () // TONE
D10 GPIO10 [11]       (10) // PDL-DOT
3V3        [12]
GND        [13]
5V         [14]

/**************************************************************************
Adafruit OLED 128x32 0.91"OLED
 This example is for a 128x32 pixel display using I2C to communicate
 3 pins are required to interface (two I2C and one reset).
 Written by Limor Fried/Ladyada for Adafruit Industries,
 with contributions from the open source community.
 BSD license, check license.txt for more information
 All text above, and the splash screen below must be
 included in any redistribution.
 **************************************************************************/
//#include <string.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <RotaryEncoder.h>   // by Maattias Hertel http://www.mathertel.de/Arduino/RotaryEncoderLibrary.aspx
#include <si5351.h>     // Etherkit si3531 library from NT7S,  V2.1.4   https://github.com/etherkit/Si5351Arduino 
#include <PCF8574.h>    // pcf8574 library by Rob Tillaart 0.3.2

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 oled128x32(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


byte len_max = 0;
unsigned long smeter_ms; 


// struct for 'VFO parameter set' records -- the parameters that will change with each VFO
typedef struct {
  boolean  active;
  uint32_t vfo;
  uint32_t radix;
} VFOset_type;

#define NBR_VFOS 3

// VFOset_type VFOSet[NBR_VFOS]; // array of band parameter sets

VFOset_type VFOSet[NBR_VFOS] = {
  // First element (index 0)
  {
    .active = true,
    .vfo = 3525700,
    .radix = 100
  },
  // Second element (index 1)
  {
    .active = true,
    .vfo = 7033700,
    .radix = 100
  },
  // Third element (index 2)
  {
    .active = true,
    .vfo = 14062700,
    .radix = 100
  }
};

byte v = 1;                   // index into VFOSet array (representing the current VFO)
// byte v_prev;


Si5351 si5351;                // I2C address defaults to x60 in the NT7S lib

#define ENCODER_A 2    // Use GPIO2 (pin 1)
#define ENCODER_B 3    // Use GPIO3 (pin 2)
RotaryEncoder gEncoder = RotaryEncoder(ENCODER_A, ENCODER_B, RotaryEncoder::LatchMode::FOUR3);
long gEncoderPosition = 0;

const unsigned long int FREQ_DEFAULT =    7035700ULL;
// const unsigned long int FREQ_DEFAULT =  535000ULL;  // bottom of AM band for monitoring on transistor radio
// unsigned long int gFrequency = FREQ_DEFAULT;
unsigned long int  freq_hz; 
unsigned int   gStep = 100;

bool   modeTx = false;  // true when in transmit mode 


// USB/LSB initialisation
#define SIDEBAND_THRESHOLD  10000000ULL  // threshold VFO freq for auto sideband selection: above use USB, below use LSB
volatile uint32_t BFO_USB = 4915600ULL;  
volatile uint32_t BFO_LSB = 4914200ULL;  
volatile uint32_t bfo = BFO_LSB;         // the actual BFO freq for si5351 CLK2, arbitrary set to LSB, reset in main loop  

// PCF8574 demux for IOexpansion
#define PCF8574_MUTE       0
#define PCF8574_ANTSWITCH  1
#define PCF8574_TXENABLE   2
#define PCF8574_SEL_80     3
#define PCF8574_SEL_40     4
#define PCF8574_SEL_20     5
PCF8574 pcf20(0x20);    // I2C address is x20

#define D10_TMP 10  // temp for testing D10/GPIO10/pin 10


// CW paddle and keyer declarations
#define CW_DOT   '.'
#define CW_DASH  '-'
#define CW_SPACE ' '
#define CW_KEY   '*'

#define PIN_PADDLE_KEY 4       // REDFINE this to PUSHBUTTONS and KEY
#define CW_DASHDOT_RATIO 4     // number of dots to a dash
#define CW_SPACEDOT_RATIO 3    // number of dots for a space 
#define CW_TX_OFFSET_HZ 700    // CW transmit offset in CW mode 
byte nbrSpacesChar = 0;        //  nbr spaces to write to CW character stream after a CW character (short pause) 
byte nbrSpacesWord = 0;        //  nbr spaces to write to CW character stream after a CW word (longer pause) 
unsigned int CWDotPeriodMS = 80;  // period of a CW dot in mS 
unsigned long lastCWCharMS = millis();   // time when last CW char was sent  
unsigned long keyUpMS = 0;     // time when last CW char was sent  

#define CW_BREAKIN_DROPOUT_MS 1000  // silent period after which Tx drops out
#define CW_CHAR_SPACE_MS 50         // period after the last paddle closure which marks the end of a CW letter 

#define PBTN_LONG_PRESS_MS  1000  // number of mS of a 'long pess'on a pushbutton 

// Beacon parameters
#define BEACON_PERIOD_MS 15000    // period in mS between beacon transmits
unsigned long lastBeaconTx_ms = millis();    // time since last beacon transmit  
unsigned int bcnMsgNbr = 0;   // counter to alternate between the two beacon messages 
bool beaconMode = false;      // turns the beacon on or off via a pushbutton
bool msgInterrupt = false;    // flag to allow message sends to be interrupted by a key or paddle tap 

// CW message declarations
String morseMsgMem[] = {
                         "...- ...- ...-  -.. .  ...- -.- ...-- .... -.  --.- ..-. .---- .---- -.-. --- .-.-.",
                         "- . ... -  -.. .  ...- -.- ...-- .... -.  ...- -.- ...-- .... -.  .-.-."
                       };


String morseMsgChars[] = {"VVV DE VK3HN QF11CO .",
                           "TEST DE VK3HN VK3HN ."};  

//String morseMsgMem[] = {"-.-. --.-    ... --- - .-  ...- -.- ...-- .... -. -..-. .--.   -.-"};
//                       C    Q     S   O T  A         V   K     3    H  N     /    P      K


char decodeDiDahBuff[128];         // buffer of dits and dahs '.' and '-' that will become a CW character
//byte decodeDiDahBuffSize = 10;
byte decodeDiDahBuffIndex = 0;
//decodeDiDahBuff[0] = '\0';

// char decodeMsgBuff[81];    // buffer of decoded ASCII characters

char msgSentBuff[256];       // buffer for accumulated sent (transmitted) morse message
//byte msgSentBuffSize = 81;
byte msgSentBuffIndex = 0;
//msgSentBuff[0] = '\0';

// char lastChar = ' ';     // the most recently decoded sent CW ASCII char 
char dsplySentCWbuff[9];
byte dsplySentCWbuffIndex = 0;

struct morseCharT {
  char ch[10];
}; 

// map of morse di-dah sequences and prosigns
const char *morseMap[] = {
"-.-.--",   // ! 33 dah-di-dah-di-dah-dah
".-..-.",   // 34 di-dah-di-di-dah-dit
" ",        // # 35
" ",        // $ 36
" ",        // % 37
".-...",    // & 38 di-dah-di-di-dit
".----.",   //  39 di-dah-dah-dah-dah-dit
"-.--.",    // ( 40 dah-di-dah-dah-dit
"-.--.-",   // ) 41 dah-di-dah-dah-di-dah
"-..-",     // * 42 dah-di-di-dah
".-.-.",    // + 43 di-dah-di-dah-dit
"--..--",   // , 44 dah-dah-di-di-dah-dah
"-....-",   // - 45 dah-di-di-di-di-dah
".-.-.-",   // . 46 di-dah-di-dah-di-dah
"-..-.",    // / 47 dah-di-di-dah-dit
"-----",    // 0 48
".----",    // 1 49
"..---",    // 2 50
"...--",    // 3 51
"....-",    // 4 52
".....",    // 5 53
"-....",    // 6 54
"--...",    // 7 55
"---..",    // 8 56
"----.",    // 9 57
"---...",   // : 58 dah-dah-dah-di-di-dit
" ",        // ; 59
" ",        // < 60
"-...-",    // = 61 dah-di-di-di-dah
" ",        // > 62
" ",        // ? 63
".--.-.",   // @ 64 di-dah-dah-di-dah-dit
".-",       // A  65
"-...",       // B
"-.-.",       // C
"-..",       // D
".",       // E
"..-.",       // F
"--.",       // G
"....",       // H
"..",       // I
".---",       // J
"-.-",       // K
".-..",       // L
"--",       // M
"-.",       // N
"---",       // O
".--.",       // P
"--.-",       // Q
".-.",       // R
"...",       // S
"-",       // T
"..-",       // U
"...-",       // V
".--",       // W
"-..-",       // X
"-.--",       // Y
"--..",       // Z  90
"x"           // end of array marker
};

const char *prosignMap[] = {
"-.-.-",    // CT 
".-.-.",    // AR
"...-.-",   // SK
"-...-.-",  // BK
"........",  // ERR
"x"         // end of array marker
};

const char *prosignStr[]   = {
  "CT", 
  "AR",
  "SK",
  "BK",
  "ERR"
};



void refresh_display()
{
  char fb[10];

  oled128x32.clearDisplay();
  delay(1);
  oled128x32.setTextSize(3); // was 3
  oled128x32.setTextColor(SSD1306_WHITE);        // Draw white text
  oled128x32.setCursor(0, 0);

  if(modeTx == true)
  {
    // display sent/decoded CW 
    //for (byte k=0; k<sizeof(dsplySentCWbuff); k++) oled128x32.print(dsplySentCWbuff[k]);
    //oled128x32.print(dsplySentCWbuff); 
    oled128x32.setTextSize(2);  

    byte k = dsplySentCWbuffIndex;
    for (byte j=0; j<sizeof(dsplySentCWbuff); j++) 
    {
      oled128x32.print(dsplySentCWbuff[k]);
      k = (k+1)%sizeof(dsplySentCWbuff);
    }
    oled128x32.display();
  }
  else
  {
    // in receive mode
    freq_hz = VFOSet[v].vfo;
    if(freq_hz > SIDEBAND_THRESHOLD)
    {
      oled128x32.print( freq_hz/1000);
    }
    else
    {
      sprintf(fb, "%d", freq_hz);
      // Serial.print(fb); Serial.println('/');
  
      String s(fb); 
      //Serial.print(s); Serial.println('/');

      oled128x32.print(s.charAt(0));
      oled128x32.print(',');
      for (byte j=1; j<=3; j++) oled128x32.print(s.charAt(j));
      oled128x32.print('.');
      oled128x32.print(s.charAt(4));
      oled128x32.display();
      delay(1);
    };
    // meter simulation 
    byte rnd = random(0, 50); 
    if(rnd < 15) rnd = 15;
    testdrawrect(rnd);      // Draw rectangles (outlines)
    oled128x32.display();
  }
};


void testdrawrect(int16_t len) 
{
  // routine to draw s-meter

 // oled128x32.clearDisplay();

//  for(int16_t i=0; i<oled128x32.height()/2; i+=2) {
    oled128x32.drawRect(0, 26, len, 4, SSD1306_WHITE);
    oled128x32.fillRect(0, 26, len, 4, SSD1306_INVERSE);

    if(len>len_max) len_max = len;
    if( (millis() - smeter_ms) > 500 ) 
    {
      len_max = len_max - 4;
      if (len_max < 10) len_max = 10; 
      smeter_ms = millis(); 
    }
    oled128x32.drawRect(len_max, 26, 2, 4, SSD1306_WHITE);

 //   oled128x32.display(); // Update screen with each newly-drawn rectangle
    delay(1);
}


void setup() {
  Serial.begin(9600);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!oled128x32.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  oled128x32.display();
  delay(1000); // Pause 

  // Clear the buffer
  oled128x32.clearDisplay();

//  testscrolltext();    // Draw scrolling text

//  testdrawbitmap();    // Draw a small bitmap image

  // Invert and restore display, pausing in-between
  oled128x32.invertDisplay(true);
  delay(200);
  oled128x32.invertDisplay(false);
  delay(200);

  smeter_ms = millis(); 
  
  // initialise and start the si5351 clocks

  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);         // If using 27Mhz xtal, put 27000000 instead of 0 (0 is the default xtal freq of 25Mhz)
  si5351.set_correction(132200, SI5351_PLL_INPUT_XO);

  si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
  
  freq_hz = VFOSet[v].vfo;  //=====================
//  set up the VFO on CLK1 
  si5351.set_freq(freq_hz*SI5351_FREQ_MULT, SI5351_CLK0);
  si5351.output_enable(SI5351_CLK0, 1);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_2MA); 

  // set up the Carrier Oscillator (CO) for CW 
  si5351.set_freq(freq_hz*SI5351_FREQ_MULT, SI5351_CLK1);
  si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_8MA);   // reduce this when testing in a transceiver
  si5351.output_enable(SI5351_CLK1, 0);

  // can set up CLK2 as BFO here if needed 


  setupRotaryEncoder();
  delay(100);

  refresh_display();
  //delay(1);

  // start the PCF8574 demux
  if (pcf20.begin() == false) Serial.println("\n!! PCF8574 err");

  pinMode(PIN_PADDLE_KEY, INPUT);

  pinMode(D10_TMP, INPUT_PULLUP);

//  delay(5000);
// sendMsg(0);  // test only  
//  delay(5000);

  //for(byte i=0; i<sizeof(msgSentBuff); i++) msgSentBuff[i] = '\0';  // zero-fill the sent message buffer 
  msgSentBuff[0] = '\0';
  msgSentBuffIndex = 0;
  
  for(byte k=0; k<sizeof(dsplySentCWbuff); k++) dsplySentCWbuff[k] = '\0';
  dsplySentCWbuffIndex = 0;

  Serial.println();
  Serial.println("setup");
}


void setupRotaryEncoder() {
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), checkPosition, CHANGE);
}


void checkPosition()
{
  // This interrupt routine will be called on any change of one of the input signals
  gEncoder.tick(); // just call tick() to check the state
}

void frequencyAdjust(int delta) {
  //Serial.print("Adjust: "); Serial.println(delta);
  VFOSet[v].vfo += (delta * gStep);
  setVfoFrequency(VFOSet[v].vfo);
  refresh_display();
}

void setVfoFrequency(unsigned long int frequency) {
  si5351.set_freq(frequency * SI5351_FREQ_MULT, SI5351_CLK0); //  
  //Serial.print("set frequency: ");  Serial.println(frequency);
  //printSi5351Status();
}


char readPaddleKey()
{
  // read analog GPIO and detect paddle or keyclosure
  // Voltage divider: 5v -- 4k5 -- GPIO -- 2k2 -- paddle_left -- 2k2 -- paddle_right -- 1k from GPIO to key -- GND unused. 
  // projected readings: 
  // all open 4095
  // paddle left 3304-3323
  // paddle right 2130-2173
  // key down 1113-1128
  // GND <100

  u_int v, s[3];
  s[0] = analogRead(PIN_PADDLE_KEY);  
  delay(1); 
  s[1] = analogRead(PIN_PADDLE_KEY);
  delay(1);  
  s[2] = analogRead(PIN_PADDLE_KEY);
  v = (u_int)(s[0] + s[1] + s[2])/3;  
  // Serial.println(v);

  char r = (char)0; 
  if ((v>3000) and (v<4000)) r = CW_DOT;
  if ((v>1800) and (v<3000)) r = CW_DASH;
  if ((v> 500) and (v<1800)) r = CW_KEY;

//  if(r != (char)0) lastCWCharMS = millis(); 
  return r;  
}


void demux(unsigned int i, bool s)
{
//  pcf20.write(i, s);
}

void rx2Tx()
{
  // transition from receive to transmit mode 
  modeTx = true; 
  Serial.print("\nTx ");

  // raise MUTE_HI to mute receiver
  demux(PCF8574_MUTE, 1);

  si5351.output_enable(SI5351_CLK0, 0);  // kill the VFO

  // raise TR_HI to engage antenna switch
  demux(PCF8574_ANTSWITCH, 1);
  delay(5);

  // raise TXENABLE_HI to put DC on PA/bias
  demux(PCF8574_TXENABLE, 1);

  si5351.set_freq((VFOSet[v].vfo - CW_TX_OFFSET_HZ) * SI5351_FREQ_MULT, SI5351_CLK1);  // prime CO clock
}; 


void tx2Rx()
{
  // transition from transmit to receive mode 
  modeTx = false; 
  Serial.print(" Rx");

  // drop TXENABLE_HI to take DC or bias off PA
  demux(PCF8574_TXENABLE, 0);

  // disengage antenna switch
  demux(PCF8574_ANTSWITCH, 0);
  delay(5); 


  // reset MUTE_HI to unmute receiver
  demux(PCF8574_MUTE, 0);

  si5351.output_enable(SI5351_CLK0, 1);  // turn the VFO back on
}; 


void sendCW(char c)
{
  // send a CW character (dot or dash)
  // if(!modeTx) return;
  unsigned int p = 0;  // key down period in mS
  
  if(c == CW_DOT)  p = CWDotPeriodMS;
  if(c == CW_DASH) p = CWDotPeriodMS * CW_DASHDOT_RATIO;
  // send the character and following space

  if(!modeTx) rx2Tx(); 
  Serial.print(c);

  if(c == CW_SPACE) 
    delay(CWDotPeriodMS * CW_SPACEDOT_RATIO);
  else
  {
    delay(CWDotPeriodMS/2);  // dot space after after dot or dash
    si5351.output_enable(SI5351_CLK1, 1); // key down
    delay(p);
    si5351.output_enable(SI5351_CLK1, 0); // key up 
    //   delay(CWDotPeriodMS/2);  // 1/3 dot space after after dot or dash
    decodeDiDahBuff[decodeDiDahBuffIndex] = c;
    if(decodeDiDahBuffIndex < sizeof(decodeDiDahBuff)) decodeDiDahBuffIndex++;
  }; 
  lastCWCharMS = millis(); 
}


void flushMsgSentBuff()
{
  // if incremented index not past buffer length, return
  // otherwise flush buffer (to EEPROM) and reset buffer
  if(msgSentBuffIndex >= sizeof(msgSentBuff)) 
  {
    // flush buffer to EEPROM

    // add code here
    Serial.println();
    for (int j=0; j<sizeof(msgSentBuff); j++) 
    {
      Serial.print(msgSentBuff[j]);  // print out buffer for now
      msgSentBuff[j] = '\0'; 
    }
    Serial.print("~ ");
    msgSentBuffIndex = 0; 
  }
}


void decodeCwChar()
{
  // look up the sequence of ASCII '.' and '-' chars in the morseMap table  
  byte i=0;
  char cbuff[20];
  cbuff[0] = ' '; 

//  Serial.print("["); Serial.print(decodeDiDahBuff); Serial.print(']'); 

  if(sizeof(decodeDiDahBuff) == 0) return; 

  // compare string at morseMap[i] with that which was sent by the operator
  
  while(cbuff[0] != 'x')
  {
    if(strcmp(decodeDiDahBuff, morseMap[i])==0) 
//    if(strCompare(&decodeDiDahBuff, &morseMap[i])) 
    {
      // strings are identical, so use the ordinal value of the index
      Serial.print((char)(i+33));

      dsplySentCWbuff[dsplySentCWbuffIndex] = (char)(i+33); 
      dsplySentCWbuffIndex = (dsplySentCWbuffIndex + 1)%sizeof(dsplySentCWbuff);

    //  if(dsplySentCWbuffIndex == sizeof(dsplySentCWbuff)) dsplySentCWbuffIndex = 0; 

      refresh_display();

      msgSentBuff[msgSentBuffIndex++] = (char)(i+33);  flushMsgSentBuff(); 
      
      //for(int k=0; k<sizeof(decodeDiDahBuff); k++) decodeDiDahBuff[k] = '\0';  decodeDiDahBuffIndex=0;
      return;
    }
    else
    {
      i++; 
      strcpy(cbuff, morseMap[i]);
    }
  };

 // Serial.print(" not in morseMap "); 
//  delay(100); 
  // no match yet, so look into the prosigns
  i=0; 
  while(strlen(prosignMap[i]) != 1)
  {
    //    String currStr(prosignMap[i]);
  //  Serial.print(strlen(decodeDiDahBuff)); Serial.print(' '); 
  //  Serial.print(strlen(prosignMap[i]));   Serial.println(' '); 
    
 //   delay(100); 

    if( (strlen(decodeDiDahBuff) == strlen(prosignMap[i])) and
        (strcmp(decodeDiDahBuff, prosignMap[i])==0))
    {
      // strings match 
//      Serial.print("prosign["); Serial.print(prosignStr[i]); Serial.print("] "); delay (100); 
        Serial.print(prosignStr[i]); Serial.print(' ');

//      strAppend(msgSentBuff[msgSentBuffIndex++], prosignMap[i]); 
      strcat(&msgSentBuff[msgSentBuffIndex], prosignStr[i]);

      msgSentBuffIndex += strlen(prosignStr[i]);  // increment buff index by size of prosignMap string
      flushMsgSentBuff(); 

      msgSentBuff[msgSentBuffIndex++] = ' ';  flushMsgSentBuff(); 

      // copy prosign string to display buffer
      strcpy(cbuff, prosignStr[i]);
      for(byte j=0; j<strlen(prosignStr[i]); j++)
      {
        dsplySentCWbuff[dsplySentCWbuffIndex] = cbuff[j]; 
        dsplySentCWbuffIndex = (dsplySentCWbuffIndex + 1)%sizeof(dsplySentCWbuff);
      }

 //     Serial.print(currStr);
      //for(int k=0; k<sizeof(decodeDiDahBuff); k++) decodeDiDahBuff[k] = '\0'; decodeDiDahBuffIndex=0;
      return;
    }
    else
    {
      i++;
    //  strcpy(cbuff, prosignMap[i]);
    }
  };

  // still not matched, so append a generic err character, but stop them from stacking up
  if(msgSentBuff[msgSentBuffIndex-1] != (char)248)
  {
    Serial.print((char)248);
    msgSentBuff[msgSentBuffIndex++] = (char)248;  flushMsgSentBuff(); 
  }
   
   //for(int k=0; k<sizeof(decodeDiDahBuff); k++) decodeDiDahBuff[k] = '\0'; decodeDiDahBuffIndex=0;
}


void keyDown()
{
  // straight key is down, transmit until it breaks
  if(!modeTx) rx2Tx(); 

  si5351.output_enable(SI5351_CLK1, 1); // key down
  while(readPaddleKey() == CW_KEY) delay(10);
  si5351.output_enable(SI5351_CLK1, 0); // key up

  lastCWCharMS = millis(); 
}


bool sendMsg(byte i)
{
  // sends the message at morseMsgMem[i]
  char c;
  byte n = 0; 
  int j = 0; 
  // Serial.println(morseMsgMem[i].length());
 
  oled128x32.clearDisplay();
  oled128x32.setCursor(0, 0);
  oled128x32.setTextSize(2);  

  while(j < morseMsgMem[i].length())
  {
    //Serial.println(morseMsgMem[i].charAt(j)); 
    c = morseMsgMem[i].charAt(j); 
    sendCW(c);      // send a '.'or '-'or ' '

    // if we are at a ' ' then put the corresponding ASCII char on the display 
    if(c == ' ')
    {
      if(n==20) {  // OLED 128x32 display has 2 lines of 10 chars at text size 2
        oled128x32.clearDisplay();   // when the display fills, wipe it and start again! (this should really do a line scroll)
        oled128x32.setCursor(0, 0);
      };
      oled128x32.print( morseMsgChars[i].charAt(n) );
      oled128x32.display();   // show the scrolling message as we go
      n++;
    };

    // see if a paddle or key is down -- if so, interrupt the message send
    if(readPaddleKey() != (char)0)
    {
      return false;  
    };
    j++;
  };

  Serial.println();
  return true; 
}


void loop() 
{
  // check for change in the rotary encoder
  gEncoder.tick();
  long newEncoderPosition = gEncoder.getPosition();
  if(newEncoderPosition != gEncoderPosition) {
    long encoderDifference = newEncoderPosition - gEncoderPosition;
    gEncoderPosition = newEncoderPosition;
    //  Serial.print(" end diff: ");   Serial.println(encoderDifference);
    frequencyAdjust(encoderDifference);
  };

  refresh_display();  // do this to update the simulated meter

  // check if paddle or key are closed
  char c = readPaddleKey();
  if(c == CW_KEY)
  {
    // straight key closed
    keyDown();
  }
  else if((c == CW_DOT) or (c == CW_DASH))
  {
      // paddle closed
      sendCW(c);
      nbrSpacesChar = 0; 
      nbrSpacesWord = 0; 
  };

  //delay(100);  // slow down loop for testing 

  if(modeTx)  // calculate the mS since last key-down or paddle-close
  {
      keyUpMS = millis() - lastCWCharMS;
  //    Serial.println(keyUpMS);
  }

  if((modeTx) and (nbrSpacesChar < 1) and (keyUpMS > CWDotPeriodMS * 1.4))  // was 1.4
  {
    // insert spaces between letters in the CW character stream
  //  Serial.print(' ');
    nbrSpacesChar++;

    // we have a CW character, so attempt to decode it
    decodeCwChar();
    // flush the buffer
    for(byte j=0; j<sizeof(decodeDiDahBuff); j++) decodeDiDahBuff[j] = '\0';  decodeDiDahBuffIndex=0;
  } 

  if((modeTx) and (nbrSpacesWord < 1) and (keyUpMS > CWDotPeriodMS * 5))
  {
    // insert additional spaces between words in the CW character stream
    Serial.print(" ");
    nbrSpacesWord++;

    msgSentBuff[msgSentBuffIndex++] = ' '; flushMsgSentBuff(); 

    dsplySentCWbuff[dsplySentCWbuffIndex] = ' '; 
    dsplySentCWbuffIndex = (dsplySentCWbuffIndex + 1)%sizeof(dsplySentCWbuff);
  } 

 // if(modeTx and ( millis() - lastCWCharMS) > CW_BREAKIN_DROPOUT_MS)
 if((modeTx) and (keyUpMS > CW_BREAKIN_DROPOUT_MS))
  {
    // time to drop out of transmit mode
    tx2Rx(); 
    nbrSpacesChar = 0;
    nbrSpacesWord = 0;
  //  for(byte j=0; j<sizeof(decodeDiDahBuff); j++) decodeDiDahBuff[j] = '\0';
   
    for(byte k=0; k<sizeof(dsplySentCWbuff); k++) dsplySentCWbuff[k] = '\0';  dsplySentCWbuffIndex = 0;
  }

  // read pushbuttons
  if( digitalRead(D10_TMP) == LOW )
  {
    unsigned long buttonTimer = millis(); 
    while(digitalRead(D10_TMP) == LOW) delay(100);  // spin while button is down

    if( (millis() - buttonTimer) > PBTN_LONG_PRESS_MS)
    {
      // long press, let's toggle the beacon
      beaconMode = !beaconMode;
      lastBeaconTx_ms = millis(); 
      // put something on the display...
      Serial.print("\nBeacon:"); Serial.println(beaconMode);
      delay(1000);
    }
    else 
    {
      //short press, let's increment the band register
      v++;
      if(v == NBR_VFOS) v=0; // wrap around
      Serial.print("New band:");
      Serial.print(v); Serial.print(' '); Serial.println(VFOSet[v].vfo);
      delay(1000);
    }
  };

  // Beacon mode...
  if(beaconMode && ((millis() - lastBeaconTx_ms) > BEACON_PERIOD_MS)) 
  {
    bool b;
    b = sendMsg((bcnMsgNbr++)%2);  // beacon mode, send messages 0 and 1 alternately   
    if(!b) {
      beaconMode = false;     // the message was interrupted, stop beacon
      Serial.print("\nBeacon:"); Serial.println(beaconMode);
    };
    lastBeaconTx_ms = millis(); 
    delay(100); 
  }


}
