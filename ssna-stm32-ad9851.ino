#include <SPI.h>
#include <ILI_SdSpi.h>
#include <ILI_SdFatConfig.h>
#include <ILI9341_due_gText.h>
#include <ILI9341_due.h>
#include "./fonts/Arial14.h"
#include <Average.h>

// SSNA STM32 based on http://rheslip.blogspot.com.es/2015/08/the-simple-scalar-network-analyser.html

// multi encoder code with speed up values adapted for STM32-arduino by Matthias Diro

#define MAXENCODERS 2
volatile int encstate[MAXENCODERS];
volatile int encflag[MAXENCODERS];
boolean A_set[MAXENCODERS];
boolean B_set[MAXENCODERS];
volatile int16_t encoderpos[MAXENCODERS];
volatile int  encodertimer = millis(); // acceleration measurement
int encoderpinA[MAXENCODERS] = {15}; // pin array of all encoder A inputs {pin A1, pin B1,...
int encoderpinB[MAXENCODERS] = {16}; // pin array of all encoder B inputs {pin A2, pin B2,...
unsigned int lastEncoderPos[MAXENCODERS];

// timer
#define ENCODER_RATE 1000    // in microseconds; 
HardwareTimer timer(1);

#define TFT_DC 9
#define TFT_CS 10
#define rst  11

#define  DATA   20
#define  CLOCK  22 //pin connections for DDS //w_clk
#define  LOAD   21
#define  RESET  19

int AD8307 = 3;

#define LED 33
#define ENC_BUTTON 17

#define DDS_CLOCK 180000000

// Color set
#define  BLACK           0x0000
#define RED             0xF800
#define GREEN           0x07E0
#define BLUE            0x102E
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0
#define ORANGE          0xFD20
#define GREENYELLOW     0xAFE5
#define DARKGREEN       0x03E0
#define WHITE           0xFFFF


ILI9341_due tft = ILI9341_due(TFT_CS, TFT_DC, rst);

ILI9341_due_gText t1(&tft);
char textBuff[20];
uint16_t  color;
uint16_t  colorFONDO = BLACK;


float adc[320];
int temp[320];
Average<float> ave(320);

#define AVE_COUNT 20
Average<float> ave_adc(AVE_COUNT);

long f_inicial = 100000, f_final = 30000000;

char inputcmd[100];  // serial data input
int cmdindex = 0;

long freq;
boolean cambio_f = 0;
boolean cambio_cursor = 0;

void setup() {


  Serial.begin(9600);
  Serial.println("ILI9341 Test!");

  initEncoders();

  tft.begin();
  tft.setRotation(iliRotation270);
  tft.fillScreen(colorFONDO);
  t1.defineArea(0, 0, 320, 240);
  t1.selectFont(Arial14);

  //ILI9341due NEW
  tft.fillRect(2, 2, 317, 237, BLACK);
  ejes();

  // init dds
  pinMode (DATA,  OUTPUT);
  pinMode (CLOCK, OUTPUT);
  pinMode (LOAD,  OUTPUT);
  pinMode (RESET, OUTPUT);
  AD9851_init();
  AD9851_reset();

  ejes();

  pinMode (LED, OUTPUT);
  digitalWrite(LED, HIGH);

  pinMode (ENC_BUTTON, INPUT_PULLUP);
  digitalWrite(ENC_BUTTON, HIGH);


}

void loop()   // Arduino superloop - where everything gets done
{

  char ch;
  long int temp_cmd;
  static uint8_t enc_button_state = 0;

  // serial command interpreter
  // enter a number to set the frequency, anything else shows power
  while (Serial.available()) {
    ch = (char)Serial.read();
    if (((ch >= '0') && (ch <= '9')) || ((ch >= 'A') && (ch <= 'Z'))) inputcmd[cmdindex++] = ch; // ignore everything other than numbers and uppercase letters, save to command string
    if (ch == '\n') {    // parse command if its a newline
      inputcmd[cmdindex] = 0; // terminate the string
      if ((temp_cmd = atol(inputcmd)) > 0) {
        freq = temp_cmd;
        SetFrequency(freq);
        Serial.println(freq);
      }
      cmdindex = 0; // reset command line
    }
  }

  for (byte counter = 0; counter < MAXENCODERS; counter++)
  {
    if ((lastEncoderPos[counter] != encoderpos[counter])) {
      encflag[counter] = LOW;

      if (cambio_f)
        f_final = f_final + (encoderpos[counter] - lastEncoderPos[counter]) * 100000;
      else
        f_inicial = f_inicial + (encoderpos[counter] - lastEncoderPos[counter]) * 100000;

      //freq = freq + (encoderpos[counter] - lastEncoderPos[counter]) * 100000;
      //SetFrequency(freq);
      lastEncoderPos[counter] = encoderpos[counter];
    }
  }

  tft.setTextColor(YELLOW, BLACK);
  sprintf(textBuff, "F: %d.%03d.%03d  ", freq / 1000000, (freq - freq / 1000000 * 1000000) / 1000,
          freq % 1000 );
  t1.drawString(textBuff, 10, 0);


  tft.setTextColor(WHITE, BLACK);
  sprintf(textBuff, "S: %d.%03d ", f_inicial / 1000000, (f_inicial - f_inicial / 1000000 * 1000000) / 1000 );
  t1.drawString(textBuff, 100, 0);

  sprintf(textBuff, "E: %d.%03d ", f_final / 1000000, (f_final - f_final / 1000000 * 1000000) / 1000 );
  t1.drawString(textBuff, 190, 0);

  int paso = (f_final - f_inicial) / 320;

  sprintf(textBuff, "Sp: %d.%03d ", paso / 1000000, (paso - paso / 1000000 * 1000000) / 1000 );
  t1.drawString(textBuff, 260, 0);

  if (!digitalRead(ENC_BUTTON)) {
    if (enc_button_state == 0) {

      if (cambio_cursor) {
        cambio_cursor = 0;
        graph_cursor();

      }

      else
      { cambio_cursor = 1;
        cambio_f ^= 1;

      }
      enc_button_state = 1;

    }
  }
  else enc_button_state = 0; // flag switch not pressed
  graph();

}

// AD8307 outputs 25mv per db. 0db is 2.1v out
// if we use a 2.56v ref this works out to
// 2.5mv per bit, or 0.1dbm
// the NANO has only 1.1v internal ref or the 5V supply so I used the 3.3v regulator as a ref
// with the 3.38V ref the output will be 3.38V/.025V/1024 or 0.13203125db/bit

float dBm_power( float lectura ) {


  float dBm = 0;

  //  0db = 2.1v = 2706 with 3.3V ref
  // -74dbm = .25v, 1dbm = .025v. if the AD8307 has an offset error we correct that here as well
  dBm = lectura - 2607;
  dBm = dBm * 0.13203125 ;
  return dBm;
}


void graph(void) {

  int paso = (f_final - f_inicial) / 320;
  int maximo, minimo = 0;

  for (int i = 0; i < 320; i++)
  {
    freq = f_inicial + i * paso;
    SetFrequency(freq);
    for (int j = 0; j < AVE_COUNT; ++j )
      ave_adc.push ( (float)analogRead(AD8307)); // average AVE_COUNT raw readings
    adc[i] = ave_adc.mean();
    ave.push(adc[i]);
  }

  for (int i = 4; i < 320; i++)
    tft.drawPixel(i, temp[i]  , BLACK);

  maximo = ave.maximum();
  minimo = ave.minimum();

  for (int i = 4; i < 320; i++)
  {
    temp[i] = map( adc[i], minimo, maximo, 234 , 20);
    tft.drawPixel(i, temp[i]  , CYAN);

  }

}

void graph_cursor(void) {

  int paso = (f_final - f_inicial) / 320;
  int ejeX = 160;
  int diferencia = 0;
  int bucle = 1;
  static uint8_t enc_button_state = 0;

  tft.drawLine(ejeX, 20, ejeX, 234 , YELLOW);

  while (bucle)

  {

    for (byte counter = 0; counter < MAXENCODERS; counter++)
    {
      if ((lastEncoderPos[counter] != encoderpos[counter])) {
        encflag[counter] = LOW;
        tft.drawLine(ejeX, 20, ejeX, 234 , BLACK);
        tft.drawPixel(ejeX, temp[ejeX]  , CYAN);
        diferencia = (encoderpos[counter] - lastEncoderPos[counter]);

        if ( diferencia > 0 ) ejeX++;
        else
          ejeX--;

        if ( ejeX < 4 ) ejeX = 4;
        if ( ejeX > 319 ) ejeX = 319;

        tft.drawLine(ejeX, 20, ejeX, 234 , YELLOW);
        tft.setTextColor(WHITE, BLACK);

        long freq_cursor = ejeX * paso+f_inicial;

        sprintf(textBuff, "F: %d.%03d.%03d  ", freq_cursor / 1000000, ( freq_cursor -  freq_cursor / 1000000 * 1000000) / 1000,
                freq_cursor % 1000 );
        t1.drawString(textBuff, 10, 0);

        tft.setTextColor(GREEN, BLACK);
        sprintf(textBuff, "dBm: %.02f ", dBm_power(adc[ejeX] ));
        t1.drawString(textBuff, 10, 210);

        lastEncoderPos[counter] = encoderpos[counter];
      }
    }

    if (!digitalRead(ENC_BUTTON)) {
      if (enc_button_state == 0) {

        bucle = 0;
        enc_button_state = 1;
        tft.drawLine(ejeX, 20, ejeX, 234 , BLACK);
      }

    }
    else enc_button_state = 0; // flag switch not pressed

  }


}

void ejes() {

  tft.drawLine(1, 1, 1, 239, BLUE);
  tft.drawLine(1, 239, 319, 239, BLUE);
  for (int i = 9; i < 310; i += 10)
    tft.drawLine(i, 235, i, 239, BLUE);
  for (int i = 19; i < 220; i += 10)
    tft.drawLine(1, i, 4, i, BLUE);
}


void SetFrequency(unsigned long frequency)
{

  unsigned long tuning_word;
  tuning_word = (frequency * 4294967296LL) / 180000000LL;
  digitalWrite (LOAD, LOW);

  shiftOut(DATA, CLOCK, LSBFIRST, tuning_word);
  shiftOut(DATA, CLOCK, LSBFIRST, tuning_word >> 8);
  shiftOut(DATA, CLOCK, LSBFIRST, tuning_word >> 16);
  shiftOut(DATA, CLOCK, LSBFIRST, tuning_word >> 24);
  shiftOut(DATA, CLOCK, LSBFIRST, 0x09);

  digitalWrite (LOAD, HIGH);
}

void AD9851_init()
{

  digitalWrite(RESET, LOW);
  digitalWrite(CLOCK, LOW);
  digitalWrite(LOAD, LOW);
  digitalWrite(DATA, LOW);
}

void AD9851_reset()
{
  //reset sequence is:
  // CLOCK & LOAD = LOW
  //  Pulse RESET high for a few uS (use 5 uS here)
  //  Pulse CLOCK high for a few uS (use 5 uS here)
  //  Set DATA to ZERO and pulse LOAD for a few uS (use 5 uS here)

  // data sheet diagrams show only RESET and CLOCK being used to reset the device, but I see no output unless I also
  // toggle the LOAD line here.

  delay(2);
  digitalWrite(CLOCK, HIGH);  //Strobe Clock to get hardware bits D0, D1 and D2 into the input register
  digitalWrite(CLOCK, LOW);
  digitalWrite(LOAD, HIGH);   //And raise Load  to get them into the control register

  //Clear DDS registers and set 6x multiplier mode by write 32 zeros and 0x01
  delay(2);
  digitalWrite(LOAD, LOW);    //Drop the Load to start the programming sequence

  digitalWrite(DATA, LOW);    //Write 32 zeros
  for (int i = 0; i < 32; i++)
  {
    digitalWrite(CLOCK, HIGH);
    digitalWrite(CLOCK, LOW);
    delay(2);
  }
  shiftOut(DATA, CLOCK, LSBFIRST, 0x01);            //Write 0x01 to set 6x Multiplier and complete the init sequence
  digitalWrite(LOAD, HIGH); //Raise the Load lone to cload the DDS registers and complete the process
  /*
    digitalWrite(CLOCK, LOW);
    digitalWrite(LOAD, LOW);

    digitalWrite(RESET, LOW);
    delay(5);
    digitalWrite(RESET, HIGH);  //pulse RESET
    delay(5);
    digitalWrite(RESET, LOW);
    delay(5);

    digitalWrite(CLOCK, LOW);
    delay(5);
    digitalWrite(CLOCK, HIGH);  //pulse CLOCK
    delay(5);
    digitalWrite(CLOCK, LOW);
    delay(5);
    digitalWrite(DATA, LOW);    //make sure DATA pin is LOW

    digitalWrite(LOAD, LOW);
    delay(5);
    digitalWrite(LOAD, HIGH);  //pulse LOAD
    delay(5);
    digitalWrite(LOAD, LOW);*/
  // Chip is RESET now
}

// ********encoder function
void readEncoders() {
  for (byte counter = 0; counter < MAXENCODERS; counter++)
  {
    if ( (gpio_read_bit(PIN_MAP[encoderpinA[counter]].gpio_device, PIN_MAP[encoderpinA[counter]].gpio_bit) ? HIGH : LOW) != A_set[counter] )
    {
      A_set[counter] = !A_set[counter];
      if ( A_set[counter] && !B_set[counter] )
      {
        if (millis() - encodertimer > 3)
          encoderpos[counter] += 1;
        else
          encoderpos[counter] += 10;
      }
      encodertimer = millis();
    }
    if ( (gpio_read_bit(PIN_MAP[encoderpinB[counter]].gpio_device, PIN_MAP[encoderpinB[counter]].gpio_bit) ? HIGH : LOW) != B_set[counter] )
    {
      B_set[counter] = !B_set[counter];
      if ( B_set[counter] && !A_set[counter] )
        if (millis() - encodertimer > 3)
          encoderpos[counter] -= 1;
        else
          encoderpos[counter] -= 10;
      encodertimer = millis();
    }
  }
}

void initEncoders()
{
  encodertimer = millis(); // acceleration measurement
  for (byte counter = 0; counter < MAXENCODERS; counter++)
  {
    encstate[counter] = HIGH;
    encflag[counter] = HIGH;
    A_set[counter] = false;
    B_set[counter] = false;
    encoderpos[counter] = 0;
    pinMode(encoderpinA[counter], INPUT_PULLUP);
    pinMode(encoderpinB[counter], INPUT_PULLUP);
    lastEncoderPos[counter] = 1;
  }
  // timer setup for encoder
  timer.pause();
  timer.setPeriod(ENCODER_RATE); // in microseconds
  timer.setChannel1Mode(TIMER_OUTPUT_COMPARE);
  timer.setCompare(TIMER_CH1, 1);  // Interrupt 1 count after each update
  timer.attachCompare1Interrupt(readEncoders);
  timer.refresh();
  timer.resume();
}

