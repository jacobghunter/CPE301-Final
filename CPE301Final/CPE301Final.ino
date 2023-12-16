#include <dht.h> // for temp sensor
#include <LiquidCrystal.h> // for lcd display
#include <RTClib.h> // for time stuff
#include <Stepper.h> // for stepper motor

// Pins for LEDs
#define RED 47
#define YELLOW 53
#define GREEN 51 
#define BLUE 49  
// define pins for temp sensor and button
#define BUTTON_PIN 2
#define STEPBUTTON_PIN 3
#define DHT11_PIN 52
#define waterPin A15

// fan pins and speed
int speedPin=7;
int dir1=6;
int dir2=5;
int mSpeed=90;

// Real-time clock and stepper objects
RTC_DS1307 rtc;
const int totalSteps = 2038;
Stepper stepper(totalSteps, 8, 10, 9, 11); // first entry is how many steps for full rev, rest are pins

LiquidCrystal lcd(33, 31, 29, 27, 25, 23);  //RS,E,D4,D5,D6,D7

dht DHT;
int state;

// ADC stuff from lab 7
#define RDA 0x80
#define TBE 0x20  

volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;
 
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;


//port E pointers
volatile unsigned char* port_e = (unsigned char*) 0x2E;
volatile unsigned char* ddr_e = (unsigned char*) 0x2D;
volatile unsigned char* pin_e = (unsigned char*) 0x2C;

//PORT B pointers
volatile unsigned char* port_b = (unsigned char*) 0x25;
volatile unsigned char* ddr_b = (unsigned char*) 0x24;
volatile unsigned char* pin_b = (unsigned char*) 0x23;

//PORT L pointers
volatile unsigned char* port_l = (unsigned char*) 0x10B;
volatile unsigned char* ddr_l = (unsigned char*) 0x10A;
volatile unsigned char* pin_l = (unsigned char*) 0x109;


//port G pointers
volatile unsigned char* port_g = (unsigned char*) 0x34;
volatile unsigned char* ddr_g = (unsigned char*) 0x33;
volatile unsigned char* pin_g = (unsigned char*) 0x32;

//port H pointers
volatile unsigned char* port_h= (unsigned char*) 0x102; 
volatile unsigned char* ddr_h = (unsigned char*) 0x101;
volatile unsigned char* pin_h = (unsigned char*) 0x100;

// port K pointers
volatile unsigned char* port_k = (unsigned char*) 0x108; 
volatile unsigned char* ddr_k = (unsigned char*) 0x107;
volatile unsigned char* pin_k = (unsigned char*) 0x106;

void setup() {
  // more adc stuff from lab 7
  U0init(9600);
  adc_init(); 

  state = 0;

  // stepper motor
  *ddr_b |= (0x01<<5); // pinMode(11, OUTPUT); 
  *ddr_b |= (0x01<<4); // pinMode(10, OUTPUT);
  *ddr_h |= (0x01<<6); // pinMode(9, OUTPUT);
  *ddr_h |= (0x01<<5); // pinMode(8, OUTPUT);

  // dc motor with fan
  *ddr_h |= (0x01<<4); // pinMode(speedPin,OUTPUT);
  *ddr_h |= (0x01<<3); // pinMode(dir1,OUTPUT);
  *ddr_e |= (0x01<<3); // pinMode(dir2,OUTPUT);

  // LEDs
  *ddr_b |= (0x01<<0); // pinMode(53, OUTPUT);
  *ddr_b |= (0x01<<2); // pinMode(51, OUTPUT);
  *ddr_l |= (0x01<<0); // pinMode(49, OUTPUT);
  *ddr_l |= (0x01<<2); // pinMode(47, OUTPUT);
  
  // water sensor
  *ddr_k &= ~(0x01<<7); // pinMode(A15, INPUT);

  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);

  // button interrupt setup
  *ddr_e &= ~(0x01<<4); 
  *port_e |= (0x01<<4);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonChangeState, RISING);

  *ddr_e &= ~(0x01<<5); // stepbutton

  stepper.setSpeed(15);
}

int steps = totalSteps/16; // segment stepper steps into 16

void loop() {  
  int chk = DHT.read11(DHT11_PIN);
  int waterLevel = adc_read(waterPin);
  if (state == 0) { // disabled state

    // allows for adjusting of vent position
    if (steps < 0) {
      steps = totalSteps/16 * -1;
    } else {
      steps = totalSteps/16;
    }
    
    if (*pin_e & (0x01<<5)) { // if pin 3 (stepper button) is high
      steps *= -1;
    }

    runFan(0);
    clearLED();
    *port_b |= (0x01<<0); // digitalWrite(YELLOW, HIGH);

  } else {
    if (state == 1) {
      updateLCD();
      if (*pin_e & (0x01<<5)) {
        steps *= -1;
      }
      runFan(0);
      
      timeStamp();
      clearLED();
      *port_b |= (0x01<<2); // turn on green LED
      if (waterLevel < 50) {
        state = 2; // error state
      }
      if (DHT.temperature > 22) {
        state = 3;
      }
    } else if (state == 2) {
            steps = 0;

      lcd.clear();
      lcd.print("Error");
      runFan(0);
      clearLED();
      *port_l |= (0x01<<2); // turn on red LED
    } else if (state == 3) {
      runFan(255); // turn on fan motor
      updateLCD();
      
      if (*pin_e & (0x01<<5)) {
        steps *= -1;
      }
      if (DHT.temperature <= 22) {
        state = 1; //send to idle
      }
      if (waterLevel < 50) {
        state = 2; // error state
      }
      clearLED();
      *port_l |= (0x01<<0); // turn on blue LED
    }
  }
  stepperLoop();
}

void runFan(int speed) {
  *port_h |= (0x01<<4); // analogWrite(speedPin, speed); this didnt work with our setup even with analogWrite so it wasnt worth fully doing
  *port_h |= (0x01<<3); // digitalWrite(dir1, LOW);
  *port_e |= (0x01<<3); // digitalWrite(dir2, HIGH);
  delay(25);
}

// Display temperature and humidity on LCD screen
void lcdScreen(float temp, float Humidity) {
  // set cursor to column 0, line 1
  // line 1 is the second row
  lcd.setCursor(0, 0);
  lcd.print("Humidity: ");
  lcd.print(Humidity, 1);
  lcd.print("%");
  
  lcd.setCursor(0, 1);
  // print the number of seconds
  lcd.print("Fahrenheit: ");
  lcd.print(temp, 1);
}

// Control stepper motor based on analog input
void stepperLoop(){
  stepper.setSpeed(15);
  stepper.step(steps); // move the amount of steps needed to turn by the angle
}

 // Log the time for temperature readings
void timeStamp() {
  DateTime now = rtc.now();
  printString("\nTime: ");
  printString((String)now.month());
  printString("/");
  printString((String)now.month());
  printString(" ");
  int hour = now.hour();
  hour -= 4; //adjusting the time
  printString((String)hour);
  printString(":");
  printString((String)now.minute());
  printString(":");
  printString((String)now.second());
  printString("\n");
}

void updateLCD() {
  lcd.clear();
  lcd.print("Temp: ");
  lcd.print(DHT.temperature);
  lcd.setCursor(0,1);
  lcd.print("Humidity: ");
  lcd.print(DHT.humidity);
}

void buttonChangeState() {
  if (state == 0) {
    state = 1;
  }
  else if (state != 0) {
    state = 0;
  }
}

void clearLED() {
  *port_b &= ~(0x01<<0); // digitalWrite(YELLOW, LOW)
  *port_b &= ~(0x01<<2); // digitalWrite(GREEN, LOW);
  *port_l &= ~(0x01<<0); // digitalWrite(BLUE, LOW);
  *port_l &= ~(0x01<<2); // digitalWrite(RED, LOW);
}

// for printing a string using U0putchar
void printString(String str) {
  for (int i = 0; i < str.length(); i++) {
    U0putchar(str[i]);
  }
}

// All adc stuff from lab 7
void U0init(int U0baud) {
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 // Same as (FCPU / (16 * U0baud)) - 1;
 *myUCSR0A = 0x20;
 *myUCSR0B = 0x18;
 *myUCSR0C = 0x06;
 *myUBRR0  = tbaud;
}

void U0putchar(unsigned char U0pdata) {
  while((*myUCSR0A & TBE)==0);
  *myUDR0 = U0pdata;
}

void adc_init() {
  // setup the A register
  *my_ADCSRA |= 0b10000000; // set bit   7 to 1 to enable the ADC
  *my_ADCSRA &= 0b11011111; // clear bit 6 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11110111; // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11111000; // clear bit 0-2 to 0 to set prescaler selection to slow reading
  // setup the B register
  *my_ADCSRB &= 0b11110111; // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11111000; // clear bit 2-0 to 0 to set free running mode
  // setup the MUX Register
  *my_ADMUX  &= 0b01111111; // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX  |= 0b01000000; // set bit   6 to 1 for AVCC analog reference
  *my_ADMUX  &= 0b11011111; // clear bit 5 to 0 for right adjust result
  *my_ADMUX  &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
}

unsigned int adc_read(unsigned char adc_channel_num) {
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX  &= 0b11100000;
  // clear the channel selection bits (MUX 5)
  *my_ADCSRB &= 0b11110111;
  // set the channel number
  if(adc_channel_num > 7) {
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
    // set MUX bit 5
    *my_ADCSRB |= 0b00001000;
  }
  // set the channel selection bits
  *my_ADMUX  += adc_channel_num;
  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0x40;
  // wait for the conversion to complete
  while((*my_ADCSRA & 0x40) != 0);
  // return the result in the ADC data register
  return *my_ADC_DATA;
}