#include <dht_nonblocking.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <LiquidCrystal.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Wire.h>
#include "RTClib.h"
#include <SPI.h>

//used to set up serial port (Serial.begin())
#define RDA 0x80
#define TBE 0x20  
volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;

// Pins for LEDs
#define RED 47 
#define YELLOW 49 
#define GREEN 51 
#define BLUE 53   
#define DHT_SENSOR_TYPE DHT_TYPE_11

// Char array for days of the week
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

// Real-time clock and servo objects
RTC_DS1307 rtc;
Servo myservo;
const int A = 0b00000010; 

//integer variables for LEDs
int redval = 255;
int greenval = 255;
int blueval = 255;
int yellowval = 255;

//ADC Register Pointers
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;


//variables for water sensor code.
int adc_id = 0;
int ReadingVal = 0;
char printBuffer[128];

//numbers of the interface pins
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);  //RS,E,D4,D5,D6,D7

static const int DHT_SENSOR_PIN = 2; //pin two for temp sensor
//fan
//#define WRITE_HIGH_PE(pin_num) *port_e |= (0x01 << pin_num);
//#define WRITE_LOW_PE(pin_num) *port_e &= ~(0x01 << pin_num);
 /*
  if(DHT_SENSOR_PIN.temperature<18)
  {
    state = 1;
    WRITE_LOW_PE(3);
  }
  if(DHT_SENSOR_PIN.temperature>18 && state!=2)
  {
    state = 3;
    WRITE_HIGH_PE(3);
  }*/

// DHT non-blocking sensor setup
DHT_nonblocking dht_sensor( DHT_SENSOR_PIN, DHT_SENSOR_TYPE );

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

//channel_selector
// Pins for button and button state variables
int inPin = 52;         //input pin for button
int outPin = 50;       //output pin for button
int state = LOW;      //state of the output pin
int reading;           //reading from the input pin
int previous = HIGH;    //reading from the input pin
long time = 0;         //last time the output pin was toggled
long debounce = 200;   //debounce time

void setup( )
{
  U0init(9600);//// Serial communication setup
  adc_init(); //initialize water and servo controls
  myservo.attach(13);//servo pin
  myservo.write(90);// move servo to 90°
  
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);

  //setting pins to output
  *ddr_l |= B00000100;//red
  
  *ddr_b |= B00000100;//green
  
  *ddr_b |= B00000001;//blue
  
  *ddr_l |= B00000001;//yellow
  
  *ddr_l &= B11111011;//setting red to low
 
  *port_l |= B00000001;//setting yellow to high

 //DC motor setup
 //Set PE5 to output
  *ddr_e |= 0x20;
//set PE3 to output
  *ddr_e |= 0x08;
//set PG5 to output
  *ddr_g |= 0x20;

  pinMode(inPin, INPUT);
  pinMode(outPin, OUTPUT);

  //error messages for RTC
 if (!rtc.begin()) {
   Serial.println("Couldn't find RTC");
   while (1);
 }
 if (!rtc.isrunning()) {
   Serial.println("RTC is NOT running!");
 }
}

void loop( )
{  
  float temp;
  float Humidity;
  reading = digitalRead(inPin);

  if (reading == HIGH && previous == LOW && millis() - time > debounce) {
    if (state == HIGH){
      state = LOW;
      Serial.print("OFF\n");
    }
    else{
      Serial.print("ON\n");
      state = HIGH;
    }
    time = millis();    
  }

  digitalWrite(outPin, state);

  previous = reading;

  if(state == HIGH){
  if( measure_environment( &temp, &Humidity ) == true )
  {
    timeStamp();
    Serial.print( "Temperature: " );
    float temp1 = temp * 1.8;
    float temp2 = temp1 + 32; //conversion from C to F
    temp = temp2;
    Serial.print(temp, 1);
    Serial.print("\n");
    Serial.print( "deg. F, Humidity = " );
    Serial.print(Humidity, 1);
    Serial.print("%\n");
  }
    //water sensor loop code:
    int val = adc_read(adc_id); // get adc value
    
    //red LED water level low 
    if(val < 50){ errorLED(val); }
    
    if(((ReadingVal>=val) && ((ReadingVal - val) > 10)) || ((ReadingVal<val) && ((val - ReadingVal) > 10)))
    {
      sprintf(printBuffer,"Water level is %d\n", val);
      Serial.print(printBuffer);
      ReadingVal = val;
    }

    //servo loops:
  servoLoop();
  
if(temp > 0){
  lcdScreen(temp, Humidity);
}

  motorToggle(temp, val);
  }
  else if(state == LOW){
  lcd.setCursor(0, 0);
  lcd.print("STATUS:           ");
  lcd.setCursor(0, 1);
  lcd.print("IDLE...          ");

  
  *ddr_b |= B00000001; //write a 0 to blue LED
   *ddr_l &= B11111011; //write a 0 to red LED
  *port_l |= B00000001;  //analogWrite(YELLOW, yellowval);
  *port_b &= B11111011;//write a 0 to green LED
    
  }
}

// Initialize UART communication
void U0init(unsigned long U0baud)
{
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 // Same as (FCPU / (16 * U0baud)) - 1;
 *myUCSR0A = 0x20;
 *myUCSR0B = 0x18;
 *myUCSR0C = 0x06;
 *myUBRR0  = tbaud;
}

// Control DC motor and LEDs based on temperature and water level
void motorToggle(float temp, float val){
      if(temp > 80 && val > 50){
  //write a 1 to the enable bit on PE3
  *port_e |= 0x08;
  //analogWrite(BLUE, blueval);
  *port_b |= B00000001;
  //analogWrite(RED, 0); && analogWrite(YELLOW, 0);
  *port_l &= 11111010;
   //analogWrite(GREEN, 0);
  *port_b &= B11111011;
  }
  
  if(temp < 80){
  *port_e &= 0x00;
  *port_l &= B11111010;
  *port_b &= B11111110;
  *port_b |= B00000100; 
  }

//writing a 0 to PG5
*port_g &= 0x20;

//writing a 1 to PE5
*port_e |= 0x20;

}
//<8
// Display temperature and humidity on LCD screen
void lcdScreen(float temp, float Humidity)
{
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

// Control LEDs to indicate water level error
void errorLED(int waterLevel){
  *port_b &= B11111010;
  *port_l |= B00000100;
  //Error messages when water is too low/sensor is not on
  lcd.setCursor(0, 0);
  lcd.print("Error!         ");
  lcd.setCursor(0, 1);
  lcd.print("WATER TOO LOW!      ");
  waterLevel = adc_read(adc_id);
  *port_e &= 0x00;
  delay(4000);
  if(waterLevel < 50){
  errorLED(waterLevel);
  }
}

// Measure temperature and humidity using the DHT sensor
static bool measure_environment( float *temp, float *Humidity )
{
  static unsigned long measurement_timestamp = millis( );

  /* Measure once every four seconds. */
  if( millis( ) - measurement_timestamp > 3000ul )
  {
    if( dht_sensor.measure( temp, Humidity ) == true )
    {
      measurement_timestamp = millis( );
      return( true );
    }
  }

  return( false );
}

// ADC setup
//function for water sensor loop and servo loop
void adc_init()
{
  // setup the A register
  *my_ADCSRA |= B10000000;
  *my_ADCSRA &= B11110111;
  *my_ADCSRA &= B11011111;
  
  // setup the B register
  *my_ADCSRB &= B11111000;

  // setup the MUX Register
  *my_ADMUX |= (1<<REFS0);
}

 // Read ADC value from a specific channel
//second funciton for water sensor and servo loop:
unsigned int adc_read(unsigned int adc_channel_num)
{
  int channel_selector;
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX &= B11100000;

  // clear the channel selection bits (MUX 5)
  *my_ADCSRB &= B11110111;

  //Assign correct channel using MUX 5:0
  if (adc_channel_num < 8) {
    *my_ADMUX |= adc_channel_num;
  }
  else if ((adc_channel_num > 7) && (adc_channel_num < 16)) {
     channel_selector = (adc_channel_num - 8);
     *my_ADCSRB |= B00001000;
     *my_ADMUX |= channel_selector;
  }

  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= B01000000;
  
  // wait for the conversion to complete
  while ((*my_ADCSRA & 0x40) != 0);
  
  // return the result in the ADC data register
  return (*my_ADC_DATA & 0x03FF);
}

// Control servo motor based on analog input
void servoLoop(){
  int voltage = adc_read(A);//read voltage from POT
  int angle = voltage/5.7;//Scale down analog input to be between 180 and 0
  myservo.write(angle);// move servos   
}

 // Log the time for temperature readings
void timeStamp(){
//loop to log time for temp readings
 DateTime now = rtc.now();
 Serial.print("\nTime: ");
 Serial.print(now.month(), DEC);
 Serial.print('/');
 Serial.print(now.day(), DEC);
 Serial.print(" ");
 int hour = now.hour();
 hour -= 4; //adjusting the time
 Serial.print(hour, DEC);
 Serial.print(':');
 Serial.print(now.minute(), DEC);
 Serial.print(':');
 Serial.print(now.second(), DEC);
 Serial.println();
 delay(3000); 
}
