#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <math.h> //included to support power function
#include "lcd.c"

#define PATH (Left_white_line<THRESHOLD && Center_white_line>THRESHOLD && Right_white_line<THRESHOLD)
#define NODE_INSIDE (Left_white_line>THRESHOLD && Center_white_line>THRESHOLD && Right_white_line>THRESHOLD)
#define NODE_CORNER (Left_white_line<THRESHOLD && Center_white_line>THRESHOLD && Right_white_line>THRESHOLD)
#define LEFT_DEVIATION (Left_white_line<THRESHOLD && Center_white_line<THRESHOLD && Right_white_line>THRESHOLD)
#define RIGHT_DEVIATION (Left_white_line>THRESHOLD && Center_white_line<THRESHOLD && Right_white_line<THRESHOLD)
#define THRESHOLD 0x10 //   set threshold for White_line_Sensor
#define V_MAX 150
#define V_MIN 50

void port_init();
void timer5_init();
void velocity(unsigned char, unsigned char);
void motors_delay();

unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned char flag = 0;
unsigned char sharp, distance, adc_reading;
unsigned int value,x,y,p,q,l,r,count;
unsigned char Left_white_line = 0;
unsigned char Center_white_line = 0;
unsigned char Right_white_line = 0;

//Function to configure LCD port
void lcd_port_config (void)
{
 DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
 PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}

//ADC pin configuration
void adc_pin_config (void)
{
 DDRF = 0x00; 
 PORTF = 0x00;
 DDRK = 0x00;
 PORTK = 0x00;
}

//Function to configure ports to enable robot's motion
void motion_pin_config (void) 
{
 DDRA = DDRA | 0x0F;
 PORTA = PORTA & 0xF0;
 DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}

//Function to Initialize PORTS
void port_init()
{
  lcd_port_config();
  adc_pin_config();
  motion_pin_config();  
}

// Timer 5 initialized in PWM mode for velocity control
// Prescale:256
// PWM 8bit fast, TOP=0x00FF
// Timer Frequency:225.000Hz
void timer5_init()
{
  TCCR5B = 0x00;  //Stop
  TCNT5H = 0xFF;  //Counter higher 8-bit value to which OCR5xH value is compared with
  TCNT5L = 0x01;  //Counter lower 8-bit value to which OCR5xH value is compared with
  OCR5AH = 0x00;  //Output compare register high value for Left Motor
  OCR5AL = 0xFF;  //Output compare register low value for Left Motor
  OCR5BH = 0x00;  //Output compare register high value for Right Motor
  OCR5BL = 0xFF;  //Output compare register low value for Right Motor
  OCR5CH = 0x00;  //Output compare register high value for Motor C1
  OCR5CL = 0xFF;  //Output compare register low value for Motor C1
  TCCR5A = 0xA9;  /*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
            For Overriding normal port functionality to OCRnA outputs.
              {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
  
  TCCR5B = 0x0B;  //WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

void adc_init()
{
  ADCSRA = 0x00;
  ADCSRB = 0x00;    //MUX5 = 0
  ADMUX = 0x20;   //Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
  ACSR = 0x80;
  ADCSRA = 0x86;    //ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

//Function For ADC Conversion
unsigned char ADC_Conversion(unsigned char Ch) 
{
  unsigned char a;
  if(Ch>7)
  {
    ADCSRB = 0x08;
  }
  Ch = Ch & 0x07;       
  ADMUX= 0x20| Ch;        
  ADCSRA = ADCSRA | 0x40;   //Set start conversion bit
  while((ADCSRA&0x10)==0);  //Wait for conversion to complete
  a=ADCH;
  ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
  ADCSRB = 0x00;
  return a;
}

//Function To Print Sesor Values At Desired Row And Coloumn Location on LCD
void print_sensor(char row, char coloumn,unsigned char channel)
{
  
  ADC_Value = ADC_Conversion(channel);
  lcd_print(row, coloumn, ADC_Value, 3);
}

//Function for velocity control
void velocity (unsigned char left_motor, unsigned char right_motor)
{
  forward();
  OCR5AL = (unsigned char)left_motor;
  OCR5BL = (unsigned char)right_motor;
}

//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
 unsigned char PortARestore = 0;

 Direction &= 0x0F;     // removing upper nibbel for the protection
 PortARestore = PORTA;    // reading the PORTA original status
 PortARestore &= 0xF0;    // making lower direction nibbel to 0
 PortARestore |= Direction; // adding lower nibbel for forward command and restoring the PORTA status
 PORTA = PortARestore;    // executing the command
}

void forward (void) 
{
  motion_set (0x06);
  //velocity(50,50);
}
void right (void)
 {
  motion_set(0x0A);
  //delay(2000);
 }
void left(void)
 {
  motion_set(0x04); 
  //delay(2000); 
 }
void stop (void)
{
  motion_set (0x00);
}

void init_devices (void)
{
  cli(); //Clears the global interrupts
  port_init();
  adc_init();
  timer5_init();
  sei();   //Enables the global interrupts
}

void get_white_line()
{
   Left_white_line = ADC_Conversion(3);  //Getting data of Left WL Sensor
   Center_white_line = ADC_Conversion(2);  //Getting data of Center WL Sensor
   Right_white_line = ADC_Conversion(1); //Getting data of Right WL Sensor

   print_sensor(1,1,3);  //Prints value of White Line Sensor1
   print_sensor(1,5,2);  //Prints Value of White Line Sensor2
   print_sensor(1,9,1);  //Prints Value of White Line Sensor3
}

unsigned int Sharp_GP2D12_estimation(unsigned char adc_reading)
{
  float distance;
  unsigned int distanceInt;
  distance = (int)(10.00*(2799.6*(1.00/(pow(adc_reading,1.1546)))));
  distanceInt = (int)distance;
  if(distanceInt>800)
   {
     distanceInt=800;
   }
  return distanceInt;
}

void get_distance(void)
{
    sharp = ADC_Conversion(11);           //Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
    value = Sharp_GP2D12_estimation(sharp);       //Stores Distance calsulated in a variable "value".
    lcd_print(2,1,value,3);
    if (value<200)
    {
      lcd_wr_command(0x01);
      //delay(1000);
      lcd_string("Obstacle present");
      delay(1000);
      lcd_wr_command(0x01);
      //lcd_string(" Moving");
      //delay(1000);
      //lcd_wr_command(0x01);
    }
}

void Black_line_follower()
{
  if (value>=200)
 {
   if(PATH)
   {
    //flag=1;
    //forward();
    velocity(50,50);
    delay(5000);
    //return;
   }
   else if(LEFT_DEVIATION)//&&flag==0)
   {
    //flag=1;
    //while(!PATH)
    //{
    // right();
    //}
    velocity(60,40);
    delay(500);
    //return;
   }
   else if(RIGHT_DEVIATION)//&&flag==0)
   {
    //flag=1;
    //while(!PATH)
    //{
    //left();
    //}
    velocity(40,60);
    delay(500);
    //return;
   }
   else if((NODE_INSIDE)||(NODE_CORNER))
   {
    get_distance();
    return;
   }
}
else
 {
   right();
   delay(500);
   get_distance();
   return;
 }
return;
}

void setup()
{
  init_devices();
  lcd_set_4bit();
  lcd_init();
  get_distance();
  forward();
  //velocity(150,150);
}

void loop()
{
  get_white_line();
  //flag=0;
  //get_distance();
  velocity(150,150);
  Black_line_follower();
}
