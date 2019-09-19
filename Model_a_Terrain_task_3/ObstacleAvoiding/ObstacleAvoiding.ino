#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "lcd.c"

unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned char sharp, distance, adc_reading;
unsigned int value;

//Function to configure LCD port
void lcd_port_config (void)
{
 DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
 PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}

//ADC pin configuration
void adc_pin_config (void)
{
 DDRF = 0x00; //set PORTF direction as input
 PORTF = 0x00; //set PORTF pins floating
 DDRK = 0x00; //set PORTK direction as input
 PORTK = 0x00; //set PORTK pins floating
}

//Function to Initialize PORTS
void port_init()
{
  lcd_port_config();
  adc_pin_config(); 
  motion_pin_config();
}

void motion_pin_config (void)
{
 DDRA = DDRA | 0x0F; //set direction of the PORTA 3 to PORTA 0 pins as output
 PORTA = PORTA & 0xF0; // set initial value of the PORTA 3 to PORTA 0 pins to logic 0
 DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM
}

//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
 unsigned char PortARestore = 0;

 Direction &= 0x0F;       // removing upper nibbel as it is not needed
 PortARestore = PORTA;      // reading the PORTA's original status
 PortARestore &= 0xF0;      // setting lower direction nibbel to 0
 PortARestore |= Direction;   // adding lower nibbel for direction command and restoring the PORTA status
 PORTA = PortARestore;      // setting the command to the port
}


void forward (void) //both wheels forward
{
  motion_set(0x06);
}

void right (void) //Left wheel forward, Right wheel backward
{
  motion_set(0x0A);
}

//Function to Initialize ADC
void adc_init()
{
  ADCSRA = 0x00;
  ADCSRB = 0x00;    //MUX5 = 0
  ADMUX = 0x20;   //Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
  ACSR = 0x80;
  ADCSRA = 0x86;    //ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}



//This Function accepts the Channel Number and returns the corresponding Analog Value 
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
  while((ADCSRA&0x10)==0);  //Wait for ADC conversion to complete
  a=ADCH;
  ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
  ADCSRB = 0x00;
  return a;
}


// This Function calculates the actual distance in millimeters(mm) from the input
// analog value of Sharp Sensor. 
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

void init_devices (void)
{
 cli(); //Clears the global interrupts
 port_init();
 adc_init();
 sei(); //Enables the global interrupts
}

void setup() 
{
  // put your setup code here, to run once:
  init_devices();
  lcd_set_4bit();
  lcd_init();
}

void loop() 
{
  while(1)
  {
    //print_sensor(2,9,11);             //Analog Value Of Front Sharp Sensor

    sharp = ADC_Conversion(11);           //Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
    value = Sharp_GP2D12_estimation(sharp);       //Stores Distance calsulated in a variable "value".
    lcd_print(1,8,value,3);            //Prints Value Of Distanc in MM measured by Sharp Sensor.
    if (value>=300)
     forward();
    else
     right();
  }

  // put your main code here, to run repeatedly:

}

