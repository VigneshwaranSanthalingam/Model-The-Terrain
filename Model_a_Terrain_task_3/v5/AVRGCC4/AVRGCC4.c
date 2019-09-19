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
#define V_MAX 130
#define V_MIN 50
#define THRESHOLD 0x10 //   set threshold for White_line_Sensor

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

void fwdA (void);
void fwdB(void);
void q0(void);
void q5(void);
void follower(void)

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
  //forward();
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
  lcd_cursor(1,13);
  lcd_string("FD");
  motion_set (0x06);
  //velocity(50,50);
}
void right_turn (void)
 {
  lcd_cursor(1,13);
  lcd_string("RT");
  motion_set(0x0A);
  delay(600);
 }
void left_turn(void)
 {
  lcd_cursor(1,13);
  lcd_string("LT");
  motion_set(0x04); 
  delay(600); 
 }

void back(void)
{
  lcd_cursor(1,13);
  lcd_string("BK");
  motion_set(0x09);
}
void stop_1 (void)
{
  lcd_cursor(1,13);
  lcd_string("ST");
  motion_set (0x00);
}

/void Black_Follower(void)
{
    if(PATH)
    {
	  lcd_cursor(2,13);
      lcd_string("PA");
      flag=1;
      forward();
      velocity(V_MAX,V_MAX);
      //lcd_print(2,6,b,2);
    }
  
   else if(LEFT_DEVIATION)
    {
	  lcd_cursor(2,13);
      lcd_string("PA");
      flag=1;
      forward();
      velocity(V_MAX,V_MIN);
    }

   else if(RIGHT_DEVIATION)
    {
	  lcd_cursor(2,13);
      lcd_string("PA");
      flag=1;
      forward();
      velocity(V_MIN,V_MAX);
    }
  else if((Center_white_line<THRESHOLD) && (Left_white_line < THRESHOLD) && (Right_white_line < THRESHOLD))
    {
      stop1();
      velocity(0,0);  
    }
  else if(NODE_INSIDE || NODE_CORNER)
   {
     lcd_cursor(2,13);
     lcd_string("NO");
     /* b=b+1;
      lcd_print(2,6,b,2);
      delay(100);*/
	 break;
   }
}*/
void straight() //white=0 black=1
{
 if(!(((White_line_sensor_1>=0x2F)&&(White_line_sensor_2>=0x2F)&&(White_line_sensor_3<0x2F))||((White_line_sensor_1>=0x2F)&&(White_line_sensor_2>=0x2F)&&(White_line_sensor_3>=0x2F))||((White_line_sensor_1<0x2F)&&(White_line_sensor_2>=0x2F)&&(White_line_sensor_3>=0x2F))))
 {
  forward();
 }
}

void left()
{
  //if(!(((White_line_sensor_1>=0x2F)&&(White_line_sensor_2>=0x2F)&&(White_line_sensor_3<0x2F))||((White_line_sensor_1>=0x2F)&&(White_line_sensor_2>=0x2F)&&(White_line_sensor_3>=0x2F))||((White_line_sensor_1<0x2F)&&(White_line_sensor_2>=0x2F)&&(White_line_sensor_3>=0x2F))))
  //{
   left_turn();
  //}
  l+=1;
  if(r==2)
  {
    l=0;
    r=1;
  }
  if(l==3)
  {
    l=0;
    r=1;
  }
  if((l==2)&&(r==1))
  {
    l=1;
    r=0;
  }
}

void right()
{
  
 /* if(!(((White_line_sensor_1>=0x2F)&&(White_line_sensor_2>=0x2F)&&(White_line_sensor_3<0x2F))||((White_line_sensor_1>=0x2F)&&(White_line_sensor_2>=0x2F)&&(White_line_sensor_3>=0x2F))||((White_line_sensor_1<0x2F)&&(White_line_sensor_2>=0x2F)&&(White_line_sensor_3>=0x2F))))
  {*/

   right_turn();

 // }
  r+=1;
  if(l==2)
  {
    r=0;
    l=1;
  }
  if(r==3)
  {
    r=0;
    l=1;
  }
  if((r==2)&&(l==1))
  {
    r=1;
    l=0;
  }
}

void updateA()
{
  if(l==1)
    { x--;}
  if(r==1)
    { x++;}
  if(l+r==2)
  {

    if((l==2)||(r==2))
      { y--;}
    else
      { y++;}
    l=0;
    r=0;
    }
    
}

void updateB()
{ 
  if(l==1)
    { x++;}
  if(r==1)
    { x--;}
  if(l+r==2)
  {
    if((l==2)||(r==2))
      { y++;}
    else
      { y--;}
    l=0;
    r=0;
  }
}

void checkA()
{ 
  if(x>p)
   {

    left();

    fwdA();
   }
  if(x<p)
   {

    right();
    fwdA();
   }
  if(x==p)
   {
    if(y>q)
     {

      left();

      fwdA();
     }
    if(y<q)
     {

      right();

      fwdA();
     }
    else
     {

      q++;

      if(l==1)
      {

        right();
      }
      if(r==1)
      {

        left();
      }
      if((l==2)||(r==2))
       {

        left();

        left();
       }
      l=0;
      r=0;
     }
   }
}

void checkB()
{
  if(x>p)
   {

    right();

    fwdB();
   }
  if(x<p)
   {

    left();

    fwdB();
   }
  if(x==p)
   {
    if(y>q)
     {

      right();

      fwdB();
     }
    if(y<q)
     {

      left();

      fwdB();
     }
    else
     {
      q--;
      if(l==1)
      {

        right();
      }
      if(r==1)
      {

        left();
      }
      if((l==2)||(r==2))
       {

        left();

        left();

       }
      l=0;
      r=0;
     }
   }
   
}

void fwdA()
{
   
   get_white_line();
  if(value>180)
  {

    straight();

    updateA();

    checkA();

  }
  else
  {

    if((l==1)&&(r==1))
     {
      l=0;
      r=0;
     }

    right();

    fwdA();

  }
  
}

void fwdB()
{
get_white_line();
  if(value>180)
  {

    straight();

    updateB();

    checkB();

  }
  else
  {

    if((l==1)&&(r==1))
     {
      l=0;
      r=0;
     }

    left();

    fwdB();
  }
  
}

void A()
{
 
 while(q<6)
  {
    get_white_line();

    if(value>180)
     {

      straight();

      y++;
      q++;
      
     }
    else
     {

      right();

      fwdA();
     }
  }

 q=5;
 p++;
 

 right();
 r=0;
 

 q5();
}

void B()
{
 

 while(q>-1)
  {
    get_white_line();
    if(value>180)
     {

      straight();

      y--;
      q--;
      
     }
    else
     {

      right();

      fwdB();

     }
  }

 q=0;
 p++;
 
 if (p!=6)
 {
 left();
 l=0;

 q0();
 }
}


void q5()
{
  
  straight();
  x++;
  right();
  r=0;
  q--;
  
}

void q0()
{
   
  straight();
  x++;
  left();
  l=0;
  q++;
 
}


void get_white_line()
{
   Left_white_line = ADC_Conversion(3);  //Getting data of Left WL Sensor
   Center_white_line = ADC_Conversion(2);  //Getting data of Center WL Sensor
   Right_white_line = ADC_Conversion(1); //Getting data of Right WL Sensor

   print_sensor(1,1,3);  //Prints value of White Line Sensor1
   print_sensor(1,5,2);  //Prints Value of White Line Sensor2
   print_sensor(1,9,1);  //Prints Value of White Line Sensor3
   
   get_distance();
   
   
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
    if (value<180)
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
    //print_sensor(2,9,11);             //Analog Value Of Front Sharp Sensor
   x=0;
   y=0;
   p=0;
   q=1;
   l=0;
   r=0;
   while(p<6)
    {

     if(p%2==0)
      A();
     else
      B();
    }
   left();
   left();
  
}

