#define __OPTIMIZE__ -O0
#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <math.h> //included to support power function
#include "lcd.c"


#define THRESHOLD 0x10 //   set threshold for White_line_Sensor
#define V_MAX 150
#define V_MIN 50
#define V_SOFT 100
#define NODE_1 (Left_white_line>THRESHOLD && Center_white_line>THRESHOLD && Right_white_line<THRESHOLD)
#define NODE_2 (Left_white_line<THRESHOLD && Center_white_line>THRESHOLD && Right_white_line>THRESHOLD)
#define NODE_3 (Left_white_line>THRESHOLD && Center_white_line>THRESHOLD && Right_white_line>THRESHOLD)
#define PATH (Left_white_line<THRESHOLD && Center_white_line>THRESHOLD && Right_white_line<THRESHOLD)
#define NODE (NODE_1||NODE_2||NODE_3)
#define WHITE (Left_white_line<THRESHOLD && Center_white_line<THRESHOLD && Right_white_line<THRESHOLD)
#define LEFT_DEVIATION (Right_white_line>THRESHOLD)
#define RIGHT_DEVIATION (Left_white_line>THRESHOLD)

void port_init();
void get_input();

unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned char sharp, distance, adc_reading;
unsigned int value;
unsigned char flag = 0;
unsigned char Left_white_line = 0;
unsigned char Center_white_line = 0;
unsigned char Right_white_line = 0;
int x,y,p,q,l,r,SHARP_IR;

void fwdA (void);
void  fwdB(void);
void  q0(void);
void  q5(void);
//void follower(void);

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
  timer5_init();
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
              {WGM51=0, WGMVELOCITY_MIN=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
  
  TCCR5B = 0x0B;  //WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}



//Function To Print Sesor Values At Desired Row And Coloumn Location on LCD


//Function for velocity control
void velocity (unsigned char left_motor, unsigned char right_motor)
{
  OCR5AL = (unsigned char)left_motor;
  OCR5BL = (unsigned char)right_motor;
}

void adc_init()
{
  ADCSRA = 0x00;
  ADCSRB = 0x00;    //MUX5 = 0
  ADMUX = 0x20;   //Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
  ACSR = 0x80;
  ADCSRA = 0x86;    //ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

void init_devices (void)
{
  cli();   //Clears the global interrupts
  port_init();
  adc_init();
  sei();   //Enables the global interrupts
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
  while((ADCSRA&THRESHOLD)==0);  //Wait for conversion to complete
  a=ADCH;
  ADCSRA = ADCSRA|THRESHOLD; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
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

void getinput_WL()
{
    Left_white_line = ADC_Conversion(3);  //Getting data of Left WL Sensor
    Center_white_line = ADC_Conversion(2);  //Getting data of Center WL Sensor
    Right_white_line = ADC_Conversion(1);   //Getting data ofRight WL Sensor

    print_sensor(1,1,3);  //Prints value of White Line Sensor1
    print_sensor(1,5,2);  //Prints Value of White Line Sensor2
    print_sensor(1,9,1);
 }
 
void getinput_sharp()
{
  sharp = ADC_Conversion(11);           //Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
  value = Sharp_GP2D12_estimation(sharp);   //Stores Distance calsulated in a variable "value".
  lcd_print(2,10,value,3);
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

void lcd()
{
  lcd_print(2,1,x,1);
  lcd_print(2,2,y,1);
  lcd_print(2,4,p,1);
  lcd_print(2,5,q,1);
  lcd_print(2,7,l,1);
  lcd_print(2,8,r,1);
  lcd_print(2,10,value,3);
}

void forward (void) //both wheels forward
{
 //printf("\nforward");
 lcd_cursor(1,13);
 lcd_string("F");
 motion_set(0x06);
}

void left_turn (void) //Left wheel backward, Right wheel forward
{
 //printf("\nleft_turn");
 //printf("\ndelay(750)");
 lcd_cursor(1,13);
 lcd_string("L");
 motion_set(0x05);
 delay(600);
}

void right_turn (void) //Left wheel forward, Right wheel backward
{
  //printf("\nright_turn");
  //printf("\ndelay(750)");
   lcd_cursor(1,13);
 lcd_string("R");
  motion_set(0x0A);
  delay(600);
}

void stop1(void)
{
 lcd_cursor(1,13);
 lcd_string("S");
  motion_set(0x00);
}


/*void straight() //white=0 black=1
{
 printf("straight_mode\n");
 if(!(((White_line_sensor_1>=0x2F)&&(White_line_sensor_2>=0x2F)&&(White_line_sensor_3<0x2F))||((White_line_sensor_1>=0x2F)&&(White_line_sensor_2>=0x2F)&&(White_line_sensor_3>=0x2F))||((White_line_sensor_1<0x2F)&&(White_line_sensor_2>=0x2F)&&(White_line_sensor_3>=0x2F))))
 {

  forward();

 }
}*/

void left()
{
   //printf("left\n");
 lcd_cursor(2,13);
 lcd_string("FL");
 //if(!(NODE))
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
  lcd();
}

void right()
{
   //printf("right_mode\n");
 /* if(!(((White_line_sensor_1>=0x2F)&&(White_line_sensor_2>=0x2F)&&(White_line_sensor_3<0x2F))||((White_line_sensor_1>=0x2F)&&(White_line_sensor_2>=0x2F)&&(White_line_sensor_3>=0x2F))||((White_line_sensor_1<0x2F)&&(White_line_sensor_2>=0x2F)&&(White_line_sensor_3>=0x2F))))
  {*/
lcd_cursor(2,13);
 lcd_string("FR");
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
  lcd();
}

void updateA()
{
   //printf("upadateA\n");
 lcd_cursor(2,13);
 lcd_string("UA");
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
    //printf("x=%d\ny=%d\np=%d\nq=%d\nl=%d\nr=%d\n",x,y,p,q,l,r);
 lcd();
}

void updateB()
{ //printf("updateB\n");
  lcd_cursor(2,13);
 lcd_string("UB");
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
  //printf("x=%d\ny=%d\np=%d\nq=%d\nl=%d\nr=%d\n",x,y,p,q,l,r);
lcd();
}

void checkA()
{ //printf("checkA\n");
 lcd_cursor(2,13);
 lcd_string("CA");
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
   //printf("x=%d\ny=%d\np=%d\nq=%d\nl=%d\nr=%d\n",x,y,p,q,l,r);
lcd();
}

void checkB()
{//printf("checkB\n");
 lcd_cursor(2,13);
 lcd_string("CB");
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
   //printf("x=%d\ny=%d\np=%d\nq=%d\nl=%d\nr=%d\n",x,y,p,q,l,r);
lcd();
}

void fwdA()
{   //printf("fwdA\n");
   //follower();
 lcd_cursor(2,13);
 lcd_string("FA");
     //if (NODE)
    //{
  getinput_sharp();
  getinput_WL();   
  if(value>200)
  {

   // straight();
    Black_Follower();
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
 //}
 /*else
 {
   Black_Follower();
 }   */
 // printf("x=%d\ny=%d\np=%d\nq=%d\nl=%d\nr=%d\n",x,y,p,q,l,r);/
 lcd();
}

void fwdB()
{
//printf("fwdB\n");
//follower();
  
  lcd_cursor(2,13);
 lcd_string("FB");
 getinput_WL();
 //if (NODE)
 //{
 getinput_sharp();
  if(value>200)
  {

   // straight();
   Black_Follower();  

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
 //}
/* else
 {
   Black_Follower();
 } */   
 // printf("x=%d\ny=%d\np=%d\nq=%d\nl=%d\nr=%d\n",x,y,p,q,l,r);
 lcd();
}

void A()
{
 //printf("A\n");
 lcd_cursor(2,13);
 lcd_string("AA");
 while(q<6)
  {
    //follower();
  //if (NODE)
  //  { 
    getinput_sharp();
    getinput_WL();
    if(value>200)
     {

      //straight();
    Black_Follower();

      y++;
      q++;
    delay(100);
     // printf("x=%d\ny=%d\np=%d\nq=%d\nl=%d\nr=%d\n",x,y,p,q,l,r);
   lcd();
     }
    else
     {

      right();

      fwdA();
     }
  //}  
  /* else 
   {
     Black_Follower();
   } */    
  }

 q=5;
 p++;
 //printf("x=%d\ny=%d\np=%d\nq=%d\nl=%d\nr=%d\n",x,y,p,q,l,r);
 lcd();
 right();
 r=0;
 //printf("x=%d\ny=%d\np=%d\nq=%d\nl=%d\nr=%d\n",x,y,p,q,l,r);
 lcd();
 q5();
}

void B()
{
  //printf("B\n");
  lcd_cursor(2,13);
 lcd_string("BB");

 while(q>-1)
  {
    //follower();
 //if (NODE)
 // {
  getinput_sharp();
  getinput_WL();
    if(value>200)
     {

      //straight();
    Black_Follower();

      y--;
      q--;
    delay(100);
      //printf("x=%d\ny=%d\np=%d\nq=%d\nl=%d\nr=%d\n",x,y,p,q,l,r);
    lcd();
     }
    else
     {

      right();

      fwdB();

     }
  //}
  /*else 
  {
    Black_Follower();
  }*/
         
  }  

 q=0;
 p++;
 //printf("x=%d\ny=%d\np=%d\nq=%d\nl=%d\nr=%d\n",x,y,p,q,l,r);
 lcd();
 if (p!=6)
 {
 left();
 l=0;

 q0();
 }
}


void q5()
{
  //printf("q5\n");
  lcd_cursor(2,13);
  lcd_string("Q5");
  //straight();
  Black_Follower();
  x++;
  right();
  r=0;
  q--;
  //printf("x=%d\ny=%d\np=%d\nq=%d\nl=%d\nr=%d\n",x,y,p,q,l,r);
  lcd();
}

void q0()
{
   //printf("q0\n");
   lcd_cursor(2,13);
   lcd_string("Q0");
 
 // straight();
  Black_Follower();
  x++;
  left();
  l=0;
  q++;
 // printf("x=%d\ny=%d\np=%d\nq=%d\nl=%d\nr=%d\n",x,y,p,q,l,r);
  lcd();
}

/*void Black_Follower(void)
{
  
  
  getinput_WL();
   lcd_cursor(1,14);
    lcd_string("P");
   if (NODE)
  {
  lcd_cursor(1,14);
  lcd_string("N");
    return;
  }  
    if(PATH)
    {
      //flag=1;
      forward();
      velocity(V_MAX,V_MAX);
      lcd();
    }
  
    if(LEFT_DEVIATION)// && (flag==0))
    {
     // flag=1;
      forward();
      velocity(V_MAX,V_SOFT);
    }

   if(RIGHT_DEVIATION)// && (flag==0))
    {
      //flag=1;
      forward();
      velocity(V_SOFT,V_MAX);
    }
  if (WHITE)
  {
    stop1();
    velocity(0,0);
  }
   /* if(NODE_1||NODE_2||NODE_3)
    {
      /*b=b+1;
      lcd_print(2,1,b,2);
      delay(100);
    return;
    }*/
   /*if (WHITE)
    {
      stop1();
      velocity(0,0);  
    }
   
  }*/
  void Black_Follower(void)
{
  getinput_WL();
  while(!NODE)
  { 
  getinput_WL();
   lcd_cursor(1,14);
    lcd_string("P");
    if(PATH)
    {
     // flag=1;
      forward();
      velocity(V_MAX,V_MAX);
      lcd();
    }
  
    if(LEFT_DEVIATION) // && (flag==0))
    {
      //flag=1;
      forward();
      velocity(V_MAX,V_SOFT);
    }

   if(RIGHT_DEVIATION) // && (flag==0))
    {
      //flag=1;
      forward();
      velocity(V_SOFT,V_MAX);
    }
  }
  /* if(NODE_1||NODE_2||NODE_3)
    {
      /*b=b+1;
      lcd_print(2,1,b,2);
      delay(100);
    return;
    }*/
   if (WHITE)
    {
      stop1();
      velocity(0,0);  
    }
    lcd_cursor(1,14);
    lcd_string("N");
    delay(250);
   return;
  }
     
  
void setup()
{
  init_devices();
  lcd_set_4bit();
  lcd_init();
}

void loop()
{
//printf("main");
 
   getinput_WL();
    //print_sensor(2,9,11);             //Analog Value Of Front Sharp Sensor
   flag=0;

    print_sensor(1,1,3);  //Prints value of White Line Sensor1
    print_sensor(1,5,2);  //Prints Value of White Line Sensor2
    print_sensor(1,9,1);  //Prints Value of White Line Sensor3
   x=0;
   y=0;
   p=0;
   q=1;
   l=0;
   r=0;
    lcd();
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

