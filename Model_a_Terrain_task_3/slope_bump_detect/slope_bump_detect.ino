#include <Wire.h>
#include <MPU6050.h>
#include "lcd.c"
#define F_CPU 14745600
#include<avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>
#include<math.h>

#define THRESHOLD 0x10 //   set threshold for White_line_Sensor

MPU6050 mpu;
unsigned int Xnorm,Ynorm,Znorm;
volatile unsigned long int ShaftCount = 0;
unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned char flag = 0;
unsigned char sharp, distance, adc_reading;

unsigned int Slope_Count_low = (unsigned int)350/5.338; // division by resolution to get shaft count
unsigned int Bump_Count_low = (unsigned int) 180/5.338;
unsigned int Slope_Count_high = (unsigned int)370/5.338;
unsigned int Bump_Count_high = (unsigned int)200/5.338;

//ADC pin configuration
void adc_pin_config (void)
{
 DDRF = 0x00; 
 PORTF = 0x00;
 DDRK = 0x00;
 PORTK = 0x00;
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

void adc_init()
{
  ADCSRA = 0x00;
  ADCSRB = 0x00;    //MUX5 = 0
  ADMUX = 0x20;   //Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
  ACSR = 0x80;
  ADCSRA = 0x86;    //ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}



void position_encoder_interrupt_init (void)
{
  cli(); //Clears the global interrupt
  EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
  EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
  sei();   // Enables the global interrupt
}


//ISR for right position encoder
ISR(INT5_vect)
{ 
   ShaftCount++;  //increment right shaft position count
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

void port_init()
{
  adc_pin_config();
  timer5_init();
  position_encoder_interrupt_init();
}
void setup() 
{
  //Initialise USART0 for Xbee
 UCSR0B = 0x00; //disable while setting baud rate
 UCSR0A = 0x00;
 UCSR0C = 0x06;
 UBRR0L = 0x5F; // 14745600 Hzset baud rate lo
 UBRR0H = 0x00; //set baud rate hi
 UCSR0B = 0x98;
 
 //Initialise LCD
 DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
 PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
 lcd_set_4bit();
 lcd_init(); 

 //Initialise Motor
 DDRA = DDRA | 0x0F;
 PORTA = PORTA & 0xF0;
 DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.

 // Initialize MPU6050
  lcd_string("Initialize MPU6050");
  delay(1000);
  lcd_wr_command(0x01);
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    lcd_string("Could not find a valid MPU6050 sensor, check wiring!");
    lcd_wr_command(0x01);
    delay(1000);
  }
  mpu.calibrateGyro();
  mpu.setThreshold(3);
  checkSettings();
 
}
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
}

void checkSettings()
{ 
  lcd_wr_command(0x01);
  lcd_cursor(1,1);  
  lcd_string(" * Sleep Mode:        ");
  lcd_cursor(2,1);
  lcd_string(mpu.getSleepEnabled() ? "En" : "DEN");
  delay(1000);
  lcd_wr_command(0x01);
  lcd_cursor(1,1);
  lcd_string(" * Clock Source:      ");
  lcd_cursor(2,1);
  switch(mpu.getClockSource())
  {
    case MPU6050_CLOCK_KEEP_RESET:     lcd_string("Stops"); break;
    case MPU6050_CLOCK_EXTERNAL_19MHZ: lcd_string("19.2MHz ref"); break;
    case MPU6050_CLOCK_EXTERNAL_32KHZ: lcd_string("32.768kHz ref"); break;
    case MPU6050_CLOCK_PLL_ZGYRO:      lcd_string("Z axis"); break;
    case MPU6050_CLOCK_PLL_YGYRO:      lcd_string("Y axis"); break;
    case MPU6050_CLOCK_PLL_XGYRO:      lcd_string("X axis"); break;
    case MPU6050_CLOCK_INTERNAL_8MHZ:  lcd_string("8MHz osc"); break;
  }
  delay(1000);
  lcd_wr_command(0x01);
  lcd_string(" * Gyroscope:         ");
  lcd_cursor(2,1);
  switch(mpu.getScale())
  {
    case MPU6050_SCALE_2000DPS:        lcd_string("2000 dps"); break;
    case MPU6050_SCALE_1000DPS:        lcd_string("1000 dps"); break;
    case MPU6050_SCALE_500DPS:         lcd_string("500 dps"); break;
    case MPU6050_SCALE_250DPS:         lcd_string("250 dps"); break;
  } 
  delay(500);
  lcd_wr_command(0x01);
  lcd_cursor(1,1);
  lcd_string(" * Gyroscope Xoffsets: ");
  lcd_print(2,1,mpu.getGyroOffsetX(),5);
  delay(500);
  lcd_wr_command(0x01);
  lcd_string(" * Gyroscope Yoffsets: ");
  lcd_print(2,1,mpu.getGyroOffsetY(),5);
  delay(500);
  lcd_wr_command(0x01);
  lcd_string(" * Gyroscope Zoffsets: ");
  lcd_print(2,1,mpu.getGyroOffsetZ(),5);
  delay(500);
    
}
void loop()
{
  forward();
  Gyro();
  lcd_wr_command(0x01);
  lcd_print(1,1,Xnorm,5);
  delay(750);
  lcd_print(1,7,Ynorm,5);
  delay(750);
  lcd_print(2,1,Znorm,5);
  delay(750);
  
 if ((ShaftCount>=Slope_Count_low) && (ShaftCount<=Slope_Count_high))
{
  lcd_cursor(2,7);
  lcd_string("SLOPE");
  delay(500);
  lcd_wr_command(0x01);
}
else if ((ShaftCount>=Bump_Count_low) && (ShaftCount<=Bump_Count_high))
{
  lcd_cursor(2,7);
  lcd_string("BUMP");
  delay(500);
  lcd_wr_command(0x01);
}
}
unsigned int data;

ISR(USART0_RX_vect)     // ISR for receive complete interrupt
{
  data = UDR0;
  UDR0 = data;
  UDR0=Xnorm;
  delay(200);
  UDR0=Ynorm;
  delay(200);
  UDR0=Znorm;
  delay(200);
}

void Gyro(void)
{
  Vector normGyro = mpu.readNormalizeAccel();
  
  Xnorm = normGyro.XAxis;
  Ynorm = normGyro.YAxis;
  Znorm = normGyro.ZAxis;

}



