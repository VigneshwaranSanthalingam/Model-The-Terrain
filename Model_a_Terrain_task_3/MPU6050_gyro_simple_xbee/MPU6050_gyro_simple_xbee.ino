

#include <Wire.h>
#include <MPU6050.h>
#include "lcd.c"
#define F_CPU 14745600
#include<avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>
#include<math.h>
unsigned char data1,data2,data3;
MPU6050 mpu;

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
void right (void)
{
  motion_set(0x0A);
  }
  void left(void)
  {
  motion_set(0x04);  
  }
void stop1 (void)
{
  motion_set (0x00);
}
void reverse(void)
{
  motion_set(0x09);
}
void loop()
{
//Accel();
forward();
//Accel();
delay(500);
stop1();
//Accel();
delay(500);
reverse();
//Accel();
delay(500);
stop1();
//Accel();
delay(500);
left();
//Accel();
delay(500);
stop1();
//Accel();
delay(500);
right();
//Accel();
delay(500);
stop1();
//Accel();
delay(500);


}

ISR(USART0_RX_vect)     // ISR for receive complete interrupt
{
  UDR0=data1;
  delay(200);
  UDR0=data2;
  delay(200);
  UDR0=data3;
  delay(200);
  
}



