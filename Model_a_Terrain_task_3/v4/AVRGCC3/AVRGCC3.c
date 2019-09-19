#include "Wire.h"
#include <MPU6050.h>
#include "lcd.c"
#define F_CPU 14745600
#include<avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>

unsigned char data;

MPU6050 mpu;

void setup() 
{
 //Initialise UART0
 UCSR0B = 0x00; //disable while setting baud rate
 UCSR0A = 0x00;
 UCSR0C = 0x06;
 // UBRR0L = 0x47; //11059200 Hz
 UBRR0L = 0x5F; // 14745600 Hzset baud rate lo
 UBRR0H = 0x00; //set baud rate hi
 UCSR0B = 0x98;
 //Initialise LCD
 DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
 PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
 lcd_set_4bit();
 lcd_init(); 
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
  
  // If you want, you can set gyroscope offsets
  // mpu.setGyroOffsetX(155);
  // mpu.setGyroOffsetY(15);
  // mpu.setGyroOffsetZ(15);
  
  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro();

  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu.setThreshold(3);
  
  // Check settings
  checkSettings();
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
SIGNAL(SIG_USART0_RECV)     // ISR for receive complete interrupt
{
  UDR0 = 0x55;        //echo data back to PC
}
void loop()
{
  Vector rawGyro = mpu.readRawGyro();
  Vector normGyro = mpu.readNormalizeGyro();
  delay(500);
  lcd_wr_command(0x01);
  lcd_cursor(1,1);
  lcd_string(" Xraw = ");
  lcd_print(2,1,rawGyro.XAxis,5);
  delay(500);
  lcd_wr_command(0x01);
  lcd_cursor(1,1);
  lcd_string(" Yraw = ");
  lcd_print(2,1,rawGyro.YAxis,5);
  delay(500);
  lcd_wr_command(0x01);
  lcd_cursor(1,1);
  lcd_string(" Zraw = ");
  lcd_print(2,1,rawGyro.ZAxis,5);

  delay(500);
  lcd_wr_command(0x01);
  lcd_cursor(1,1);
  lcd_string(" Xnorm = ");
  lcd_print(2,1,normGyro.XAxis,5);
  delay(500);
  lcd_wr_command(0x01);
  lcd_cursor(1,1);
  lcd_string(" Ynorm = ");
  lcd_print(2,1,normGyro.YAxis,5);
  delay(500);
  lcd_wr_command(0x01);
  lcd_cursor(1,1);
  lcd_string(" Znorm = ");
  lcd_print(2,1,normGyro.ZAxis,5);
  

}

