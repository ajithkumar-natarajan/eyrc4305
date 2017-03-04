#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h> //included to support power function
#include "lcd.c"


void motion_pin_config (void)
{
 DDRA = DDRA | 0x0F; //set direction of the PORTA 3 to PORTA 0 pins as output
 PORTA = PORTA & 0xF0; // set initial value of the PORTA 3 to PORTA 0 pins to logic 0
 DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM
}

void adc_pin_config (void)
{
 DDRF = 0x00; 
 PORTF = 0x00;
 DDRK = 0x00;
 PORTK = 0x00;
}

void adc_init()
{
  ADCSRA = 0x00;
  ADCSRB = 0x00;    //MUX5 = 0
  ADMUX = 0x20;   //Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
  ACSR = 0x80;
  ADCSRA = 0x86;    //ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

void uart0_init(void)
{
 UCSR0B = 0x00; //disable while setting baud rate
 UCSR0A = 0x00;
 UCSR0C = 0x06;
 UBRR0L = 0x5F; //set baud rate lo
 UBRR0H = 0x00; //set baud rate hi
 UCSR0B = 0x98;
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

void lcd_port_config (void)
{
 DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
 PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}

void print_sensor(char row, char coloumn,unsigned char channel)
{
  unsigned char ADC_Value;
  ADC_Value = ADC_Conversion(channel);
  lcd_print(row, coloumn, ADC_Value, 3);
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

void rev (void)
{
  motion_set (0x09);
}

void stp (void)
{
  motion_set (0x00);
}

void left_sharp (void) //Left wheel backward, Right wheel forward
{
  motion_set(0x05);
}

void right_sharp (void) //Left wheel forward, Right wheel backward
{
  motion_set(0x0A);
}

void right (void)
{
  motion_set(0x02);
//  delay(2000);
}

void left (void)
{
  motion_set(0x04);
//  delay(2000);
}

void velocity (unsigned char left_motor, unsigned char right_motor)
{
  OCR5AL = (unsigned char)left_motor;
  OCR5BL = (unsigned char)right_motor;
}

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

void init_devices (void)
{
  cli(); //Clears the global interrupts
  adc_init();
  adc_pin_config();
  motion_pin_config();
  lcd_port_config();
  timer5_init();
  lcd_set_4bit();
  lcd_init();
  uart0_init();
  sei();   //Enables the global interrupts
}



void setup() {
  init_devices();
  unsigned char flag = 0;

  pinMode(A2,INPUT);
  pinMode(A3,INPUT);
  pinMode(A1,INPUT);
  pinMode(22,OUTPUT);
  pinMode(23,OUTPUT);
  pinMode(24,OUTPUT);
  pinMode(25,OUTPUT);
  pinMode(26,OUTPUT);
  pinMode(27,OUTPUT);
  pinMode(28,OUTPUT);
  pinMode(29,OUTPUT);
  // put your setup code here, to run once:

}
void loop() {
  // put your main code here, to run repeatedly:
/*      
      rev();
      delay(770);
      left();
      delay(1000);
      forward();
      delay(130);
      stp();
      delay(2000);
*/

/*      right();
      delay(1100);
      forward();
      delay(500);
      left();
      delay(1100);
      forward();
      delay(50);
      while(1)
      stp();
*/

/*    
      right_sharp();
      delay(300);
      forward();
      delay(250);
      right_sharp();
      delay(100);
      forward();
      delay(150);
      right_sharp();
      delay(200);
      forward();
      delay(800);
      left_sharp();
      delay(525);
      forward();
      delay(400);
      while(1)
      stp();



/*
      rev();
      delay(540);
      left_sharp();
      delay(500);
      forward();
      delay(290);
      while(1)
      stp();
*/

/*      
      rev();
      delay(520);
      left_sharp();
      delay(500);
      forward();
      delay(320);
      stp();
      delay(2000);
      rev();
      delay(290);
      right_sharp();
      delay(600);
      forward();
      delay(540);
      right_sharp();

      
      delay(300);
      forward();
      delay(250);
      right_sharp();
      delay(100);
      forward();
      delay(150);
      right_sharp();
      delay(200);
      forward();
      delay(800);
      left_sharp();
      delay(530);
      forward();
      delay(400);
      while(1)
      stp();

*/

/*
      rev();
      delay(1250);
      right_sharp();
      delay(620);
      forward();
      delay(330);
      stp();
      delay(2000);
      rev();
      delay(390);
      left_sharp();
      delay(550);
      forward();
      delay(1230);

      
      right_sharp();
      delay(300);
      forward();
      delay(250);
      right_sharp();
      delay(100);
      forward();
      delay(150);
      right_sharp();
      delay(200);
      forward();
      delay(1000);
      left_sharp();
      delay(515);
      forward();
      delay(200);
      while(1)
      stp();
    
*/    delay(6000);

      right_sharp();
      delay(500);
      while(1)
      stp();
      
}
