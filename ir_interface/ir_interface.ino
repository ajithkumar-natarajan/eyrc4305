#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h> //included to support power function
#include "lcd.h"
unsigned char Front_Sharp_Sensor=0;
unsigned char Front_IR_Sensor=0;
unsigned char Left_IR_Sensor=0;
unsigned char Right_IR_Sensor=0;
void lcd_port_config (void)
{
 DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
 PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}
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
  float ADC_Value;
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
}

void left (void)
{
  motion_set(0x04);
}


void setup() {
   cli(); //Clears the global interrupts
  port_init();
  adc_init();
  sei();  
  lcd_set_4bit();
  lcd_init();
  
  // put your setup code here, to run once:

}

void loop() {

  Front_Sharp_Sensor = ADC_Conversion(11);
    Front_IR_Sensor = ADC_Conversion(6);
    Left_IR_Sensor = ADC_Conversion(5);
    Right_IR_Sensor = ADC_Conversion(7);



    print_sensor(2,4,11);  //Prints Value of Front Sharp Sensor
    print_sensor(2,8,6);  

    if (Left_IR_Sensor > 0x96 && Front_IR_Sensor < 0x96 && Right_IR_Sensor < 0x96 ){
    right_sharp();
    delay(400);
    }

    else if (Left_IR_Sensor < 0x96 && Front_IR_Sensor > 0x96 && Right_IR_Sensor < 0x96 ){
    rev();
    delay(400);
    }

    else if (Left_IR_Sensor < 0x96 && Front_IR_Sensor < 0x96 && Right_IR_Sensor > 0x96 ){
    left_sharp();
    delay(400);
    }

  
  // put your main code here, to run repeatedly:

}
