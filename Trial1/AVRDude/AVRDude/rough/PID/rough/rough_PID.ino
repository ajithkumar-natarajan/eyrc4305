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
//  delay(500);
}

void right_sharp (void) //Left wheel forward, Right wheel backward
{
  motion_set(0x0A);
//  delay(500);
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

int weight=0;
int lastError = 0;
int Kp=10;
int rightBaseSpeed=160;
int leftBaseSpeed=160;


void loop() {

  unsigned char Left= 0;
  unsigned char Center= 0;
  unsigned char Right= 0;
  //unsigned char BATT_Voltage=0;
  int white=0;
  int flag=0;

  int lt=0;
  int rt=0;
  int st=0;
  
  Center = ADC_Conversion(2);
  Left = ADC_Conversion(3);
  Right = ADC_Conversion(1);
  //BATT_Voltage = ((ADC_Conversion(0)*100)*0.07902) + 0.7;

    print_sensor(1,1,3);  //Prints value of White Line Sensor1
    print_sensor(1,5,2);  //Prints Value of White Line Sensor2   
    print_sensor(1,9,1);  //Prints Value of White Line Sensor3

    int error= 0-weight;  
    int motorSpeed = Kp * error;
    int lastError = error;

    int rightMotorSpeed = rightBaseSpeed + motorSpeed;
    int leftMotorSpeed = leftBaseSpeed - motorSpeed;

    if (rightMotorSpeed<0)
      rightMotorSpeed=0;

    if (leftMotorSpeed<0)
      leftMotorSpeed=0 ;

      if (rightMotorSpeed>256)
      rightMotorSpeed=256;

    if (leftMotorSpeed>256)
      leftMotorSpeed=256 ;
    
    //print_sensor(2,1,BATT_Voltage);

    if( Left>0x28 && Center>0x28 && Right>0x28)
    {
       forward();
       velocity(rightMotorSpeed, leftMotorSpeed);
       left_sharp();
    }

    if( Left>0x28 && Center>0x28 && Right<0x28)
    {
     left();
     velocity(rightMotorSpeed, leftMotorSpeed);
      lt=1;
      weight=-1;
    }

    if( Left<0x28 && Center>0x28 && Right<0x28)
    {
      forward();
      velocity(180,180);
      st=1;
      weight=0;
    }

    if( Left>0x28 && Center<0x28 && Right<0x28)
    {
       
      left_sharp();
      velocity(rightMotorSpeed, leftMotorSpeed);
      lt=1;
      weight=-2;
    }

    if( Left<0x28 && Center>0x28 && Right>0x28)
    {
       
       right();
      velocity(rightMotorSpeed, leftMotorSpeed);
      rt=1;
      weight=1;
    }

    if( Left<0x28 && Center<0x28 && Right>0x28)
    {
     
       right_sharp();
      velocity(rightMotorSpeed, leftMotorSpeed);
      rt=1;
      weight=2;
    }

   if(Center<0x28 && Left<0x28 && Right<0x28 && white==0)
    {
      if (st==1)
        rev();
      if (lt==1)
        right();
      if (rt==1)
        left();
      delay(150); 
     
 //     white=1;
    }
    

//    if(Center<0x28 && Left<0x28 && Right<0x28 && white==1)
  //  {
    //  stp();
    //}


  /*  if(Center>0x28)
    {
      forward();
      velocity(150,150);
      flag=1;
    }

    if( Left>0x28 && flag==1 )
    {
      left_sharp();
      velocity(105,105);
    }

    if(Right>0x28 && flag==1)
    {
      right_sharp();
      velocity(105,105);
    }

    if(Center<0x28 && Left<0x28 && Right<0x28 && white==0)
    {
      delay(500);
      white=1;
    }

    if(Center<0x28 && Left<0x28 && Right<0x28 && white==1)
    {
      stp();
    }
*/

}
