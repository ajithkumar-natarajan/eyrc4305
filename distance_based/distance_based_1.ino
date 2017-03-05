#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h> //included to support power function
#include "lcd.c"
#include <Servo.h>


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
  velocity(200,200);
}

void rev (void)
{
  motion_set (0x09);
  velocity(200,200);
}

void stp (void)
{
  motion_set (0x00);
}

void left_sharp (void) //Left wheel backward, Right wheel forward
{
  motion_set(0x05);
  velocity(200,200);
}

void right_sharp (void) //Left wheel forward, Right wheel backward
{
  motion_set(0x0A);
  velocity(200,200);
}

void right (void)
{
  motion_set(0x02);
  velocity(200,200);
}

void left (void)
{
  motion_set(0x04);
  velocity(200,200);
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


Servo servo1;

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
  pinMode(34,OUTPUT);
  servo1.attach(10);

}

int xco, xcod;
char xc,yc;
//char input[5];
int yco, ycod;
int flag=0;
char boul;
int xdistance, ydistance;
float xtimer,ytimer,xrtimer,yrtimer;
//int j=0,k=0;
 
int pos=0;
int cno;

void start(void){
    for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    servo1.write(pos);                  // tell servo to go to position in variable 'pos'
    delay(15);                          // waits 15ms for the servo to reach the position
  }
      stp();
      flag=0;
      boul='*';
}

void boulder_1(void){
      forward();
      delay(1.13*xtimer);
      right_sharp();
      delay(570);
      forward();
      delay(0.9*ytimer);

      stp();
      delay(5000);

      boulder_1r();

      flag=0;
}

void boulder_2(void){
      
      for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
      servo1.write(pos);                  // tell servo to go to position in variable 'pos'
      delay(15);                          // waits 15ms for the servo to reach the position
      }
      
      forward();
      delay(0.97*xtimer);
      stp();
      delay(500);
      right_sharp();
      delay(470);

      forward();
      delay(2*ytimer);
      stp();
      delay(500);
      
      for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees in steps of 1 degree
      servo1.write(pos);                  // tell servo to go to position in variable 'pos'
      delay(15);                          // waits 15ms for the servo to reach the position
      }

      
  
      stp();
      delay(500);
      left_sharp();
      delay(470);
      stp();
      delay(500);
      rev();
      delay(0.97*xtimer);
      forward();
      delay(2*ytimer);

      //boulder_2r();
      bridge_1();
      flag=0;
}


void boulder_3(void){
      forward();
      delay(1.15*xtimer);
      left_sharp();
      delay(540);
      forward();
      delay(ytimer);

      for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees in steps of 1 degree
      servo1.write(pos);                  // tell servo to go to position in variable 'pos'
      delay(15);                          // waits 15ms for the servo to reach the position
      }

      stp();
      delay(500);

      boulder_3r();

      flag=0;
    }


void boulder_4(void){
      forward();
      delay(2*xtimer);
      left_sharp();
      delay(540);
      forward();
      delay(1.1*ytimer);

      stp();
      delay(5000);

      boulder_4r();

      flag=0;
    }



void boulder_1r(void){
      rev();
      delay(1.35*xrtimer);
      right_sharp();
      delay(570);
      forward();
      delay(0.85*yrtimer);

      flag=0;
}

void boulder_2r(void){
      rev();
      delay(xrtimer);
      right_sharp();
      delay(580);
      forward();
      delay(1.3*yrtimer);

      bridge_1();
      flag=0;
}


void boulder_3r(void){
      rev();
      delay(1.2*xrtimer);
      left_sharp();
      delay(525);
      forward();
      delay(0.9*yrtimer);

      flag=0;
    }


void boulder_4r(void){
      rev();
      delay(1.3*xrtimer);
      left_sharp();
      delay(515);
      forward();
      delay(1.8*yrtimer);

      flag=0;
    }


    void bridge_1(void){
      //left_sharp();
      //delay(520);
      right_sharp();
      delay(580);
      forward();
      delay(1.49*xtimer);

      right_sharp();
      delay(560);
      forward();
      delay(0.83*ytimer);
      
      stp();
      delay(5000);


      flag=0;
}

/*
    void bridge_2(void){
      right_sharp();
      delay(570);
      forward();
      delay(0.9*ytimer);

      stp();
      delay(5000);


      flag=0;
}
*/

    void bridge_2(void){
      right_sharp();
      delay(550);
      forward();
      delay(1.34*xtimer);

      left_sharp();
      delay(525);
      forward();
      delay(0.73*ytimer);

      stp();
      delay(5000);


      flag=0;

}

/*
    void bridge_4(void){
      left_sharp();
      delay(540);
      forward();
      delay(0.9*ytimer);

      stp();
      delay(5000);


      flag=0;
}
*/

void bridge_1r(void){
      rev();
      delay(0.65*ytimer);
      right_sharp();
      delay(580);

      forward();
      delay(1.45*xtimer);
      right_sharp();
      delay(560);

      stp();
      delay(5000);


      flag=0;
}

    void bridge_2r(void){
      rev();
      delay(0.65*ytimer);
      left_sharp();
      delay(520);

      forward();
      delay(1.4*xtimer);
      left_sharp();
      delay(540);

      stp();
      delay(5000);


      flag=0;

}

      void station_1(void){
      forward();
      delay(1.05*xtimer);

      right_sharp();
      delay(575);
      forward();
      delay(1.175*xtimer);

      stp();
      digitalWrite(34,HIGH);
      delay(5000);
      digitalWrite(34,LOW);

      flag=0;


      flag=0;
}

/*
      void station_2(void){
      right_sharp();
      delay(575);
      forward();
      delay(1.175*xtimer);

      stp();
      digitalWrite(34,HIGH);
      delay(5000);
      digitalWrite(34,LOW);

      flag=0;

}
*/

      void station_2(void){
      forward();
      delay(1.05*xtimer);

      left_sharp();
      delay(545);
      forward();
      delay(1.22*xtimer);

      stp();
      digitalWrite(34,HIGH);
      delay(5000);
      digitalWrite(34,LOW);

      flag=0;
}

/*
      void station_4(void){
      left_sharp();
      delay(545);
      forward();
      delay(1.22*xtimer);

      stp();
      digitalWrite(34,HIGH);
      delay(5000);
      digitalWrite(34,LOW);


      flag=0;

}
*/

/*void temp1(void){
      left_sharp();
      velocity(150,150);
      delay(1300);
      forward();
      velocity(150,150);
      delay(1600);
      left_sharp();
      velocity(150,150);
      delay(1300);
      forward();
      velocity(150,150);
      delay(750);
      stp();

      flag=0;
}
*/

/*int bridge_c1(){
      forward();
      delay(1*1.589*xtimer);
      stp();
      for (pos = 0; pos <= 180; pos += 1){
        servo1.write(pos);
        delay(15);
  }
      for (pos = 180; pos >= 0; pos -= 1) { 
        servo1.write(pos);
        delay(15);
  }
}

int bridge_c2(){
      forward();
      delay(2*1.589*xtimer);
      stp();
      for (pos = 0; pos <= 180; pos += 1){
        servo1.write(pos);
        delay(15);
  }
      for (pos = 180; pos >= 0; pos -= 1) { 
        servo1.write(pos);
        delay(15);
  }
}

int bridge_c3(){
      forward();
      delay(3*1.589*xtimer);
      stp();
      for (pos = 0; pos <= 180; pos += 1){
        servo1.write(pos);
        delay(15);
  }
      for (pos = 180; pos >= 0; pos -= 1) { 
        servo1.write(pos);
        delay(15);
  }
}

int bridge_c4(){
      forward();
      delay(4*1.589*xtimer);
      stp();
      for (pos = 0; pos <= 180; pos += 1){
        servo1.write(pos);
        delay(15);
  }
      for (pos = 180; pos >= 0; pos -= 1) { 
        servo1.write(pos);
        delay(15);
  }
}

int bridge_c5(){
      forward();
      delay(5*1.589*xtimer);
      stp();
      for (pos = 0; pos <= 180; pos += 1){
        servo1.write(pos);
        delay(15);
  }
      for (pos = 180; pos >= 0; pos -= 1) { 
        servo1.write(pos);
        delay(15);
  }
}

int bridge_c6(){
      forward();
      delay(5*1.589*xtimer);
      stp();
      for (pos = 0; pos <= 180; pos += 1){
        servo1.write(pos);
        delay(15);
  }
      for (pos = 180; pos >= 0; pos -= 1) { 
        servo1.write(pos);
        delay(15);
  }
}

int bridge_c7(){
      forward();
      delay(7*1.589*xtimer);
      stp();
      for (pos = 0; pos <= 180; pos += 1){
        servo1.write(pos);
        delay(15);
  }
      for (pos = 180; pos >= 0; pos -= 1) { 
        servo1.write(pos);
        delay(15);
  }
}
*/

int bridge_c9(){
      right_sharp();
      delay(173);
      stp();
      for (pos = 180; pos >= 0; pos -= 1) { 
        servo1.write(pos);
        delay(15);
  }
      rev();
      delay(500);
      forward();
      delay(500);
      left_sharp();
      delay(173);
}

int bridge_c16(){
      left_sharp();
      delay(173);
      stp();
      for (pos = 180; pos >= 0; pos -= 1) { 
        servo1.write(pos);
        delay(15);
  }
      rev();
      delay(500);
      forward();
      delay(500);
      right_sharp();
      delay(173);
}

/*void temp1(void){
      //stp();
      //delay(10000);
      //while(!Serial.available());
      //while(Serial.available()){
      //cno=Serial.read();
      //}
      //if(cno>0){
        bridge_c1();
      //}

      flag=0;
      cno=0;
}
*/

void loop() { 

while (Serial.available()){


/*  if (flag==0){
    String instr = Serial.readString();
    instr.toCharArray(input,1) ;
    flag=1;
  }
*/  
  
  if (flag==0){
    boul = Serial.read();
    //flag=1;
  }
/*  k=1;
   if(flag==1){
    while((xc=Serial.read())!='/'){
       xco = xco + (xc-'0') * (k);
       k = k*10;
    }
    flag=2;
  }
  k=1;
  if(flag==2){
    while((yc=Serial.read())!='/'){
       yco = yco + (yc-'0') * (k);
       k = k*10;
  }
  }
  */
  
/*  boul=instr[0];
  //boul[k] = '\0';
  //k=0;
  j=2;
  while(instr[j]!='/'){
    xco += instr[j++];
  }
  //xco[k] = '\0';
  //k=0;
  j++;
  while(instr[j]){
    yco += instr[j++];
  }
  //yco[k] = '\0';

/*  
  if (flag==1){
    boul = Serial.read();
    flag=2;
  }
  else if (flag==2){
    xco = Serial.read();
    flag=3;
  }
  else if (flag==3){
    yco = Serial.read();
    flag=0;
  }
*/

/*  if (flag==0){
    boul = Serial.read();
    flag=1;
  }      
  if (flag==1){
    xc = Serial.read();
    xco=xc-'0';
    xco=xco*5;
    flag=2;
  }
  if (flag==2){
    xc = Serial.read();
    xcod=xc-'0';
    flag=3;
  }
  if (flag==3){
    yc = Serial.read();
    yco=yc-'0';
    yco=yco*5;
    flag=4;
  }
  if (flag==4){
    yc = Serial.read();
    ycod=yc-'0';
    flag=5;
  }
*/
  
  //xdistance=xco.toInt();
  //ydistance=yco.toInt();
  //xdistance=xco+xcod;
  //ydistance=yco+ycod;
  //boul=input[0];
  //xdistance=input[1]*10+input[2];
  //ydistance=input[3]*10+input[4];
  //xdistance=250;
  //ydistance=300;
  
  //xtimer=(xdistance*60000)/(12750);
  //ytimer=(ydistance*60000)/(12750);

  xtimer=847.0;
  xrtimer=xtimer/2.6;
  ytimer=xtimer/2;
  yrtimer=2*ytimer;

  //if (flag==1)

    if (boul=='s')
    {
      start();
    }
  
    if (boul=='a')
    {
      boulder_1();
      bridge_1();
    }

    if (boul=='b')
    {
      boulder_2();
    }

    if (boul=='c')
    {
      boulder_3();
      bridge_1();
    }

    if (boul=='d')
    {
      boulder_4();
      bridge_1();
    }

    
    if (boul=='e')
    {
      boulder_1();
      bridge_2();
    }

    if (boul=='f')
    {
      boulder_2();
      bridge_2();
    }

    if (boul=='g')
    {
      boulder_3();
      bridge_2();
    }

    if (boul=='h')
    {
      boulder_4();
        bridge_2();
    }


/*
    if (boul=='e')
    {
      boulder_1r();
    }

    if (boul=='f')
    {
      boulder_2r();
    }

    if (boul=='g')
    {
      boulder_3r();
    }

    if (boul=='h')
    {
      boulder_4r();
    }

*/
/*
    if (boul=='i')
    {
      bridge_1();
    }
    
    if (boul=='k')
    {
      bridge_2();
    }

    if (boul=='m')
    {
      bridge_3();
    }

    if (boul=='o')
    {
      bridge_4();
    }
*/

    if (boul=='i')
    {
      bridge_1r();
    }

    if (boul=='j')
    {
      bridge_2r();
    }
    
    if (boul=='k')
    {
      station_1();
    }

    if (boul=='l')
    {
      station_2();
    }
/*
    if (boul=='s')
    {
      station_3();
    }

    if (boul=='t')
    {
      station_4();
    }
*/
/*    if (boul=='`')
    {
      bridge_c1();
    }

    if (boul=='1')
    {
      bridge_c2();
    }

    if (boul=='2')
    {
      bridge_c3();
    }

    if (boul=='3')
    {
      bridge_c4();
    }

    if (boul=='4')
    {
      bridge_c5();
    }

    if (boul=='5')
    {
      bridge_c6();
    }

    if (boul=='6')
    {
      bridge_c7();
    }
*/
/*    if (boul=='7')
    {
      bridge_c8();
    }
*/
    if (boul=='8')
    {
      bridge_c9();
    }

/*    if (boul=='9')
    {
      bridge_c10();
    }

    if (boul=='0')
    {
      bridge_c11();
    }

    if (boul=='-')
    {
      bridge_c12();
    }

    if (boul=='=')
    {
      bridge_c13();
    }

    if (boul=='[')
    {
      bridge_c14();
    }

    if (boul==']')
    {
      bridge_c15();
    }
*/
    if (boul=='|')
    {
      bridge_c16();
    }

/*    if (boul==';')
    {
      bridge_c17();
    }

    if (boul=='"')
    {
      bridge_c18();
    }

    if (boul==',')
    {
      bridge_c19();
    }

    if (boul=='.')
    {
      bridge_c20();
    }

    if (boul=='/')
    {
      bridge_c21();
    }*/
    if (boul=='+')
    {
        forward();
        delay(7*1.589*xtimer);
    }
}
}
