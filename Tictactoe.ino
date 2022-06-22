#include <AccelStepper.h>

int first=0;
const float pi=3.14;
int X,Y;
float L1=250,L2=200;
float S1,C1,S2,C2,t1,t2,px,py;
long pulse1,pulse2;
int16_t delpulse1,delpulse2;
int16_t v1,v2,vmax;
int x[10] = {0,175,275,375,0};
int y[10] = {0,100,0,-100,450};

#define dirPin1 5
#define stepPin1 2
#define dirPin2 6
#define stepPin2 3
#define motorInterfaceType 1
const int stepPin3 = 4; // Z AXES IS PIN 3  
const int dirPin3 = 7; 
AccelStepper stepper1(1, 2, 5);
AccelStepper stepper2(1, 3, 6);
void setup() {
 Serial.begin(9600);
 if(first==0){delay(3000);first=1;}
 pinMode(stepPin1,OUTPUT); 
 pinMode(dirPin1,OUTPUT);
 pinMode(stepPin2,OUTPUT); 
 pinMode(dirPin2,OUTPUT); 
 pinMode(stepPin3,OUTPUT); 
 pinMode(dirPin3,OUTPUT); 
 pinMode(8, OUTPUT); // ENABLE PIN, RUN WHEN LOW
 digitalWrite(8, LOW);
 stepper1.setMaxSpeed(500);
 stepper1.setAcceleration(200);
 stepper2.setMaxSpeed(1000);
 stepper2.setAcceleration(500);
}
void inverse_kinematic()
{
 C2=(pow(X,2)+pow(Y,2)-pow(L1,2)-pow(L2,2))/(2*L1*L2);
 S2=sqrt(abs(1-pow(C2,2)));
 t2=atan2(S2,C2);
 
 C1=(X*(L1+L2*C2)+Y*L2*S2)/(pow((L1+L2*C2),2)+pow((L2*S2),2));
 S1=(Y*(L1+L2*C2)-X*L2*S2)/(pow((L1+L2*C2),2)+pow((L2*S2),2));
 t1=atan2(S1,C1);
 px=L1*cos(t1)+L2*cos(t1+t2);
 py=L1*sin(t1)+L2*sin(t1+t2);
}
void calculate_pulse()
{
 pulse1 = -(t1*1600)/(2*pi) ;
 pulse2 = -(t2*1600)/(2*pi) +0*pulse1;
}
void stepper3(float so_vong)
{
 int opt=100;
 float theta=so_vong;
 float pulse3=0;
 float t3=0;
 if (theta!=0)
 { 
 Serial.println(theta);
 if(theta>10)//12--> - ANGLE
 {
 t3=theta-10;
 digitalWrite(dirPin3,LOW);
 Serial.println("DOWN");
 }
 else if (theta<10)//0--> + ANGLE
 {
 t3=theta;
 digitalWrite(dirPin3,HIGH);
 Serial.println("UP");
 }
 // TRANFER DEGREE TO PULSE
 pulse3=(360*t3)/0.225; 
 Serial.print("NUMBER OF PULSE : ");
 Serial.println(pulse3);
 //STEPPER3 
 for(int i = 0; i < pulse3; i++) 
 {
 digitalWrite(stepPin3,HIGH);
 delayMicroseconds(opt);
 digitalWrite(stepPin3,LOW);
 delayMicroseconds(opt);
 }
 }
}
void go_home(void)
{
 X=0;
 Y=450;
 inverse_kinematic();
 calculate_pulse();
 
 stepper1.moveTo(pulse1); 
 stepper2.moveTo(pulse2);
 
 while((stepper1.distanceToGo() != 0)||(stepper2.distanceToGo() != 0))
 {
 stepper1.run();
 stepper2.run();
 }
 
 Serial.println("done");
}
void calib(void)
{
 int val[100] = {0,11,12,13,23,22,21,31,32,33};
 for(int j = 1; j <=9 ; j++)
 {
 //DECRYPT THE CHAR
 X=x[val[j]/10];
 Y=y[val[j]%10];
 Serial.print("X COORDINNATE: ");
 Serial.println(X);
 Serial.print("Y COORDINATE: ");
 Serial.println(Y);
 
 inverse_kinematic();
 calculate_pulse();
 
 stepper1.moveTo(pulse1); 
 stepper2.moveTo(pulse2);
 
 while((stepper1.distanceToGo() != 0)||(stepper2.distanceToGo() != 0))
 {
 stepper1.run();
 stepper2.run();
 }
 delay(700);//WAIT FOR BALANCE
 }
 go_home();
}


                  void drawCircle()
                  {
                   Serial.println("DRAW O");
                   int Radius=20;
                   float a,b;
                   a=X;b=Y;
                   for (int t=0;t<360;t++)
                   {
                    // Equation of a circle in parametric form
                    X = Radius * cos(t *(pi/180)) + a; 
                    Y = Radius * sin(t *(pi/180)) + b;
                    inverse_kinematic();
                    calculate_pulse();
                   
                    stepper1.moveTo(pulse1); 
                    stepper2.moveTo(pulse2);
                   
                    while((stepper1.distanceToGo() != 0)||(stepper2.distanceToGo() != 0))
                    {
                    stepper1.run();
                    stepper2.run();
                    }
                    delay(1);
                   }
                  }

void drawX()
{
  Serial.println("DRAW X");
  float a,b; a=X;b=Y;
  for (int t=-25;t<=25;t=t+1)
   {
    // Equation of a circle in parametric form
    X = a+t; 
    Y = b+t;
    inverse_kinematic();
    calculate_pulse();
   
    stepper1.moveTo(pulse1); 
    stepper2.moveTo(pulse2);
   
    while((stepper1.distanceToGo() != 0)||(stepper2.distanceToGo() != 0))
    {
    stepper1.run();
    stepper2.run();
    }
    delay(10);
   }
   /////////////////
   stepper3(2.5);
   X= a-25;
   Y= b+25;
   inverse_kinematic();
    calculate_pulse();
   
    stepper1.moveTo(pulse1); 
    stepper2.moveTo(pulse2);
   
    while((stepper1.distanceToGo() != 0)||(stepper2.distanceToGo() != 0))
    {
    stepper1.run();
    stepper2.run();
    }
    delay(10);
   stepper3(12.5);
  ////////////////
  for (int t=-25;t<=25;t=t+1)
   {
    // Equation of a circle in parametric form
    X = a+t; 
    Y = b-t;
    inverse_kinematic();
    calculate_pulse();
   
    stepper1.moveTo(pulse1); 
    stepper2.moveTo(pulse2);
   
    while((stepper1.distanceToGo() != 0)||(stepper2.distanceToGo() != 0))
    {
    stepper1.run();
    stepper2.run();
    }
    delay(10);
   }
   
  
}

void loop() {
 // if there's any Serial available, read it: 
 while (Serial.available() == 0){}
 int val=Serial.parseFloat();
 if(val==99) { go_home();}
 if(val==77) { calib();}
 if ((val!=0)&&(val!=99)&&(val!=77))
 {
 //IN GIÁ TRỊ NHẬN ĐƯỢC
 Serial.println("CHOOSE THE BOX <11 -> 13> ");
 Serial.println(val);
 //DECRYPT THE CHAR
 X=x[val/10];
 Y=y[val%10];
 Serial.print("Toa do X: ");
 Serial.println(X);
 Serial.print("Toa do Y: ");
 Serial.println(Y);
 
 inverse_kinematic();
 calculate_pulse();
 stepper1.moveTo(pulse1); 
 stepper2.moveTo(pulse2);
 while((stepper1.distanceToGo() != 0)||(stepper2.distanceToGo() != 0))
 {
 stepper1.run();
 stepper2.run();
 }
 delay(500);//WAIT FOR BALANCE
 stepper3(12.5);
 delay(500);//HEAD OF THE PEN REACH THE BOARD

int xo =0;
 Serial.println("PRESS 1 FOR DRAW X, 2 FOR DRAW O");
while(xo==0){ xo=Serial.parseInt(); }
 if (xo==1)
 drawX(); else if(xo==2) drawCircle();
 
 stepper3(2.5);
 delay(500);//HEAD OF THE PEN REACH THE BOARD
 Serial.println("done1");
 go_home();
 Serial.println("done!");
 }
}
///////////////
