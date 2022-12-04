
#include <LiquidCrystal.h>
#include<Servo.h>

/* Macro Definition  */
#define massSensor				A0
#define cruiseSpeedDial 		A2
#define cruiseDistanceDial 		A3

/* Object Declaration */
Servo myservo1,myservo2;
LiquidCrystal lcd(12, 5, 4, 2, 1, 0);

/* Global Variables Declaration*/
const int pingPin = 7; 
const int servo1=9;				//Pin connected to the Throttle Servo
const int servo2=10;			//Pin connected to the Brake Servo
const int start=8;				//Pin connected to the ACC Master Control Switch
const int engineRPM=11;			//Pin connected to the Motor
const int speedSensor=3;		//Pin connected to Simulated Car Speed Sensor
int distance=0;					//Variable to store the distance of the leading vehicle from the host vehicle			
int distanceSet=0;				//Variable to store the minimum distance set by the user, that is to be maintained in ACC mode
int speed=0;					//Variable to store the speed of the Host vehicle
int speedSet=0;					//Variable to store the cruise speed from the user
int tempSpeedSet=0;				//Variable to store temporary speed value
int mass=0;						//Variable to store the total mass of the vehicle including passengers
int servoValue=0;				//Variable to store angle values of servos
long duration=0;				//Variable to store the Pulse Duration in Radar
double error=0;					//Variable to store error in speed
double lastError=0;				//Variable to store the last error in speed	
double cumError=0;				//Variable to store the cummulative error over time
double rateError=0;				//Variable to store the change of rate of error over time
double outputPWM=0;				//Final Output to be given to the Motors
double kp=15;					//Variable to store the Propotional Gain
double ki=1;					//Variable to store the Integral Gain
double kd=0.5;					//Variable to store the Derivative Gain
double force=0;					//Variable to store the overall force acting on the simulated car
double friction=0;				//Variable to store the frictional force acted upon the car
double drag=0;					//Variable to store the drag force acted upon the car
int brakeForce=0;			//Variable to store the BreakForce needed to stop the car
double acceleration=0;			//Variable to store the acceleration of the car

/* Function Prototypes*/
void defaultPos(void);			//Function that runs when system is Off
void startACC(void);			//Function that runs when system is On
void getDistance(void);			//Function to set the minimum allowed trailing distance from the user
void getSpeed(void);			//Function to set the cruise speed from the user
void checkRadar(void);			//Function to get vehicle position at front  
void getMass(void);				//Function to get the vehicle and passenger Mass
void getSimulatedSpeed(void);	//Function to get the Simulated vehicle speed
void PID(void);					//Function to Control the PWM output using PID Control Algorithm
void simulateCar(void);			//Function to simulate the linear movement of a car
void cutSpeed(void);			//Function to reduce vehicle speed due to incoming traffic
void brake(void);				//Function to calculate brake force to stop the vehicle

/* Function Definitions*/
void setup()
{
//  Serial.begin(9600);
  lcd.begin(16, 2);
  pinMode(cruiseSpeedDial,INPUT);
  pinMode(cruiseDistanceDial,INPUT);
  pinMode(speedSensor,INPUT);
  pinMode(start,INPUT);
  pinMode(engineRPM,OUTPUT);
  myservo1.attach(9);
  myservo2.attach(10);
  myservo1.write(90);
  delay(15);
  myservo2.write(90);
  delay(15);
  lcd.write("Adaptive Cruise");
  lcd.setCursor(0,1);
  lcd.write(" Control System");
  delay(1500);
  myservo1.write(0);
  delay(15);
  myservo2.write(0);
  delay(15);
  delay(1500);
  lcd.clear();
}
void checkRadar(void)
{
	pinMode(pingPin, OUTPUT);
    digitalWrite(pingPin, LOW);
    delayMicroseconds(2);
    digitalWrite(pingPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(pingPin, LOW);
    pinMode(pingPin, INPUT);
    duration = pulseIn(pingPin, HIGH);

    // convert the time into a distance
    distance = duration/29/2;

    lcd.setCursor(11,1);
  	lcd.print(distance);
  	lcd.print("m ");
  	
}
void defaultPos(void)
{
  	lcd.setCursor(7,1);
  	lcd.write("OFF ");
  	lcd.setCursor(11,0);
  	lcd.print(speed);
  	lcd.write("Kmhr");
  	getSpeed();
  	getDistance();
  	checkRadar();
  	getMass();
}

void getDistance(void)
{
	distanceSet = (analogRead(cruiseDistanceDial)*0.1953125)+100;
  	lcd.setCursor(0,1);
    lcd.print(distanceSet);
    lcd.print("m ");
}
 
void getSpeed(void)
{
	speedSet = (analogRead(cruiseSpeedDial)*0.1171875)+30;	
  	lcd.setCursor(0,0);
    lcd.print(speedSet);
    lcd.print("Km ");
}

void getMass(void)
{
	mass = (analogRead(massSensor)*0.48828125)+1500;	
  	lcd.setCursor(6,0);
    lcd.print(mass);
}

void cutSpeed(void)
{
	tempSpeedSet=speedSet-((300-distance)*0.003*speedSet);  
  	if(speedSet!=tempSpeedSet)
      speedSet=tempSpeedSet;
}

void brake(void)
{
	brakeForce=(mass*((speed*speed)+1))/(2*distance);
  	outputPWM=0;
}

void PID(void)
{
	error=speedSet-speed;
  	cumError+=error * 0.8; 		//0.8 is the simulation time
  	rateError=(error - lastError)/0.8;
  	outputPWM=(kp*error) + (ki*cumError) + (kd*rateError);
  	lastError=error;
    analogWrite(engineRPM,outputPWM);
}

void simulateCar(void)
{
  	friction=0.7*mass*9.8;				//0.7 is the assumed coefficint of friction
  	drag=(0.3*1.225*(speed*speed)*4)/2;	//03 is the assumed drag coefficient, 
      									//1.225 is the assumed density of the car
      									//4 square metres is the assumed surface area
	force=(outputPWM*78.125)-brakeForce-friction-drag;
  	acceleration=force/mass;
  	speed+=acceleration*0.8;
  	if(speed<0)
      speed=0;
  	lcd.setCursor(11,0);
    lcd.print(speed);
    lcd.print("Km ");
  	servoValue=(outputPWM*0.703125);
  	myservo1.write(servoValue);
  	delay(15);
  	servoValue=brakeForce;
  	myservo2.write(servoValue);
}



void startACC(void)
{
  	getSpeed();
  	getDistance();
  	getMass();
  	lcd.setCursor(7,1);
   	lcd.print("ON ");
  	checkRadar();
  
  	if(distance>=distanceSet)
    {
      brakeForce=0;
      PID();
	  simulateCar();
    }
  	else if((distance<distanceSet)&&(distance>=30))
    {
      brakeForce=0;
      cutSpeed();
      PID();
	  simulateCar();
    }
  	else
    {
      brake();
      brakeForce+=30;
      simulateCar();
    }

}

void loop()
{
	if(digitalRead(start)==HIGH)	//Checking for switch On
  		startACC();
  	else
      	defaultPos();
}
