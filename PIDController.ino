#include <Ultrasonic.h>
#include <Servo.h>

#define LOWER 65
#define MID 80
#define UPPER 110

#define TARGET 20
#define KP 0.6  //proportional constant
#define KI 0.005     //integral constant
#define KD 30     //derivative constant
#define SAMPLEDELAY 1 //sample delay

#define P true  //which elements are included in the controller?
#define I false
#define D false

double p;           //p term final
double i = 0;           //i term final
double d;           //d term final
double error;       // distance from setpoint
double lastError;   //saved error value for d term
double distance;    // recorded distance variable
double distBuffer;  //error recovery for ultrasonic misses - takes the last recorded value
double theta;       //value written to the servo after calculations complete

Ultrasonic ultrasonic(12, 13);  //(trig, echo)
Servo myservo;

void setup() {
  Serial.begin(9600);
  myservo.attach(3);
  myservo.write(MID);
  distBuffer = ultrasonic.read();
  delay(2000);
}

void loop() {

  // MEASUREMENT AND CALCULATION

  distance = ultrasonic.read();

  if (distance > 40) {            //ultrasonic error recovery
    distance = distBuffer;
  }
  distBuffer = distance;

  error = distance - TARGET;      //PID error calculation

  p = KP * error;                 //p calculation
  i = i + KI * error;                 //i calculation
  d = ((error - lastError) / SAMPLEDELAY) * KD;
  
  theta = myservo.read();
  if(P) theta += p;
  if(I) theta += i;
  if(D) theta += d;

  lastError = error;
  if (theta < LOWER) theta = LOWER;
  if (theta > UPPER) theta = UPPER;

  Serial.print(distance); Serial.print('\t'); Serial.print(error); Serial.print('\t'); Serial.println(theta);

  myservo.write(theta);
  delay(SAMPLEDELAY);

}
