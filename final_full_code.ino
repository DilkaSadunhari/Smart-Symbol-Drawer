#include <ContinuousStepper.h>

//limit detection
#define IRSensor_L A0
#define IRSensor_R A1
#define IRSensor_count 9
short IR_count = 0;
#define SIGNAL 10
//ink level detection
#define trigPin_1 3
#define echoPin_1 2
#define buzzerPin 6
#define trigPin_2 5
#define echoPin_2 4
#define LEDpin_1 A4
#define LEDpin_2 A3
//ink pumping
#define pumpPin_1 7
#define pumpPin_2 8

// steper moters


const uint8_t step1Pin = 0;
const uint8_t dir1Pin = 1;

const uint8_t step2Pin = 12;
const uint8_t dir2Pin = 11;

ContinuousStepper stepper1;
ContinuousStepper stepper2;

long currentTime = 0;
long previousTime = 0;
long p1Start = 0;
long p2Start = 0;


int activeTime_1 = 0, activeTime_2 = 0;
boolean pumpActive = 0;
void setup() {
  Serial.begin(115200);
  pinMode(IRSensor_R, INPUT);
  pinMode(IRSensor_L, INPUT);
  pinMode(IRSensor_count, INPUT);
  pinMode(SIGNAL, OUTPUT);  // relay signal
  pinMode(trigPin_1, OUTPUT);
  pinMode(echoPin_1, INPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(trigPin_2, OUTPUT);
  pinMode(echoPin_2, INPUT);

  stepper1.begin(step1Pin, dir1Pin);
  stepper1.spin(-20);

  stepper2.begin(step2Pin, dir2Pin);
  stepper2.spin(-20);

  digitalWrite(SIGNAL, LOW);
  digitalWrite(buzzerPin, HIGH);
  delay(100);
  pinMode(pumpPin_1,OUTPUT);
  pinMode(pumpPin_2,OUTPUT);
  digitalWrite(pumpPin_1,HIGH);
  digitalWrite(pumpPin_2,HIGH);
  long systemStartTime = millis();
}
void loop() {

  //limit exceeding detection==============================================================>
  byte sensor1Status = digitalRead(IRSensor_L);
  byte sensor2Status = digitalRead(IRSensor_R);
  if (sensor1Status == 0 || sensor2Status == 0)  // Check if the pin high or not
  {
    digitalWrite(SIGNAL, LOW);  //LOW signal for activate relay-->cut the power
                                //Serial.println("Limit exceeded"); // print Motion Detected! on the serial monitor window
  } else {
    digitalWrite(SIGNAL, HIGH);  //HIGH signal for deactivate relay-->
    //Serial.println("Limit corrected"); // print Motion Detected! on the serial monitor window
  }

  //count===================================================================================>
  if (digitalRead(IRSensor_count) == 0 && IR_count == 0) {
    IR_count++;
  } else if (digitalRead(IRSensor_count) == 1 && IR_count > 0) {
    IR_count = 0;
    //system shifts by 2 feets----------V
    stepper1.loop();
    stepper2.loop();
  }

  //ink level detection====================================================================>
  long duration1, distance1;
  digitalWrite(trigPin_1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin_1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin_1, LOW);
  duration1 = pulseIn(echoPin_1, HIGH);
  distance1 = (duration1 / 2) / 29.1;

  long duration2, distance2;
  digitalWrite(trigPin_2, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin_2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin_2, LOW);
  duration2 = pulseIn(echoPin_2, HIGH);
  distance2 = (duration2 / 2) / 29.1;

  if (distance1 <= 5) {
    digitalWrite(buzzerPin, HIGH);
    analogWrite(LEDpin_1, 0);

  } else {
    //no optimal conditions
    digitalWrite(buzzerPin, LOW);

    analogWrite(LEDpin_1, 255);
  }

  if (distance2 <= 5) {
    digitalWrite(buzzerPin, HIGH);
    analogWrite(LEDpin_2, 0);


  } else {
    //no optimal conditions
    digitalWrite(buzzerPin, LOW);
    analogWrite(LEDpin_2, 255);
  }
  //ink pumping===============================================================================>
  // if ((millis() - pumpTime) >= 9000 && pumpActive == 0) {  //turning on both pumps -----> every 15 min
  //   Serial.println(pumpTime);
  //   pumpTime = activeTime_1 = activeTime_2 = millis();
  //   digitalWrite(pumpPin_1, LOW);
  //   digitalWrite(pumpPin_2, LOW);
  //   pumpActive = 1;
  // }
  // if((millis()-activeTime_1)>=2000 && pumpActive==1){//turning off pump 1----> after 2 min
  //   digitalWrite(pumpPin_1,HIGH);
  // }
  // if((millis()-activeTime_2)>=3000 && pumpActive==1){//turning off pump 2----> after 3 min (the one having more active time)
  //   digitalWrite(pumpPin_2,HIGH);
  //   pumpActive=0;
  //   pumpTime = 0;
  // }
   currentTime = millis();
  if ((currentTime - previousTime) > 60000){
    digitalWrite(pumpPin_1, LOW);
    digitalWrite(pumpPin_2, LOW);
    delay(1500);
    digitalWrite(pumpPin_1, HIGH);
    digitalWrite(pumpPin_2, HIGH);


   
     previousTime = currentTime;
    }


 
  delay(50);
}
