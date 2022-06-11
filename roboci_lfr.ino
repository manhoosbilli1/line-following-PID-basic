#include <JC_Button.h>
#include <DRV8835.h>
#include <L298.h>
#include <QTRSensors.h>


//qtr array
#define NUM_SENSORS             5  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  4  // average 4 analog samples per sensor reading
#define EMITTER_PIN             QTR_NO_EMITTER_PIN  // emitter is controlled by digital pin 2

QTRSensorsAnalog qtra((unsigned char[]) {
  A4, A3, A2, A1, A0
}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

Button ABtn(A5);
Button BBtn(2);
Button CBtn(3);

L298 driver(4, 5, 11, 7, 6, 10); //m1,m2,enA,m3,m4,enB
//L298 driver(4, 5, 10, 7, 6, 11); //m1,m2,enA,m3,m4,enB

const double KP = 0.025;             //you should edit this with your experiments....start with same values then increase or decrease as you like
const double KD = 0.15;
double lastError = 0;
const int GOAL = 2000;
const unsigned char MAX_SPEED = 50;


bool a, b, c;
void setup() {
  pinMode(13, OUTPUT);
  driver.init();
  Serial.begin(9600);
  ABtn.begin();
  BBtn.begin();
  CBtn.begin();
  // put your setup code here, to run once:
  calibrateLineSensor();
}

void loop() {
  followLine(); 
//   put your main code here, to run repeatedly:
//  bool aS = ABtn.read();
//  bool bS = BBtn.read();
//  bool cS = CBtn.read();
//
//  Serial.print(aS);
//  Serial.print(bS);
//  Serial.print(cS);
//  Serial.print("  ");
//  Serial.print(a);
//  Serial.print(b);
//  Serial.println(c);
//  if (ABtn.wasPressed()) {
//    //flash for confirmation
//    for (int i = 0; i < 5; i++) {
//      digitalWrite(13, HIGH);
//      delay(250);
//      digitalWrite(13, LOW);
//      delay(250);
//    }
//    a = true;
//  }
//  else if (BBtn.wasPressed()) {
//    for (int i = 0; i < 5; i++) {
//      digitalWrite(13, HIGH);
//      delay(250);
//      digitalWrite(13, LOW);
//      delay(250);
//    }
//    b = true;
//  }
//  else if (CBtn.wasPressed()) {
//    for (int i = 0; i < 5; i++) {
//      digitalWrite(13, HIGH);
//      delay(250);
//      digitalWrite(13, LOW);
//      delay(250);
//    }
//    c = true;
//
//  }
//
//
//  while (a == true  && b == false && c == false) {  // for starting point A
//
//  }
//
//
//  while (a == false  && b == true && c == false) { // for starting point B
//
//  }
//
//  while (a == false  && b == false && c == true) { // for starting point C
//
//  }

//unsigned int position = qtra.readLine(sensorValues);
//Serial.println(position); 
}


void detectCheckPoint(){
  if(analogRead(A0)== false && analogRead(A1) == false && analogRead(A2) == false && analogRead(A3) == false&& analogRead(A4) == false){
    Serial.println("detected");    
  }
  
}
void followLine() {
  // Get line position
  unsigned int position = qtra.readLine(sensorValues);
    Serial.println(position);

  // Compute error from line
  int error = GOAL - position;
  //
  // Compute motor adjustment
  int adjustment = KP * error + KD * (error - lastError);
  //
  // Store error for next increment
  lastError = error;
  //
  // Adjust motors
  driver.setMotorAPower(constrain(MAX_SPEED - adjustment, 0, MAX_SPEED));
  driver.setMotorBPower(constrain(MAX_SPEED + adjustment, 0, MAX_SPEED));


}
void calibrateLineSensor() {
  delay(500);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
  for (int i = 0; i < 200; i++)  // make the calibration take about 10 seconds
  {
    qtra.calibrate();       // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
  }
  digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration
}
