#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <EEPROM.h>

// ==== أرجل الدوائر ====
#define greenLed   A1
#define yellowLed  A2
#define redLed     A3
#define startBtn   3
#define stopBtn    2
#define ir1Pin     5
#define ir2Pin     6
#define ir3Pin     7
#define ir4Pin     8
#define motorPin   9
#define P1         A0
#define ON         10
#define Save       11
#define Run        12
#define SEL        13 

#define NUM_SERVOS 6
#define MAX_STEPS  20

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// ==== متغيرات السيرفو والبرامج ====
int servoPos[NUM_SERVOS] = {90,90,90,90,90,90};
int largeProgram[MAX_STEPS][NUM_SERVOS];
int smallProgram[MAX_STEPS][NUM_SERVOS];
int numLargeSteps = 0;
int numSmallSteps = 0;

bool systemRunning = false;
bool autoMode = false;
bool returnedToCenter = false;
bool lastSaveState = LOW;
int currentServo = 0;

// ==== إعدادات السيرفو PCA9685 ====
#define SERVOMIN 150
#define SERVOMAX 600

int angleToPulse(int angle){ return map(angle,0,180,SERVOMIN,SERVOMAX); }

void setServo(int idx, int angle){
  servoPos[idx]=angle;
  pwm.setPWM(idx,0,angleToPulse(angle));
}

void setup(){
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(50);

  // تهيئة الأرجل
  pinMode(ir1Pin, INPUT); pinMode(ir2Pin, INPUT);
  pinMode(ir3Pin, INPUT); pinMode(ir4Pin, INPUT);
  pinMode(startBtn, INPUT_PULLUP); pinMode(stopBtn, INPUT_PULLUP);
  pinMode(ON, INPUT); pinMode(Save, INPUT); pinMode(Run, INPUT); pinMode(SEL, INPUT_PULLUP);

  pinMode(greenLed, OUTPUT); pinMode(yellowLed, OUTPUT); pinMode(redLed, OUTPUT); pinMode(motorPin, OUTPUT);

  // الوضع الطبيعي: الأحمر فقط
  digitalWrite(redLed,HIGH);
  digitalWrite(greenLed,HIGH);
  digitalWrite(yellowLed,LOW);
  digitalWrite(motorPin,LOW);

  // تحميل البرامج من EEPROM
  EEPROM.get(0, numLargeSteps);
  for(int i=0;i<numLargeSteps;i++) EEPROM.get(4+i*NUM_SERVOS*sizeof(int), largeProgram[i]);
  EEPROM.get(512, numSmallSteps);
  for(int i=0;i<numSmallSteps;i++) EEPROM.get(516+i*NUM_SERVOS*sizeof(int), smallProgram[i]);

  Serial.println("System Ready!");
}

// ==== عدادات الخط ====
int lastIr1 = LOW;
int lastIr2 = LOW;
int objectsOnLine = 0;
int small_in=0, large_in=0, small_out=0, large_out=0;

void loop(){
  int ir1=digitalRead(ir1Pin);
  int ir2=digitalRead(ir2Pin);
  int ir3=digitalRead(ir3Pin);
  int ir4=digitalRead(ir4Pin);

  // ==== START / STOP ====
if (digitalRead(startBtn) == LOW) {
    systemRunning = true;
    Serial.println("System STARTED");
    delay(300);
  }

if (digitalRead(stopBtn) == HIGH) {
    systemRunning = false;
    Serial.println("System STOPPED");
    digitalWrite(redLed, HIGH);
    digitalWrite(greenLed, HIGH);
    digitalWrite(yellowLed, LOW);
    digitalWrite(motorPin, LOW);
    objectsOnLine = 0;
    small_in = large_in = small_out = large_out = 0;
    returnedToCenter = false;
    delay(300);
  }

  if(!systemRunning) return;

  // ==== عداد الأجسام ====
  if(ir1==HIGH && lastIr1==LOW){
    objectsOnLine++;
    if(ir3==HIGH){ large_in++; Serial.print("Large ENTERED "); }
    else{ small_in++; Serial.print("Small ENTERED "); }
    
    Serial.println(objectsOnLine);
  }
  int count_in = large_in + small_in;
  if(ir2==HIGH && lastIr2==LOW && objectsOnLine>0){
    objectsOnLine--;
    if(ir4==HIGH){ large_out++; Serial.print("Large EXITED "); }
    else{ small_out++; Serial.print("Small EXITED "); }
    
    Serial.println(objectsOnLine);
  }
  lastIr1=ir1; lastIr2=ir2;
  int count_out = large_out + small_out;

  // ==== LEDs + MOTOR (حسب المنطق المطلوب) ====
  // الأولوية: STOP (خارج), START (تم فوق), ثم IR1, ثم IR2, وإلا يبقى على آخر حالة START (أصفر)
  if (objectsOnLine > 0) {
    if (ir2 == LOW){
    digitalWrite(redLed, LOW);
    digitalWrite(greenLed, HIGH);
    digitalWrite(yellowLed, HIGH);
    digitalWrite(motorPin, HIGH);
    }else{
    digitalWrite(redLed, HIGH);
    digitalWrite(greenLed, LOW);
    digitalWrite(yellowLed, HIGH);
    digitalWrite(motorPin, LOW);
  }
  }else{
    digitalWrite(redLed, LOW);
    digitalWrite(greenLed, HIGH);
    digitalWrite(yellowLed, HIGH);
    digitalWrite(motorPin, HIGH);
  }
  // لو مفيش IRs فعالين والنظام شغال، هنسيب الحالة أصفر (من لحظة START)

  // طباعة حالة العدادات
  Serial.print("IN—S:"); Serial.print(small_in); Serial.print(",L:"); Serial.print(large_in);
  Serial.print(" OUT—S:"); Serial.print(small_out); Serial.print(",L:"); Serial.println(large_out);

  // ==== MANUAL / AUTO ====
  autoMode = (digitalRead(ON)==LOW);

  if(!autoMode){ // MANUAL MODE
    returnedToCenter=false;
    int selectState=digitalRead(SEL);
    static int lastSelectState=HIGH;
    if(selectState==LOW && lastSelectState==HIGH){
      currentServo++; if(currentServo>=NUM_SERVOS) currentServo=0; delay(200);
      Serial.print("Selected Servo "); Serial.println(currentServo+1);
    }
    lastSelectState=selectState;

    int pos=map(analogRead(P1),0,1023,0,180);
    setServo(currentServo,pos);
    Serial.print("Servo "); Serial.print(currentServo+1); Serial.print(" Pos: "); Serial.println(pos);

    // Save
    int saveState=digitalRead(Save);
    if(saveState==HIGH && lastSaveState==LOW){
      char type; Serial.println("Enter type (L/S):"); while(!Serial.available()); type=Serial.read();
      if(type=='L'){
        if(numLargeSteps<MAX_STEPS){
          for(int i=0;i<NUM_SERVOS;i++){
            if(i==currentServo) largeProgram[numLargeSteps][i]=pos;
            else if(numLargeSteps>0) largeProgram[numLargeSteps][i]=largeProgram[numLargeSteps-1][i];
            else largeProgram[numLargeSteps][i]=90;
          }
          numLargeSteps++;
          EEPROM.put(0,numLargeSteps);
          for(int i=0;i<numLargeSteps;i++) EEPROM.put(4+i*NUM_SERVOS*sizeof(int),largeProgram[i]);
        }
      } else {
        if(numSmallSteps<MAX_STEPS){
          for(int i=0;i<NUM_SERVOS;i++){
            if(i==currentServo) smallProgram[numSmallSteps][i]=pos;
            else if(numSmallSteps>0) smallProgram[numSmallSteps][i]=smallProgram[numSmallSteps-1][i];
            else smallProgram[numSmallSteps][i]=90;
          }
          numSmallSteps++;
          EEPROM.put(512,numSmallSteps);
          for(int i=0;i<numSmallSteps;i++) EEPROM.put(516+i*NUM_SERVOS*sizeof(int),smallProgram[i]);
        }
      }
      Serial.println("Position Saved!");
      delay(150);
    }
    lastSaveState=saveState;
  }
  else{ // AUTO MODE
    if(!returnedToCenter){ for(int i=0;i<NUM_SERVOS;i++) setServo(i,90); returnedToCenter=true; Serial.println("Centered for Auto Mode"); }

    bool objectDetected=(ir2==LOW && count_in > count_out );
    bool isLarge=(ir4==HIGH);

    if(objectDetected){
      int steps=isLarge?numLargeSteps:numSmallSteps;
      int (*program)[NUM_SERVOS]=isLarge?largeProgram:smallProgram;
      Serial.print("Auto Running "); Serial.println(isLarge?"Large":"Small");

      for(int step=0;step<steps;step++){
        bool moving=true;
        while(moving){
          moving=false;
          for(int i=0;i<NUM_SERVOS;i++){
            if(servoPos[i]<program[step][i]){ setServo(i,servoPos[i]+1); moving=true; }
            if(servoPos[i]>program[step][i]){ setServo(i,servoPos[i]-1); moving=true; }
           
          }
          delay(10);
        }
      }
      Serial.println("Auto Program Finished");
    }
    else if(digitalRead(Run)==HIGH){ Serial.println("Manual Run Pressed"); /* ممكن تشغل برنامج محدد */ }
  }

  delay(50);
}
