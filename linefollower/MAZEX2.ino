// Key fixes applied:
// 1. Removed infinite recursion from maze_solver()
// 2. Fixed donereplay() infinite recursion
// 3. Fixed variable scope in turnRight()
// 4. Cleaned up speed functions
// 5. Improved sensor reading logic

#include <EEPROM.h>

#define BTN 12
bool btn_state=1;
int press=0;
bool prev_state=1;
unsigned long tn=0;
bool go =0;
int program=0;
int algo=0;
const byte interruptPinl = 2;
const byte interruptPinr = 3;
volatile unsigned long cntl=0;
volatile unsigned long cntr=0;
unsigned long lastInterruptMicros=0;
#define ir1 A0
#define ir2 A1
#define ir3 A2
#define ir4 A3
#define ir5 A4
#define ir6 A5
#define ir7 4
#define ir8 5
#define stops 7
unsigned long tm=0;

int n=0;
int ir[8];
int irr[8];

#define ML1 6
#define ML2 9
#define MR2 10
#define MR1 11

int v=240;
int node[100];
int error = 0, prev_error = 0;
float Kp = 12, Kd = 5;
int weights[8] = { -10, -7, -5, -1, 1, 5, 7, 10};
int avg_error = 0, sensors = 0, output = 0, leftmot = 0, rightmot = 0;
unsigned long t;

void countpresses(){
  if((btn_state-prev_state)==-1){
    Serial.println("Pressed");
    digitalWrite(13,HIGH);
    delay(50);
    digitalWrite(13,LOW);
    press+=1;
  }
}

void takeinput(){
  unsigned long t=millis();
  Serial.println("Start");
  digitalWrite(13,HIGH);
  delay(50);
  digitalWrite(13,LOW);
  while(millis()-t<2000){
    btn_state=digitalRead(BTN);
    countpresses();
    prev_state=btn_state;
  }
  Serial.println(press);
  program=press;
  press=0;
  digitalWrite(13,HIGH);
  delay(50);
  digitalWrite(13,LOW);
  delay(50);
  digitalWrite(13,HIGH);
  delay(50);
  digitalWrite(13,LOW);
  t=millis();
  while(millis()-t<2000){
    btn_state=digitalRead(BTN);
    countpresses();
    prev_state=btn_state;
  }
  Serial.println(press);
  algo=press;

}

void Forward(int spd) {
  
  analogWrite(ML1, spd);
  analogWrite(ML2, 0);
  analogWrite(MR2, 0);
  analogWrite(MR1, spd);
}

void left(int lspd, int rspd) {
  analogWrite(ML1, 0);
  analogWrite(ML2, lspd);
  analogWrite(MR2, 0);
  analogWrite(MR1, rspd);
}

void right(int lspd, int rspd) {
  analogWrite(ML1, lspd);
  analogWrite(ML2, 0);
  analogWrite(MR2, rspd);
  analogWrite(MR1, 0);
}



void changel() {
  //cntl++;
  unsigned long currentMicros = micros();
  
  // Ignore interrupts faster than 100 microseconds apart (debouncing)
  if (currentMicros - lastInterruptMicros > 100) {
    cntl++;
    lastInterruptMicros = currentMicros;
  }
}

void changer() {
  //cntl++;
  unsigned long currentMicros = micros();
  
  // Ignore interrupts faster than 100 microseconds apart (debouncing)
  if (currentMicros - lastInterruptMicros > 100) {
    cntr++;
    lastInterruptMicros = currentMicros;
  }
}

void setup() {
  for(int i=0;i<100;i++)
     node[i]=0;
  pinMode(interruptPinl, INPUT_PULLUP);
  Serial.begin(115200);
  pinMode(ir1, INPUT);
  pinMode(ir2, INPUT);
  pinMode(ir3, INPUT);
  pinMode(ir4, INPUT);
  pinMode(ir5, INPUT);
  pinMode(ir6, INPUT);
  pinMode(ir7, INPUT);
  pinMode(ir8, INPUT);
  pinMode(stops, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPinl), changel, RISING);
  attachInterrupt(digitalPinToInterrupt(interruptPinr), changer, RISING);
  pinMode(ML1, OUTPUT);
  pinMode(ML2, OUTPUT);
  pinMode(MR1, OUTPUT);
  pinMode(MR2, OUTPUT);
  pinMode(12, INPUT_PULLUP);
  pinMode(13, OUTPUT);

  analogWrite(ML1, 0);
  analogWrite(ML2, 0);
  analogWrite(MR1, 0);
  analogWrite(MR2, 0);
  delay(500);
  digitalWrite(13,LOW);
  takeinput();
}

void loop() {
  if(program==0){
    readSensors();  
    
    if ((ir[0] && ir[1] && ir[2]) || (ir[5] && ir[7]) || 
        (ir[0] && ir[1] && ir[2] && ir[3] && ir[4] && ir[7]) ||
        (!ir[0] && !ir[1] && !ir[2] && !ir[3] && !ir[4] && !ir[5] && !ir[6] && !ir[7]))
    {
      for(int i=0; i<8 ; i++)
        irr[i] = 0;
      t = millis();
      while (millis() - t < 100) {
        readSensors();
        for (int i = 0; i < 8; i++)
        {
          if (irr[i] == 0)
            irr[i] = ir[i]; 
        }
        normStraight();
      }

      LSR();
    }
    else {
      straight();
    }
  }

  else if(program==2){
    EEPROM.get(0, n); 
    EEPROM.get(sizeof(int), node);    
    //maze_solver();
    digitalWrite(13, HIGH);
    noInterrupts();
    cntl=0;
    cntr=0;
    interrupts();
    
    int speedlef=90;
    int speedrig=90;
    
    bool leftDone = false;
    bool rightDone = false;
    bool earlyExit = false;
    
    while(!leftDone || !rightDone){
      noInterrupts();
      
      // Check for early exit condition
      if(cntr>320 && digitalRead(ir3)==LOW){
        earlyExit = true;
      }
      
      unsigned long currentLeft = cntl;
      unsigned long currentRight = cntr;
      
      interrupts();  // Always re-enable interrupts!
      
      if(earlyExit){
        break;  // Now it's safe to break
      }
      
      leftDone = (currentLeft >= 625);
      rightDone = (currentRight >= 625);
      
      speedlef = leftDone ? 0 : 90;
      speedrig = rightDone ? 0 : 90;
      
      analogWrite(ML1, speedlef);
      analogWrite(ML2, 0);
      analogWrite(MR2, speedrig);
      analogWrite(MR1, 0);
    }
    digitalWrite(13,LOW);
    Forward(0);
    delay(500);
    return_trip();
  }
}

void readSensors() {

  ir[0] = digitalRead(ir1);
  ir[1] = digitalRead(ir2);
  ir[2] = digitalRead(ir3);
  ir[3] = digitalRead(ir4);
  ir[4] = digitalRead(ir5);
  ir[5] = digitalRead(ir6);
  ir[6] = digitalRead(ir7);
  ir[7] = digitalRead(ir8);
}

int check(){

  if(node[n-1] == 2 && node[n-2] == 3 && node[n-3] == 0) {
    n=n-3;
    node[n] = 3;

    n++;
    return 1;
  }
  else if(node[n-1] == 1 && node[n-2] == 3 && node[n-3] == 0) {
    n=n-3;
    node[n] = 2;

    n++;
    return 1;
  }
  else if(node[n-1] == 0 && node[n-2] == 3 && node[n-3] == 2) {
    n=n-3;
    node[n] = 3;

    n++;
    return 1;
  }
  else if(node[n-1] == 0 && node[n-2] == 3 && node[n-3] == 1) {
    n=n-3;
    node[n] = 2;

    n++;
    return 1;
  }
  else if(node[n-1] == 1 && node[n-2] == 3 && node[n-3] == 1) {
    n=n-3;
    node[n] = 3;

    n++;
    return 1;
  }
  else if(node[n-1] == 0 && node[n-2] == 3 && node[n-3] == 0) {
    n=n-3;
    node[n] = 1;

    n++;
    return 1;
  }
  else {
    return 0;
  }
}

int z=0;
// FIXED: Changed from recursive to loop-based
void maze_solver(){

  while(true) {  // Use loop instead of recursion
    v=240;
    readSensors();  
    
    if ((ir[0] && ir[1] && ir[2]) || (ir[5] && ir[7]) || 
        (ir[0] && ir[1] && ir[2] && ir[3] && ir[4] && ir[7]) ||
        (!ir[0] && !ir[1] && !ir[2] && !ir[3] && !ir[4] && !ir[5] && !ir[6] && !ir[7]))
    {
      for(int i=0; i<8 ; i++)
         irr[i] = 0;
      t = millis();
      while (millis() - t < 150) {
        readSensors();
        for (int i = 0; i < 8; i++)
        {
          if (irr[i] == 0)
            irr[i] = ir[i];
        }
        normStraight();
      }
      /*if(millis()-tn<1400){
        go=1;
        //digitalWrite(12,HIGH);
      }*/
      // Check for completion (all sensors off line)
      if(digitalRead(ir8)==LOW && digitalRead(ir7)==LOW && digitalRead(ir6)==LOW){
        donereplay();
      }
      if(digitalRead(ir8)==LOW && digitalRead(ir7)==LOW && digitalRead(ir6)==LOW && 
         digitalRead(ir4)==LOW && digitalRead(ir3)==LOW && digitalRead(ir1)==LOW){
        donereplay();
      }
      //digitalWrite(12,LOW);
      if(node[z]==0){
          t = millis();
          while (millis() - t < 12){
            speed_reduced();
          }
          leftenc(210,210);
      }
      else if(node[z]==2){
          t=millis();
          while(millis() - t < 8){
            speed_reduced();
          }
          rightenc(210,210);
      }
      else{
          straight();
      }
      z++;
//      int t=millis();
//      while(t-millis() <= 50)
//        normStraight();
    }
    else{
      straight();
    }
  }
}

void return_trip(){
  int r=n-1;
  while(true) {  // Use loop instead of recursion
    v=240;
    readSensors();  
    
    if ((ir[0] && ir[1] && ir[2]) || (ir[5] && ir[7]) || 
        (ir[0] && ir[1] && ir[2] && ir[3] && ir[4] && ir[7]) ||
        (!ir[0] && !ir[1] && !ir[2] && !ir[3] && !ir[4] && !ir[5] && !ir[6] && !ir[7]))
    {
      for(int i=0; i<8 ; i++)
         irr[i] = 0;
      t = millis();
      while (millis() - t < 150) {
        readSensors();
        for (int i = 0; i < 8; i++)
        {
          if (irr[i] == 0)
            irr[i] = ir[i];
        }
        normStraight();
      }
      if(millis()-tn<1400){
        go=1;
        //digitalWrite(12,HIGH);
      }
      // Check for completion (all sensors off line)
      if(digitalRead(ir8)==LOW && digitalRead(ir7)==LOW && digitalRead(ir6)==LOW){
        donereplay();
      }
      if(digitalRead(ir8)==LOW && digitalRead(ir7)==LOW && digitalRead(ir6)==LOW && 
         digitalRead(ir4)==LOW && digitalRead(ir3)==LOW && digitalRead(ir1)==LOW){
        donereplay();
      }
      //digitalWrite(12,LOW);
      if(node[r]==2){
          t = millis();
          while (millis() - t < 12){
            speed_reduced();
          }
          leftenc(210,210);
      }
      else if(node[r]==0){
          t=millis();
          while(millis() - t < 8){
            speed_reduced();
          }
          rightenc(220,220);
      }
      else{
        t=millis();
        while((millis-t)<2000){
          straight();
        }
      }
      r--;
//      int t=millis();
//      while(t-millis() <= 50)
//        normStraight();
    }
    else{
      straight();
    }
  }  
}

// FIXED: Removed infinite recursion
void donereplay(){

  analogWrite(ML1, 0);  
  analogWrite(ML2, 0);
  analogWrite(MR1, 0);
  analogWrite(MR2, 0);
  //digitalWrite(13, HIGH);
  
  while(true) {
    delay(1000);  // Stay here forever
  }
}

void LSR() {
  if((tm=(millis()-tn))<1000){
    go=1;
    //digitalWrite(12,HIGH);
  }
  tn=millis();
  Serial.println("LSR");
  if (irr[7] && irr[6] && irr[5])
  { 
    t = millis();
    while (millis() - t < 5){
      speed_reduced();
    }

    // Check if maze is complete
    if (digitalRead(ir4)==HIGH && digitalRead(ir5)==HIGH && 
        digitalRead(ir6)==HIGH && digitalRead(ir7)==HIGH){
      t=millis();
      while(millis()-t<15){
        normStraight();
      }
      if(digitalRead(ir4)==HIGH && digitalRead(ir5)==HIGH && digitalRead(ir6)==HIGH && digitalRead(ir7)==HIGH){
        EEPROM.put(0, n);
        EEPROM.put(sizeof(int), node);
        done();
        //maze_solver();
        digitalWrite(13, HIGH);
        noInterrupts();
        cntl=0;
        cntr=0;
        interrupts();
        
        int speedlef=90;
        int speedrig=90;
        
        bool leftDone = false;
        bool rightDone = false;
        bool earlyExit = false;
        
        while(!leftDone || !rightDone){
          noInterrupts();
          
          // Check for early exit condition
          if(cntr>320 && digitalRead(ir3)==HIGH){
            earlyExit = true;
          }
          
          unsigned long currentLeft = cntl;
          unsigned long currentRight = cntr;
          
          interrupts();  // Always re-enable interrupts!
          
          if(earlyExit){
            break;  // Now it's safe to break
          }
          
          leftDone = (currentLeft >= 625);
          rightDone = (currentRight >= 625);
          
          speedlef = leftDone ? 0 : 90;
          speedrig = rightDone ? 0 : 90;
          
          analogWrite(ML1, speedlef);
          analogWrite(ML2, 0);
          analogWrite(MR2, speedrig);
          analogWrite(MR1, 0);
        }
        digitalWrite(13,LOW);
        Forward(0);
        delay(500);
        return_trip();
      }
    }
    
    node[n]=0;
    n++;
    if(n >= 3){
      check();
    }
  
    //cleft();
    //delcruise(100, 40);
    //pdleft(650);
    //digitalWrite(12,LOW);
    leftenc(330,330);
    //turnLeft();
  }
  else if (irr[0] || irr[1]) {    
    if (digitalRead(ir1)==HIGH || digitalRead(ir2)==HIGH || digitalRead(ir3)==HIGH ||
        digitalRead(ir4)==HIGH || digitalRead(ir5)==HIGH || digitalRead(ir6)==HIGH || 
        digitalRead(ir7)==HIGH || digitalRead(ir8)==HIGH) {

      t = millis();
      while(millis() - t < 5)
        speed_reduced();


      if (digitalRead(ir2)==LOW && digitalRead(ir3)==LOW && 
          digitalRead(ir4)==LOW && digitalRead(ir6)==LOW && digitalRead(ir7)==LOW)
      {
        node[n]=2;
        n++;
        if (n >= 3)
          check();
        //digitalWrite(13,HIGH);
 
        //turnRight();
        //cright();
        //delcruise(220,100);
        //pdleft(650);
        rightenc(275,275); //no straight path turn right
        //turnRight();
      }
      else{ 
        node[n]=1;
        digitalWrite(13, HIGH);
        n++;
        if(n >= 3) //straight available skip right
          check();

        v=240;
      }
    }
    else {
      t=millis();
      while (millis() - t < 3)
        speed_reduced();
      node[n] = 2;
      v=240;
      n++;
      if(n >= 3)
        check();
   
      //turnRight();
      //cright();
      //delcruise(220,75);
      rightenc(275,275);
    }
  }
  else if (!irr[0] && !irr[1] && !irr[2] && !irr[3] && !irr[4] && !irr[5] && !irr[6] && !irr[7]) {
    node[n]=3;
    n++;
    if(n >= 3)
      check();
    t=millis();
    while (millis() - t < 3){
      speed_reduced(); 
    }
    v=240;

    //turnAround();
    //creverse();
    backenc(625,625);
  }
  v=240;
}

void done() {



  analogWrite(ML1, 0);
  analogWrite(ML2, 0);
  analogWrite(MR1, 0);
  analogWrite(MR2, 0);
  //digitalWrite(12, HIGH);
  delay(2000);
}

void straight() {

  readSensors();
  prev_error = avg_error;
  for (int i = 0; i < 8; i++)
  {
    if (ir[i])
      sensors++;
    error += ir[i] * weights[7 - i];
  }
  if (sensors!=0){
    avg_error = error / sensors;
  }
  else{
    avg_error=error;
  }
  sensors = 0; 
  error = 0;
  output = Kp * avg_error + Kd * (avg_error - prev_error);

  leftmot = v + output;
  rightmot = v - output;
  leftmot = constrain(leftmot, 0, 255);
  rightmot = constrain(rightmot, 0, 255);
  analogWrite(ML1, leftmot);
  analogWrite(ML2, LOW);
  analogWrite(MR1, rightmot);
  analogWrite(MR2, LOW);
  output = 0;
}

// FIXED: Improved speed functions
void speed_reduced(){

  int tempV = v;
  while(tempV > 0){
    tempV -= 50;
    if(tempV < 0) tempV = 0;
    v = tempV;
    straight();
    delay(0.8);
  }
}

/*void speed_increased(){

  int tempV = v;
  while(tempV < 170){
    tempV += 55;
    if(tempV > 170){
      tempV = 170;
    }
    v = tempV;
    straight();
    delay(2);
  }
}*/

void speed_increased(){
  Serial.println("speed_increased called");
  int tempV = v;
  while(tempV < 170){
    Serial.print("Inner loop, tempV=");
    Serial.println(tempV);
    tempV += 55;
    if(tempV > 170){
      tempV = 170;
    }
    v = tempV;
    straight();
    delay(2);
  }
  Serial.println("speed_increased exiting");
}

void normStraight(){

  analogWrite(ML1, 100);
  analogWrite(ML2, 0);
  analogWrite(MR1, 100);
  analogWrite(MR2, 0);
}

/*void leftenc(int lef, int rig){
  // Disable interrupts while resetting counters
  noInterrupts();
  cntl=0;
  cntr=0;
  interrupts();
  
  int speedlef=200;
  int speedrig=200;
  
  bool leftDone = false;
  bool rightDone = false;
  
  while(!leftDone || !rightDone){
    // Read counters atomically (disable interrupts during read)
    noInterrupts();
    if(cntl>400){
      if(digitalRead(ir4)==LOW){
        break;
      }
    }
    unsigned long currentLeft = cntl;
    unsigned long currentRight = cntr;
    interrupts();
    
    // Check completion
    leftDone = (currentLeft >= lef);
    rightDone = (currentRight >= rig);
    
    // Calculate speeds
    speedlef = leftDone ? 0 : 200;
    speedrig = rightDone ? 0 : 200;
    
    // Apply motor speeds
    analogWrite(ML2, speedlef);
    analogWrite(ML1, 0);
    analogWrite(MR1, speedrig);
    analogWrite(MR2, 0);
  }
  noInterrupts();
  Forward(0);
  Serial.println("Turn complete, starting alignment");
  
  unsigned long alignStart = millis();  // LOCAL variable
  while(millis() - alignStart < 10){
    Serial.println("Outer loop running");
    v=0;
    speed_increased();
  }
  
  Serial.println("Alignment complete, exiting leftenc");
}*/

void leftenc(int lef, int rig){
  digitalWrite(13, HIGH);
  analogWrite(ML1, 0);
  analogWrite(ML2, 100);
  analogWrite(MR1, 0);
  analogWrite(MR2, 100);
  delay(60);
  analogWrite(ML2, 0);
  analogWrite(ML1, 0);
  analogWrite(MR1, 0);
  analogWrite(MR2, 0);
  /*if(go==1){
    analogWrite(ML2, 0);
    analogWrite(ML1, 100);
    analogWrite(MR2, 0);
    analogWrite(MR1, 100);
    delay(30);
    analogWrite(ML2, 0);
    analogWrite(ML1, 0);
    analogWrite(MR1, 0);
    analogWrite(MR2, 0);
    go=0;
  }*/
  //noInterrupts();
  cntl=0;
  cntr=0;
  //interrupts();
  
  int speedlef=150;
  int speedrig=150;
  
  bool leftDone = false;
  bool rightDone = false;
  bool earlyExit = false;
  while(!leftDone || !rightDone){
    //noInterrupts();
    
    // Check for early exit condition
    if(cntr>240 && digitalRead(ir6)==HIGH){
      earlyExit = true;
    }
    
    unsigned long currentLeft = cntl;
    unsigned long currentRight = cntr;
    
    //interrupts();  // Always re-enable interrupts!
    

    
    leftDone = (currentLeft >= lef);
    rightDone = (currentRight >= rig);
    
    speedlef = leftDone ? 0 : 150;
    speedrig = rightDone ? 0 : 150;
    
    analogWrite(ML2, speedlef);
    analogWrite(ML1, 0);
    analogWrite(MR1, speedrig);
    analogWrite(MR2, 0);
  }
  
  Forward(0);
  Serial.println("Turn complete, starting alignment");
  
  unsigned long alignStart = millis();
  v = 0;
  int a=0;
  if (go){
    a=140;
  }
  else{
    a=300;
  }
  while(millis() - alignStart < 300){
    if(v < 110){
      v += 10;
    }
    straight();
    delay(2);
  }
  
  Serial.println("Alignment complete, exiting leftenc");
  v = 240;
  digitalWrite(13, LOW);
}

void rightenc(int lef, int rig){
  analogWrite(ML2, 100);
  analogWrite(ML1, 0);
  analogWrite(MR2, 100);
  analogWrite(MR1, 0);
  delay(80);
  analogWrite(ML2, 0);
  analogWrite(ML1, 0);
  analogWrite(MR1, 0);
  analogWrite(MR2, 0);
  if(go==1){
    analogWrite(ML2, 0);
    analogWrite(ML1, 80);
    analogWrite(MR2, 0);
    analogWrite(MR1, 80);
    delay(90);
    analogWrite(ML2, 0);
    analogWrite(ML1, 0);
    analogWrite(MR1, 0);
    analogWrite(MR2, 0);
  go=0;
  }
  digitalWrite(13, HIGH);
  //noInterrupts();
  cntl=0;
  cntr=0;
  //interrupts();
  
  int speedlef=150;
  int speedrig=150;
  
  bool leftDone = false;
  bool rightDone = false;
  bool earlyExit = false;

  while(!leftDone || !rightDone){
    //noInterrupts();
    
    // Check for early exit condition
    if(cntr>130 && digitalRead(ir3)==HIGH){
      earlyExit = true;
    }
    
    unsigned long currentLeft = cntl;
    unsigned long currentRight = cntr;
    
    //interrupts();  // Always re-enable interrupts
    
    
    
    leftDone = (currentLeft >= lef);
    rightDone = (currentRight >= rig);
    
    speedlef = leftDone ? 0 : 150;
    speedrig = rightDone ? 0 : 150;
    
    analogWrite(ML1, speedlef);
    analogWrite(ML2, 0);
    analogWrite(MR2, speedrig);
    analogWrite(MR1, 0);
  }
  
  Forward(0);
  delay(20);
  Serial.println("Turn complete, starting alignment");
  
  unsigned long alignStart = millis();
  v = 0;
  int a=0;
  while(millis() - alignStart < 300){
    if(v < 120){
      v += 20;
    }
    straight();
    delay(2);
  }
  
  Serial.println("Alignment complete, exiting leftenc");
  v = 240;
  digitalWrite(13, LOW);
}

void backenc(int lef, int rig){
  if(go==1){
    analogWrite(ML2, 0);
    analogWrite(ML1, 100);
    analogWrite(MR2, 0);
    analogWrite(MR1, 100);
    delay(85);
    analogWrite(ML2, 0);
    analogWrite(ML1, 0);
    analogWrite(MR1, 0);
    analogWrite(MR2, 0);
    go=0;
  }
  digitalWrite(13, HIGH);
  noInterrupts();
  cntl=0;
  cntr=0;
  interrupts();
  
  int speedlef=90;
  int speedrig=90;
  
  bool leftDone = false;
  bool rightDone = false;
  bool earlyExit = false;
  
  while(!leftDone || !rightDone){
    noInterrupts();
    
    // Check for early exit condition
    if(cntr>320 && digitalRead(ir3)==HIGH){
      earlyExit = true;
    }
    
    unsigned long currentLeft = cntl;
    unsigned long currentRight = cntr;
    
    interrupts();  // Always re-enable interrupts!
    
    if(earlyExit){
      break;  // Now it's safe to break
    }
    
    leftDone = (currentLeft >= lef);
    rightDone = (currentRight >= rig);
    
    speedlef = leftDone ? 0 : 90;
    speedrig = rightDone ? 0 : 90;
    
    analogWrite(ML1, speedlef);
    analogWrite(ML2, 0);
    analogWrite(MR2, speedrig);
    analogWrite(MR1, 0);
  }
  
  Forward(0);
  delay(10);
  Serial.println("Turn complete, starting alignment");
  
  unsigned long alignStart = millis();
  v = 0;
  int a=0;
  if (go){
    a=140;
  }
  else{
    a=300;
  }
  while(millis() - alignStart < 300){
    if(v < 120){
      v += 20;
    }
    straight();
    delay(2);
  }
  
  Serial.println("Alignment complete, exiting leftenc");
  v = 240;
  digitalWrite(13, LOW);
}

