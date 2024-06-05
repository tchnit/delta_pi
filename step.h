#include <ShiftRegister74HC595.h>
// parameters: <number of shift registers> (data pin, clock pin, latch pin)
ShiftRegister74HC595<3> sr(27, 25, 26);

const int endStop[] = {22,32,33};


// const int stepPins[] = {1, 4, 7};  // Chân Step cho 3 động cơ
const int stepPins[] = {7, 10, 13};  // Chân Step cho 3 động cơ
const int dirPins[] = {8, 11, 14};   // Chân Direction cho 3 động cơ
const int enPins[] = {6,9,12};

// const int stepsPerRevolution = 16000; // Số bước trên mỗi vòng quay của động cơ
bool cont[]={1,1,1};
int direction[]={0,0,0};

byte angle=90;
int speed=500;
int bc=0;
int x;
int y;
int z;

int x_new;
int y_new;
int z_new;

int x_old;
int y_old;
int z_old;

int a;
void command(String received);

void moveStepper(int steps[3]) {
  int maxAbsoluteValue = abs(steps[0]); // Lấy giá trị tuyệt đối của phần tử đầu tiên
//Chiều động cơ
  for (int i = 0; i < 3; i++) {
    if (steps[i]>0){
      direction[i]=LOW;
    }
    else{
      direction[i]=HIGH;
    }

    if (i==2){
      if (steps[i]<0){
        direction[i]=LOW;
      }
      else{
        direction[i]=HIGH;
      }}

    sr.set(dirPins[i], direction[i]);


    int absoluteValue = abs(steps[i]); // Lấy giá trị tuyệt đối của phần tử thứ i
    if (absoluteValue > maxAbsoluteValue) {
      maxAbsoluteValue = absoluteValue; // Cập nhật giá trị lớn nhất nếu tìm thấy giá trị tuyệt đối lớn hơn
    }
  }
  for (int i = 0; i < maxAbsoluteValue; i++) {
    
    for (int st=0;st<3;st++){
      if (i<abs(steps[st])){
        sr.set(stepPins[st], HIGH);
      }
    }
    delayMicroseconds(1000-speed); 
    // delay(5);
    for (int st=0;st<3;st++){
      sr.set(stepPins[st], LOW);
    }
    delayMicroseconds(1000-speed);
    // delay();
  }
}


void gotohome(){

  for (int i = 0; i < 3; i++) {
    sr.set(dirPins[i], HIGH);
    if (i==2)
        sr.set(dirPins[i], LOW);

    cont[i]=1;
  }
int ac=5;
  while ((cont[0]+cont[1]+cont[2])>0) {
    for (int i=0;i<3;i++){
      if (digitalRead(endStop[i])==0){
        cont[i]=0;
      }
      if (cont[i]==1){
        sr.set(stepPins[i], HIGH);
      }
    }
    delayMicroseconds(1000-speed); 
    for (int i=0;i<3;i++){
      sr.set(stepPins[i], LOW);
    }
    delayMicroseconds(1000-speed);
    // delay();
  }
   x_new=0;
   y_new=0;
   z_new=0;
   x_old=0;
   y_old=0;
   z_old=0;
  x=0;
  y=0;
  z=0;
  float thetas[]={0, 0, 0};
  int steps[]={0, 0, 0};
  delta_calcInverse(0,0,0-221.076,thetas);

  theta_step(thetas,steps);
  for(int i = 0 ; i > 3; i++){
    thetas[i] = 0;
    steps[i] = 0; 
  }
  theta_step(thetas,steps);
  command("G0 X0Y0Z-13.576");
    command("G0 X0Y0Z0");

// command("home");
}

void motoron(){
  for (int i = 0; i < 3; i++) {
      sr.set(enPins[i], LOW);
  }
}

void motoroff(){
   for (int i = 0; i < 3; i++) {
      sr.set(enPins[i], HIGH);
  }
}

void bomon(){
  sr.set(17,HIGH);
}
void bomoff(){
  sr.set(17,LOW);
}

void hut(){
  sr.set(18,LOW);
}
void tha(){
  sr.set(18,HIGH);
}

void convrun(){
  // sr.set(3,LOW);
    sr.set(16,HIGH);

}
void convstop(){
  // sr.set(3,HIGH);
  sr.set(16,LOW);

}
// void rotation(byte angle){
//   // servo1.write(angle);
// }
void pointt(String received){
    x=(received.substring(received.indexOf("x")+1, received.indexOf("y"))).toInt();
    y=(received.substring(received.indexOf("y")+1, received.indexOf("z"))).toInt();
    z=(received.substring(received.indexOf("z")+1, received.indexOf("a"))).toInt(); 
    a=(received.substring(received.indexOf("a")+1, received.length())).toInt();  
   
    // Serial.println(x);
    // Serial.println(y);
    // Serial.println(z-221.076);
    delta_calcInverse(x,y,z-221.076,thetas);
      
    // Serial.println(thetas[0]); 
    // Serial.println(thetas[1]);
    // Serial.println(thetas[2]);

    theta_step(thetas,steps);
    // Serial.println(steps[0]);
    // Serial.println(steps[1]);
    // Serial.println(steps[2]);
    // rotation(a*(0.7623));
    moveStepper(steps);

  }
void convgo(int bc){
  for (int a = 0; a < bc; a++) {
    if (bc<0){
      sr.set(5, HIGH);
    }
    sr.set(4, HIGH);
    delay(1);    
    sr.set(4, LOW);
    delay(1);  
  
}}

struct FunctionMapping {
  const char* name;
  void (*function)();
};
// Tạo bảng ánh xạ tên hàm tới con trỏ hàm
FunctionMapping functionMap[] = {
  {"bom_on", bomon},
  {"bom_off", bomoff},
  {"home", gotohome},
  {"motor_on",motoron},
  {"motor_off",motoroff},
  {"Hut",hut},
  {"Tha",tha},
  {"convrun",convrun},
  {"convstop",convstop},
  
};

// Số lượng hàm trong bảng ánh xạ
const int functionMapSize = sizeof(functionMap) / sizeof(FunctionMapping);

void command2(String command) {
  for (int i = 0; i < functionMapSize; ++i) {
    if (command == functionMap[i].name) {
      functionMap[i].function();
      return;
    }
  }
  Serial.println("Function not found.");
}

void command(String received){
  if (received.equals("home")) {
    gotohome();
    // gotohome();
    
  }
  else if (received.equals("hello_robot")) {
    Serial.println("robotday");
    }
  else if (received.equals("motor_on")) {
    motoron();
    }
  else if (received.equals("motor_off")) {
   motoroff();
   }
  else if (received.indexOf("speed") != -1){
    speed=(received.substring(received.indexOf("d")+1, received.length())).toInt();
  }
  else if (received.equals("bom_on")) {
   bomon();
   }
  else if (received.equals("bom_off")) {
   bomoff();
  }
  else if (received.equals("Hut")) {
   hut();
  }
  else if (received.equals("Tha")) {
   tha();
  }
  else if (received.equals("convrun")){
    convrun();
  }

  else if (received.equals("convstop")){
    convstop();
  }
  // else if (received.equals("BTS_ON")){
  //   BTS_ON();
  // }
  // else if (received.equals("BTS_OFF")){
  //   BTS_OFF();
  // }
  // else if (received.equals("TCP_ON")){
  //   TCP_ON();
  // }
  // else if (received.equals("TCP_OFF")){
  //   TCP_OFF();
  // }
  else if (received.indexOf("angle") != -1){
    

    
    angle=(received.substring(received.indexOf("e")+1, received.length())).toInt();
    // Serial.println(angle);
    // rotation(angle*(0.7623));
  }
  else if (received.indexOf("convgot") != -1){
    
    bc=(received.substring(received.indexOf("t")+1, received.length())).toInt();
    Serial.println(bc);
    convgo(bc);
  }
  else if (received.indexOf("round") != -1){
    int round=(received.substring(received.indexOf("d")+1, received.length())).toInt();  

    for (int angle = 0; angle <= 360; angle++) {
    float radians = angle * pi / 180.0;
    int x_new = 0 + int(round * cos(radians));
    int y_new = 0 + int(round * sin(radians));
    String c_round="x"+String(x_new)+"y"+String(y_new)+"z"+String(z_new);
    pointt(c_round);
    }
  }



  else if (received.indexOf("G0") != -1){

    x_old=x_new;
    y_old=y_new;
    z_old=z_new;

    x_new=(received.substring(received.indexOf("X")+1, received.indexOf("Y"))).toInt();
    y_new=(received.substring(received.indexOf("Y")+1, received.indexOf("Z"))).toInt();
    z_new=(received.substring(received.indexOf("Z")+1, received.indexOf("A"))).toInt(); 
    a=(received.substring(received.indexOf("A")+1, received.length())).toInt();  
   
    
    // sscanf(received, "x%d y%d z%d", &axis[0], &axis[1], &axis[2]);
      // sscanf(inputString.c_str() , "x%d y%d z%d", &x, &y, &z);
// speed=0;
float dis = sqrt((x_new - x_old)*(x_new - x_old) + (y_new - y_old)*(y_new - y_old) + (z_new- z_old)*(z_new- z_old));
int d=round(dis)+1;
    for (int t = 0; t <= d; t += 1) {
      // Serial.println(d);
      // if (speed<500){
      //   speed=t*5;
      // }
      // else{
      //   speed=t/3;
      // }
  int x = x_old + (x_new - x_old) * t/d;
  int y = y_old + (y_new - y_old) * t/d;
  int z = z_old + (z_new - z_old) * t/d;

    // Serial.println(x);
    // Serial.println(y);
    // Serial.println(z-221.076);
    delta_calcInverse(x,y,z-221.076,thetas);
      
    // Serial.println(thetas[0]); 
    // Serial.println(thetas[1]);
    // Serial.println(thetas[2]);

    theta_step(thetas,steps);
    // Serial.println(steps[0]);
    // Serial.println(steps[1]);
    // Serial.println(steps[2]);
    // rotation(a*(0.7623));
    moveStepper(steps);
    }
  }
}



