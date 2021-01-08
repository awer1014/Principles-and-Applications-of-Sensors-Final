#include <Servo.h>//引入Servo
Servo __myservo10;//定義 __myservo10

const int TouchPin = 2;//觸控輸入端 A0
int count = 0; //計數器
int m = 10; //設定初始位置

void setup() {//設定初始直
  Serial.begin(9600);
  pinMode(TouchPin, INPUT);//輸入端A0
  __myservo10.attach(m);//把馬達位置設置在m
}


void touch(){
  int sensorValue = digitalRead(TouchPin);//sensorValue 接收觸控訊號
  if(sensorValue == 1){//若有訊號
    count += 1;   //次數+1
    }
   if (count>=10){
    count = 0; //超過10歸0
    }
    //Serial.print("count:");
    //Serial.println(count);
    m = count*20; //設置位置 m = 次數*20
    //Serial.print("move:");
    //Serial.println(m);
    
  delay(100);//延遲100ms
  }

void loop() {
 touch();//接收訊號
 __myservo10.write(m); //把馬達位置設置在m
 //Serial.print("write:");
 //Serial.println(m);
}
