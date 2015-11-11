#include <Arduino.h>

#include <Servo.h>
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h> //ros_serialから受け取るメッセージの型。今回は番号

ros::NodeHandle  nh;

Servo servo1; //瞼サーボ
Servo servo2; //眼サーボ

int pos1; //瞼サーボに送る角度
int pos2; //眼サーボに送る角度

int tp = 3000;  // 撮影後に音を鳴らしたり画面に出したりするための時間
int wt = 5000; //撮影後に半目で休む時間
long r=0; //ランダム周期のための数値

int wa = 2; //瞬きの加速度
int ma = 3; //眼の角度変更の加速度

int Open = 75; //瞼を全て開く角度
int Half = 45; //半目の角度
int Close = 0; //瞼を全て閉じる角度

int L1 = 0; //眼の向き左端
int L2 = 15; //眼の向き左寄り
int Center = 30; //眼の向き真ん中
int R2 = 45; //眼の向き右寄り
int R1 = 60; //眼の向き右端



void def(){  //rosから信号を受け取るより前の標準の状態：瞼を半開きにする角度をサーボに送る。リセットしたらここから。この状態では赤LEDが光る
  pos1 = Half;
  pos2 = Center;
  servo1.write(pos1);
  servo2.write(pos2);
  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)  
}

void blink3(){ //LEDの点滅
  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)  
  delay(100);
  digitalWrite(13, LOW);   // turn the LED on (HIGH is the voltage level)  
  delay(100);
  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)  
  delay(100);
  digitalWrite(13, LOW);   // turn the LED on (HIGH is the voltage level) 
  delay(100);
  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)  
  delay(100);
  digitalWrite(13, LOW);   // turn the LED on (HIGH is the voltage level)  
  delay(10); 
}

void close_eye(const std_msgs::Int32& msg) { //
  if(msg.data == 1) {  //人の顔を（0.5秒？）検出したので瞼を半開きから全開きにする。rosからの信号=1
    
    digitalWrite(13, LOW);   // turn the LED off
 
    for(pos1 = Half; pos1 < Open; pos1 += wa){  // 眼を半開きから全開きへ                                                                                                                                     
        servo1.write(pos1);                                                                                                                                  
        delay(10);                                                                                                                              
       }
  } 
  
  else if(msg.data == 2) {  //人の顔をn秒連続で検出したので全開きから瞬きして写真撮影する。rosからの信号=2
    
    digitalWrite(13, LOW);   // turn the LED off
    for(pos1 = Open; pos1 >= Close; pos1 -= wa){  // 眼を全開きから全閉じへ                                                                                                                                     
        servo1.write(pos1);                                                                                                                                  
        delay(10);                                                                                                                              
       }
    for(pos1 = Close; pos1 < Open; pos1 += wa*3){   // 眼を全閉じから全開きへ                                                                                                                                   
        servo1.write(pos1);                                                                                                                                
        delay(10);                                                                                                                                     
       }
    delay(tp);
    pos1 = Half;
    servo1.write(pos1);
    digitalWrite(13, HIGH);   // turn the LED on
    delay(wt);
  } 

  else if(msg.data == 3) {  //人の顔を見失ったので半目に戻る。rosからの信号=3
    
    digitalWrite(13, LOW);   // turn the LED off
    
    for(pos1 = Open; pos1 > Half; pos1 -= wa/3){  // 眼を全開きから全閉じへ                                                                                                                                     
        servo1.write(pos1);                                                                                                                                  
        delay(10);                                                                                                                              
       }
    delay(10);
  } 

  else if(msg.data == 4) {  //人の顔がなくて暇なのでキョロキョロする。中心ー右端ー左寄りー中心。rosからの信号=4
    
    digitalWrite(13, LOW);   // turn the LED off
    r= random(500,5000);
    pos1 = Half;
    servo1.write(pos1);
    
    for(pos2 = Center; pos2 < R1; pos2 += ma){  // 眼を中心から右端へ                                                                                                                                     
        servo2.write(pos2);                                                                                                                                  
        delay(r/10);                                                                                                                              
      }
    for(pos2 = R1; pos2 >= L2; pos2 -= ma*2){   // 眼を右端から左寄りへ                                                                                                                                   
        servo1.write(pos2);                                                                                                                                
        delay(r/7);                                                                                                                                     
      }
    for(pos2 = L2; pos2 < Center; pos2 -= ma){   // 眼を右端から左寄りへ                                                                                                                                   
        servo1.write(pos2);                                                                                                                                
        delay(100);                                                                                                                                     
      }
    blink3(); //確認用にLED点滅
    delay(r);  
  }
}

ros::Subscriber<std_msgs::Int32> sub("close_eye", close_eye);
void setup(){
    pinMode(13, OUTPUT); //LEDピン
    nh.initNode();
    nh.subscribe(sub);
    servo1.attach(9);  //9番ピンが瞼サーボ
    servo2.attach(10); //10番ピンが眼サーボ
    nh.getHardware()->setBaud(57600);
    def();                                                                                                                                                                   
}

void loop(){
    //def();
    nh.spinOnce();
    delay(1);
}









