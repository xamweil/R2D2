#include <Servo.h>

Servo myServo;

bool open = 0;
int buttonPin = 7;
int openPosition = 30; //upper: 30 lower: 22
int closePosition =100; //upper 165 lower:120
void setup() {
  myServo.attach(9);
}

void loop() {
  if (digitalRead(buttonPin)){
    open = !open;
    if (open){
      myServo.write(openPosition);
    }
    else{
      myServo.write(closePosition);
    }
    delay(300);
  }

}
