const int ena = 5;
const int dir = 6;
const int pul = 7;

const int dt = 350;

const int b1 = 8;
const int b2 = 9;

bool puls = LOW;
bool direction = HIGH;



void setup() {
  pinMode(ena, OUTPUT);
  pinMode(dir, OUTPUT);
  pinMode(pul, OUTPUT);
  pinMode(b1, INPUT);
  pinMode(b2, INPUT);

  digitalWrite(ena, LOW);
  digitalWrite(dir, HIGH);
  digitalWrite(pul, HIGH);

}

void step(){
  puls = !puls;
  digitalWrite(pul, puls);
  delayMicroseconds(dt);
}
void loop() {
  if (digitalRead(b1)){
    digitalWrite(dir, LOW);
    step();
  }
  if (digitalRead(b2)){
    digitalWrite(dir, HIGH);
    step();
  }

}
