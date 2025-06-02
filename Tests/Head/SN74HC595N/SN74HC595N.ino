static int SRCLK = 1;
static int RCLK = 2;
static int SER = 3;
int arr[30] = {0};
String currentCommand = "default";

void setup() {
  Serial.begin(9600);
  pinMode(SRCLK, OUTPUT);
  pinMode(RCLK, OUTPUT);
  pinMode(SER, OUTPUT);
  digitalWrite(SRCLK, LOW);
  digitalWrite(RCLK, LOW);
  digitalWrite(SER, LOW);

  /*everything is 'off' when Segments are HIGH and digs are HIGH. 
    In the shift register the first 4 pins (last to fill) are the segments, 
    followed by 26 Dig pins.
  */
  for(int i=0; i<30; i++){
    arr[i] = 1;
  }

  writeToRegister(arr);
  Serial.println("Enter command: chase, chase_row, chase_col");
}
void reset_arr(){
  for(int i=0; i<30; i++){
    arr[i]=1;
  }
  writeToRegister(arr);
}

void writeToRegister(const int bits[8]){
  for(int i=29; i>=0; i--){
    digitalWrite(SER, bits[i]);
    //delayMicroseconds(50);
    digitalWrite(SRCLK, HIGH);
    //delayMicroseconds(50);
    digitalWrite(SRCLK, LOW);
    //delayMicroseconds(50);
  }
  digitalWrite(RCLK, HIGH);
  //delayMicroseconds(50);
  digitalWrite(RCLK, LOW);
  //delayMicroseconds(50);
}


void chase(){
  while (currentCommand == "chase") {
    for(int i=0; i<4; i++){
      arr[i] = 0;
      for(int j=4; j<30; j++){
        arr[j] = 0;
        writeToRegister(arr);
        if (checkForNewCommand()) return;
        delay(300);
        arr[j] = 1;
      }
      arr[i]= 1;
    }
  }
}
void chaseRow(){
  for (int i=0; i<4; i++){
    arr[i]=0;
  }
  while (currentCommand == "chase_row"){
    for(int j=4; j<30; j++){
      arr[j] = 0;
      writeToRegister(arr);
      if (checkForNewCommand()) break;
      delay(300);
      arr[j] = 1;
    }
  }
}
void chaseCol(){
  for (int i=4; i<30; i++){
    arr[i]=0;
  }
  while (currentCommand == "chase_col"){
    for(int j=0; j<4; j++){
      arr[j] = 0;
      writeToRegister(arr);
      if (checkForNewCommand()) break;
      delay(300);
      arr[j] = 1;
    }
  }
}

void allOn(){
  for (int i=4; i<30; i++){
    arr[i]=0;
    
  }
  while (currentCommand == "all_on"){
    for(int j=0; j<4; j++){
      arr[j] = 0;
      writeToRegister(arr);
      if (checkForNewCommand()) break;
      delayMicroseconds(50);
      arr[j] = 1;
    }
  }
}
bool checkForNewCommand() {
  if (Serial.available() > 0) {
    currentCommand = Serial.readStringUntil('\n');
    currentCommand.trim(); // Remove any leading/trailing whitespace.
    reset_arr();
    Serial.print("New command: ");
    Serial.println(currentCommand);
    return true;
  }
  return false;
}

void loop() {
  // Check if a new command has been received.
  if (Serial.available() > 0) {
    currentCommand = Serial.readStringUntil('\n');
    currentCommand.trim(); // remove any extra spaces/newline characters
    Serial.print("Command received: ");
    Serial.println(currentCommand);
  }
  // Decide which animation function to run based on the current command.
  if (currentCommand == "chase") {
    chase();
  } else if (currentCommand == "chase_row") {
    chaseRow();
  } else if (currentCommand == "chase_col") {
    chaseCol();
  } else if (currentCommand == "all_on") {
    allOn();
  }  else {
    // If no valid command is received, just idle.
    delay(10);
  }
}
