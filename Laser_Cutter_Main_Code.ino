
#include <SPI.h>
#include <SD.h>
#include <AccelStepper.h>

#define SD_ChipSelectPin 53 //Chip select is pin 53

File myFile;

// Define pin connections
const int dirPinX = 45;
const int stepPinX = 2;
const int dirPinY = 42;
const int stepPinY = 3;
const int endStopX1 = 5; //X is the lower railing. NOTE: endstop is normally high, will drop to 0V when triggered
const int endStopX2 = 6; 
const int endStopY1 = 7; //Y is the upper railing 
const int endStopY2 = 8;
const int xMotorEnable = 44; //Low turns the motor on, High turns it off
const int yMotorEnable = 43; 
const int laserPWM = 4;
const int force5Vpin24 = 24;
const int force5Vpin25 = 25;
const int force5Vpin26 = 26;
const int force5Vpin27 = 27;

//// Define motor interface type
#define motorInterfaceType 1 //1 means a stepper driver
AccelStepper stepperX(motorInterfaceType, stepPinX, dirPinX);; // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
AccelStepper stepperY(motorInterfaceType, stepPinY, dirPinY);; // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5

//Laser Cutter Settings  ////////////////////////////////////////////////////////////////////////

const float pulley_radius = 6.3; //in mm
float step_precision = 0.0314159/8; //in rad/step. 1.8deg/8 = 0.225deg
const int motorAccel = 1000;
const int steps_per_rot = 1600; //1600 steps per 1 rotation, in eighth mode 
int laserPower; //in %
int xySpeedMax = 50;//in mm/s
int cuttingSpeed; //in mm/s (3 is a good speed for cutting) 
int homeSpeed = 5; //in mm/s
int numberOfPasses; //how many times the program should go over the SD file (How many passes to make for cuts)

//End of printer settings ////////////////////////

//Start of Variables Section /////////////////////////////////////////////////////////////// 

//readLine()and parseLine() variables
const int maxLength = 150; 
char inputData[maxLength];
char currentChar;
int index;

const int axisLength = 12; //maximum length for each gcode instruction (example: x, y, z, ext, feed)
float xLast = 0.0;
float yLast = 0.0;
float x;
float y;
float dX;
float dY;
int xSteps;
int ySteps;
char xChar[axisLength];
char yChar[axisLength];
boolean xEmpty = true;
boolean yEmpty = true;

float distanceNetPerInstruction; //how far the laser cutter is travelling per instruction
float timeNetPerInstruction;
float Vx;
float Vy;

boolean laserOn = false;
boolean homeInProgress = false;
String userResponse = "N";
String fileName;
boolean testRun = false;
boolean xNotChangingThisStep = false;
boolean yNotChangingThisStep = false;

float mmToStep;
float homePosition = 37.5*8; //in steps

//End of variables Section ///////////////////////////

//Start of Functions Section ////////////////////////////////////////////////////////////////

void readLine(){
  resetData();
  index = 0;
  do{ //read the txt file line 
    currentChar = myFile.read();
    inputData[index] = currentChar;
//    Serial.print(inputData[index]);
    index = index + 1;
  } while(currentChar != '\n');
  
  return;
}

void parseLine(){
  index = 0;
  if(inputData[index] == 'G'){ //G section
    index = index + 1;      
    if(inputData[index] == '0' || inputData[index] == '1'){ //G0 and G1 section
      index = index + 2; //skip the first space after G0/1 and go to the first letter
      G01_Command();
    }
    else if(inputData[index] == '9' && inputData[index + 1] == '2' ){ //Set position section
      setNewCoordinate();
    }
    else {
      //ignore the other G-code commands
    }
  }
  else if(inputData[index] =='M'){ //M section
    index = index + 1;
    if(inputData[index] == '1' && inputData[index + 1] == '5'){ //Turn laser on
      laserOn = true;     
    }
    else if(inputData[index] == '1' && inputData[index + 1] == '6'){ //Turn laser off
      laserOn = false;
    }
    else {
      //ignore the other G-code commands
    }
  }
  else { //else it's a comment section or the command isn't necessary for the design
    //do nothing
  }
    
  return;
}

void resetData(){
  int i = 0;
  while(i<maxLength){
    inputData[i] = '\0'; // \0 means an empty array index, this will be used to find the end of the parsed command
    i = i + 1; 
  }
  i = 0;
  while(i<axisLength){
    xChar[i] = '\0'; // \0 means an empty array index, this will be used to find the end of the parsed command
    yChar[i] = '\0';
    i = i + 1; 
  }
} 

void setNewCoordinate() {
  if(xEmpty == false){
    //mm*(steps in 1 rot/mm in 1 rot)
    stepperX.setCurrentPosition((xLast*steps_per_rot/(2*3.14*pulley_radius))+1); //position (in steps) that will become the new zero location
    xLast = 0;
  }
  if(yEmpty == false){
    stepperY.setCurrentPosition((yLast*steps_per_rot/(2*3.14*pulley_radius))+1); // +1 so that current position is 1 instead of 0 if xLast = 0. 0 doesn't work for the function
    yLast = 0;
  }
}

void extractCommandData(){
  int i;
  while(inputData[index] != '\0'){
    //X section
    if(inputData[index] == 'X'){
      xEmpty = false;
      i = 0;
      index = index + 1;
      while(inputData[index] != ' ' && inputData[index] != '\0'){
        xChar[i] = inputData[index];
        index = index + 1;
        i = i + 1;
      }
      index = index + 1; //skip space 
      x = atof(xChar); //turn the strings into float values 
    }
    
    //Y section
    else if(inputData[index] == 'Y'){
      yEmpty = false;
      i = 0;
      index = index + 1;
      while(inputData[index] != ' ' && inputData[index] != '\0'){
        yChar[i] = inputData[index];
        index = index + 1;
        i = i + 1;
      }
      index = index + 1; //skip space
      y = atof(yChar);
    }

    else {
      index = index + 1;
    }
  }
}

void G01_Command() {
  xEmpty = true; //there is no x value in the command 
  yEmpty = true;
  extractCommandData();
  //find the number of steps that each motor requires
  if(xEmpty == true){
    x = xLast; //dX = 0
  }
   if(yEmpty == true){
    y = yLast; 
  }
  
  dX = (x-xLast); 
  dY = (y-yLast);

  if(dX == 0){
    xNotChangingThisStep = true;
  }
  else{
    xNotChangingThisStep = false;
  }
  
  if(dY == 0){
    yNotChangingThisStep = true;
  }
  else{
    yNotChangingThisStep = false;
  }

  xSteps = round(x/(pulley_radius*step_precision))+1; //in absolute mode, so using x and not dX
  ySteps = round(y/(pulley_radius*step_precision))+1;

  //calculate the max speed for the motors
  if(xNotChangingThisStep == false || yNotChangingThisStep == false){ //if there are movements in the x/y axis
    distanceNetPerInstruction = sqrt(pow(dX,2)+pow(dY,2));
    timeNetPerInstruction = distanceNetPerInstruction/cuttingSpeed;
    Vx = dX/timeNetPerInstruction;
    Vy = dY/timeNetPerInstruction;
    Vx = Vx*steps_per_rot/(2*3.14159*pulley_radius); //convert to steps/s by doing mm/s * 800steps/arc length in mm
    Vy = Vy*steps_per_rot/(2*3.14159*pulley_radius);
    
    runXYMotors();
  }

  xLast = x;
  yLast = y;
}

void runXYMotors() {
  stepperX.moveTo(xSteps);
  stepperX.setSpeed(Vx);
  stepperY.moveTo(ySteps);
  stepperY.setSpeed(Vy);

  if(homeInProgress == true){ //home the laser cutter
    stepperX.setSpeed(-homeSpeed);
    stepperY.setSpeed(-homeSpeed);

    while (digitalRead(endStopX1) == HIGH){ //X axis
      stepperX.runSpeed();  
    }
    stepperX.setCurrentPosition(1);
    
    //move away from end stop slightly
    stepperX.moveTo(homePosition);
    stepperX.setSpeed(homeSpeed);
    while (stepperX.distanceToGo() != 0){ 
      stepperX.runSpeedToPosition();
    }
    stepperX.setSpeed(0);
    stepperX.setCurrentPosition(1);

    while (digitalRead(endStopY1) == HIGH){ //Y axis
      stepperY.runSpeed();  
    }
    stepperY.setCurrentPosition(1);
    
    //move away from end stop slightly
    stepperY.moveTo(homePosition);
    stepperY.setSpeed(homeSpeed);
    while (stepperY.distanceToGo() != 0){ 
      stepperY.runSpeedToPosition();
    }
    stepperY.setSpeed(0);
    stepperY.setCurrentPosition(1);
  }
  
  else if(xNotChangingThisStep == false && yNotChangingThisStep == false){ //x y
    while (stepperX.distanceToGo() != 0 || stepperY.distanceToGo() != 0){ 
      stepperX.runSpeedToPosition(); 
      stepperY.runSpeedToPosition(); 
      if(digitalRead(endStopX1) == LOW || digitalRead(endStopX2) == LOW || digitalRead(endStopY1) == LOW || digitalRead(endStopY2) == LOW){
        Serial.println("End stop triggered. The file footprint is either too big or a mechanical error happened and must be fixed. Solve error and restart laser cutter");
        ESTOP();
      } 
    }
  }
  else if(xNotChangingThisStep == true && yNotChangingThisStep == false){ //y
    while (stepperY.distanceToGo() != 0){ 
      stepperY.runSpeedToPosition();
      if(digitalRead(endStopX1) == LOW || digitalRead(endStopX2) == LOW || digitalRead(endStopY1) == LOW || digitalRead(endStopY2) == LOW){
        Serial.println("End stop triggered. The file footprint is either too big or a mechanical error happened and must be fixed. Solve error and restart laser cutter");
        ESTOP();
      }  
    }
  }
  else if(xNotChangingThisStep == false && yNotChangingThisStep == true){ //x
    while (stepperX.distanceToGo() != 0){ 
      stepperX.runSpeedToPosition();
      if(digitalRead(endStopX1) == LOW || digitalRead(endStopX2) == LOW || digitalRead(endStopY1) == LOW || digitalRead(endStopY2) == LOW){
        Serial.println("End stop triggered. The file footprint is either too big or a mechanical error happened and must be fixed. Solve error and restart laser cutter");
        ESTOP();
      }  
    }
  }
}

void homeLaserCutter(){
  homeInProgress = true;
  runXYMotors();
  homeInProgress = false;
}

void checkLaserState(){
  if(laserOn == true){
    analogWrite(laserPWM, laserPower);
  }
  else {
    analogWrite(laserPWM, 0);
  }
}

void disableMotors(){
  digitalWrite(xMotorEnable, HIGH);
  digitalWrite(yMotorEnable, HIGH);
}

void enableMotors(){
  digitalWrite(xMotorEnable, LOW);
  digitalWrite(yMotorEnable, LOW);
}

void startUpMenu(){
  Serial.println("Enter C for cutting or S for shading");
  laserPower = 0;
  userResponse = "N";
  while(userResponse != "C" && userResponse != "S"){
    while (Serial.available() == 0) {
      //wait for user input
    }
    userResponse = Serial.readString();
  }
  if(userResponse == "C"){ 
    laserPower = 100;
    cuttingSpeed = 3;
    numberOfPasses = 4;
  }
  else{
    laserPower = 20;
    cuttingSpeed = 6;
    numberOfPasses = 1;
  }
  laserPower = map(laserPower, 0, 100, 0, 255); //map laser power from percent to PWM output
  
  Serial.println("Enter file name. Example: test.txt");
  fileName = "N";
  while (Serial.available() == 0) {
    //wait for user input
  }
  fileName = Serial.readString();
  Serial.println(fileName);
  
  Serial.println("Is the piece to be cut set up? Enter Y to continue");
  userResponse = "N";
  while(userResponse != "Y"){
    while (Serial.available() == 0) {
      //wait for user input
    }
    userResponse = Serial.readString();
  }
  
  Serial.println("Is the air supply connected? Enter Y to continue"); 
  userResponse = "N";
  while(userResponse != "Y"){
    while (Serial.available() == 0) {
      //wait for user input
    }
    userResponse = Serial.readString();
  } 
  
  Serial.println("IMPORTANT: Are your safety glasses on? DO NOT START WITHOUT THEM. Enter Y to continue"); 
  userResponse = "N";
  while(userResponse != "Y"){
    while (Serial.available() == 0) {
      //wait for user input
    }
    userResponse = Serial.readString();
  } 
  
  Serial.println("Is the protective cover on the laser cutter? DO NOT START WIHTOUT IT. Enter Y to continue"); 
  userResponse = "N";
  while(userResponse != "Y"){
    while (Serial.available() == 0) {
      //wait for user input
    }
    userResponse = Serial.readString();
  }
  
  Serial.println("Press B to begin laser cutting"); 
  userResponse = "N";
  while(userResponse != "B"){
    while (Serial.available() == 0) {
      //wait for user input
    }
    userResponse = Serial.readString();
  }
}

void laserCutterMainProgram(File myFile, int numberOfPasses){
  if (myFile) {
    while(numberOfPasses > 0){
      readLine();
      if(testRun == false){
        checkLaserState();
      }
      parseLine();
      if(myFile.available() == 0){
        myFile.seek(0);
        numberOfPasses = numberOfPasses - 1;
      }
    }
    if(testRun == false){
      myFile.close();
    }
  }
  
  else if(!myFile){
    Serial.println("Error reading file, please fix and restart system");
    ESTOP();
  }
}

void finishLaserCutterProgram(){
  laserOn = false;
  homeLaserCutter();
  Serial.println(F("Cutting is done! Restart laser cutter to start from the beginning"));
  disableMotors();
  while(1){
    //restart laser cutter to start from the beginning 
  }
}

void testRunProgram(File myFile){
  Serial.println("Would you like to run through the file without laser on? Press Y to do so. Press N to not");
  userResponse = "X";
  while(userResponse != "Y" && userResponse != "N"){
    while (Serial.available() == 0) {
      //wait for user input
    }
    userResponse = Serial.readString();
  }
  if(userResponse == "Y"){
    Serial.println("Starting test run");
    delay(2000);
    testRun = true;
    laserCutterMainProgram(myFile, 1); //input is the number of passes
    testRun = false;
  }
  else{
    //do nothing
  }
}

void ESTOP(){
  laserOn = false;
  disableMotors();
  myFile.close();
  while(1){
    //need to restart laser cutter
  }
}

void runAnotherFile(){
  Serial.println("Would you like to run another file? Enter Y to continue"); 
  userResponse = "N";
  while(userResponse != "Y"){
    while (Serial.available() == 0) {
      //wait for user input
    }
    userResponse = Serial.readString();
  } 
  if(userResponse == "Y"){
    disableMotors();
    laserOn = false;
    configureLaserCutter();
  }
  else{
    finishLaserCutterProgram();
  }
}

void configureLaserCutter(){
  resetData();
  delay(2000);

  //Go through start-up menu
  startUpMenu(); 

  //Home the laser cutter
  Serial.println("Homing sequence beginning");
  enableMotors();
  homeLaserCutter();
  Serial.println("Homing sequence finished");

  myFile = SD.open(fileName);
  
  //Ask to run through program without laser to test 
  testRunProgram(myFile);
}

//End of Functions Section /////////////////////////

void setup() {
  //disable motors and laser until cutting is ready
  disableMotors();
  laserOn = false;
  
  pinMode(endStopX1, INPUT);
  pinMode(endStopX2, INPUT);
  pinMode(endStopY1, INPUT);
  pinMode(endStopY2, INPUT);
  pinMode(xMotorEnable, OUTPUT);
  pinMode(yMotorEnable, OUTPUT);
  pinMode(force5Vpin25, OUTPUT);
  pinMode(force5Vpin27, OUTPUT);

  digitalWrite(force5Vpin24, HIGH);
  digitalWrite(force5Vpin25, HIGH);
  digitalWrite(force5Vpin26, HIGH);
  digitalWrite(force5Vpin27, HIGH);

  //Convert mm/s to step/s
  mmToStep = steps_per_rot/(2*3.14159*pulley_radius);
  xySpeedMax = xySpeedMax*mmToStep;
  homeSpeed = homeSpeed*mmToStep;

  stepperX.setMaxSpeed(xySpeedMax);
  stepperX.setAcceleration(motorAccel);
  stepperX.setCurrentPosition(1); //need this to make the motor start moving. Keep it at value of 1
  stepperY.setMaxSpeed(xySpeedMax);
  stepperY.setAcceleration(motorAccel);
  stepperY.setCurrentPosition(1); //need this to make the motor start moving. Keep it at value of 1
  
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    // wait for serial port to connect. Needed for native USB port only
  }
  Serial.print("Initializing SD card...");
  if (!SD.begin(SD_ChipSelectPin)) {
    Serial.println(F("initialization failed!"));
    while (1);
  }
  Serial.println(F("initialization done."));
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.

  configureLaserCutter(); //get user input and home the laser cutter
}

void loop() {
  Serial.println("Starting laser cutting");
  laserCutterMainProgram(myFile, numberOfPasses);
  runAnotherFile();
}
