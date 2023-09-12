/**
Speed 1 = Every Programm Zyklen
Speed 0.5 = Alle zwei Program Zyklen
*/

float programFrequenzy = 1200; // Run program in 1kHz oder in Schritte pro Secunde
float highTime = 400; // In Mikrosecunden
float lowTime = 400; // In Mikrosecunden


struct Motor {
  int stepPin; 
  int dirPin;
  int speed; // in Steps per Second
  long offset;

  float idleAmount;
  float idleCounter;
};

Motor xMotor = {2, 5, 400, 0, 1.0, 0};
Motor yMotor = {3, 6, 400, 0, 1.0, 0};
Motor zMotor = {4, 7, 400, 0, 1.0, 0};

void setup() {
  Serial.begin(9600);
  Serial.println("Initializing...");

  initMotor(xMotor);
  initMotor(yMotor);
  initMotor(zMotor);


 Serial.println("Initialized");
}

unsigned long oldTime = 0; 
void loop() {

  unsigned long currentTime = micros(); 
  if (currentTime - oldTime < 1 / programFrequenzy * 1000000) return;
  oldTime = currentTime; 

  checkSerial();

  xMotor = executeMotor(xMotor);
  yMotor = executeMotor(yMotor);
  zMotor = executeMotor(zMotor);


  delayMicroseconds(highTime);
  digitalWrite(xMotor.stepPin, LOW);
  digitalWrite(yMotor.stepPin, LOW);
  digitalWrite(zMotor.stepPin, LOW);
  delayMicroseconds(lowTime);
}


void checkSerial() {
  while (Serial.available() >= 3) {
    if(Serial.read() != ',') continue;
    
    // PRE COMMAND
    char type = Serial.read();
    char axis = Serial.read();
    Serial.print("Command: Type=");
    Serial.print(type);
    Serial.print(" Axis=");
    Serial.println(axis);
    Motor currentMotor;
    switch(axis) {
      case 'x':
      currentMotor = xMotor;
      break;
      case 'y':
      currentMotor = yMotor;
      break;
      case 'z':
      currentMotor = zMotor;
      break;
      default: 
        Serial.print("Unknown axis ");
        Serial.println(axis);
      break;
    }

    // COMMAND
    bool commandCheck = false;
    if(type == 'm' && !commandCheck) {// Move
        commandCheck = true;
        int offset = Serial.parseInt();
        currentMotor.offset += offset;

        Serial.print("offset ");
        Serial.print(axis);
        Serial.print(" axis by ");
        Serial.print(offset);
        Serial.print(" to ");
        Serial.println(currentMotor.offset);
    }
    
    if(type == 's' && !commandCheck) {// Speed
        commandCheck = true;
        int speed = Serial.parseInt();
        currentMotor.speed = speed;
        currentMotor.idleAmount = float(currentMotor.speed) / programFrequenzy;

        Serial.print("set ");
        Serial.print(axis);
        Serial.print(" axis speed to ");
        Serial.print(currentMotor.speed);
        Serial.print(" and idleAmount to ");
        Serial.println(currentMotor.idleAmount);
    }

    if(type == 'x' && !commandCheck) { // Stop
        commandCheck = true;
        Serial.print("stopped ");
        Serial.print(axis);
        Serial.println(" axis"); 
    }

    if(!commandCheck) {
        commandCheck = true;
        Serial.print("Unknown command!");
        Serial.println(type);
    }
  
    // POST COMMAND
    switch(axis) {
      case 'x':
      xMotor = currentMotor;
      break;
      case 'y':
      yMotor = currentMotor;
      break;
      case 'z':
      zMotor = currentMotor;
      break;
    }
  }
}


void initMotor(Motor motor) {
 pinMode(motor.stepPin, OUTPUT);
 pinMode(motor.dirPin, OUTPUT);
}


Motor executeMotor(Motor motor) {
  motor.idleCounter += motor.idleAmount;

  if (motor.idleCounter >= 1) {
    motor.idleCounter--;
      

    if(motor.offset != 0) {
      if (motor.offset > 0) {
        digitalWrite(motor.dirPin, LOW);
        motor.offset--;
      }

      if (motor.offset < 0) {
        digitalWrite(motor.dirPin, HIGH);
        motor.offset++;
      }
    
      digitalWrite(motor.stepPin, HIGH);
    }
  }

  return motor;
}
