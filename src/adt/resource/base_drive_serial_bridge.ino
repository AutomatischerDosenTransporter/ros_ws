/**
Speed 1 = Every Programm Zyklen
Speed 0.5 = Alle zwei Program Zyklen
*/

float programFrequenzy = 4000; // Run program in 1kHz oder in Schritte pro Secunde
float highTime = 400; // In Mikrosecunden


struct Motor {
  int stepPin;
  int dirPin;
  float speed;
  long offset;

  int idleCounter;
};

Motor xMotor = {2, 5, 1.0, 400, 0};
Motor yMotor = {3, 6, 1.0, 400, 0};
Motor zMotor = {4, 7, 1.0, 400, 0};

void setup() {
  Serial.begin(9600);
  Serial.println("Initializing...");

  initMotor(xMotor);
  initMotor(yMotor);
  initMotor(zMotor);


 Serial.println("Initialized");
}

void loop() {
  checkSerial();

  xMotor = executeMotor(xMotor);
  yMotor = executeMotor(yMotor);
  zMotor = executeMotor(zMotor);


  delayMicroseconds(highTime);
  digitalWrite(xMotor.stepPin, LOW);
  digitalWrite(yMotor.stepPin, LOW);
  digitalWrite(zMotor.stepPin, LOW);
  delayMicroseconds((((float) 1) / programFrequenzy) * 1000000 - highTime);
}


void checkSerial() {
  while (Serial.available() >= 2) {
    // PRE COMMAND
    char type = Serial.read();
    char axis = Serial.read();
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

    if(axis != 'x' && axis != 'y' && axis != 'z') continue;

    // COMMAND
    switch(type) {
      case 'm': // Move
      int offset = Serial.parseInt();
      currentMotor.offset += offset;

      Serial.print("Move axis ");
      Serial.print(axis);
      Serial.print(" ");
      Serial.print(offset);
      Serial.print(" steps");
      break;
      case 's': // Speed
      Serial.print("Speed axis ");
      Serial.print(axis);
      Serial.println();

      break;
      case 'x': // Stop
      Serial.print("Stop axis ");
      Serial.print(axis);
      Serial.println();

      break;
      default:
        Serial.print("Unknown command!");
        Serial.println(type);
      break;
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
  int idleAmount = 1.0 / motor.speed;
  if(motor.idleCounter < idleAmount) {
    motor.idleCounter++;
    return motor;
  } else {
    motor.idleCounter = 0;
  }


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
  return motor;
}
