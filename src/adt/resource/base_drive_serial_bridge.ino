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
  int offset;

  int idleCounter;
};

Motor xMotor = {2, 5, 0.1, 400, 0};
Motor yMotor = {3, 6, 0.05, 200, 0};
Motor zMotor = {4, 7, 0.05, -200, 0};

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
  if (Serial.available() > 0) {
    // read the incoming byte:
    char type = Serial.read();
    
    switch(type) {
      case 'm':

      break;
      default:
        Serial.write("Unknown command! " + type + "")
      break;
    }

    // say what you got:
    Serial.print("I received: ");
    Serial.println(incomingByte, DEC);
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
