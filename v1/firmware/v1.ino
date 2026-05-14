const int escPin = 19;

const int MIN_THROTTLE = 900; //us
const int START_THROTTLE = 1500;

const int PWM_PERIOD = 20000; //1/50 hz = 20 ms = 20 000 us

void escPulse(int pulseWidth, int pin){
  digitalWrite(pin, HIGH);
  delayMicroseconds(pulseWidth);
  digitalWrite(pin, LOW);
  delayMicroseconds(PWM_PERIOD - pulseWidth);
}

void setup() {
  pinMode(escPin, OUTPUT);

  for(int i = 0; i < 300; i++) {  // 300 * 20ms = 6s
    escPulse(MIN_THROTTLE, escPin);
  }

}

void loop() {

  for(int i = 0; i < 250; i++){
    escPulse(START_THROTTLE, escPin);
  }
  
  for(int i = 0; i < 250; i++){
    escPulse(MIN_THROTTLE, escPin);
  }

}
