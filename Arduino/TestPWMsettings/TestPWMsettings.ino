#define TIME_SIZE 20

int pwmInterruptPin = 10;   // D10 - https://www.arduino.cc/reference/de/language/functions/external-interrupts/attachinterrupt/

float tRise, tFall, tEnd;
int currIndex = 0;


unsigned long timing[TIME_SIZE];
unsigned long state[TIME_SIZE];

void ISR_PWM();


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);     // For Debugging -> use output window or h-term

  pinMode(pwmInterruptPin, INPUT_PULLDOWN);
  pinMode(LED_BUILTIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(pwmInterruptPin), ISR_PWM, CHANGE);

  Serial.println("Its a mee mario");
}

void loop() {
  // put your main code here, to run repeatedly:


for (int i = 0; i < TIME_SIZE; i++){
          Serial.print(timing[i]-timing[0]);
          Serial.print(", ");
        }
        Serial.println();
        for (int i = 0; i < TIME_SIZE; i++){
          Serial.print(state[i]);
          Serial.print(", ");
        }
        Serial.println();
        Serial.println();
        Serial.println();

        delay(2000);
}



void ISR_PWM() {
  static int counter = 0;

  timing[counter] = micros();
  if (digitalRead(pwmInterruptPin) == LOW) {
    state[counter] = 0;
  } else {
    state[counter] = 1;
  }
  counter++;
  
  if (counter >= TIME_SIZE) {
    counter = 0;
  }

  digitalWrite(LED_BUILTIN, digitalRead(pwmInterruptPin));
}
