
#define _SAMPLE_COUNT 50


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(A7, INPUT); //I’ve tried all of the audio input pins, btw
}

void loop() {
  // put your main code here, to run repeatedly:

  float t0, t;

  t0 = micros() ;
  for( int i =0; i <_SAMPLE_COUNT; i++)
  {
  analogRead (A7);
  }

  t = micros() - t0;

  Serial.print("Time per sample: ");
  Serial.print((float)t/_SAMPLE_COUNT);
  Serial.println(" uS");
  Serial.print("Frequency: ");
  Serial.print((float)_SAMPLE_COUNT*1000000/t);
  Serial.print(" HZ");
  Serial.println();

  delay( 2000 );


}
