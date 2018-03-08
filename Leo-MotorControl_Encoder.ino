#define MOTER_A 9
#define ENCODER_A_A 3
#define ENCODER_A_B 5

volatile unsigned long count = 0;
void setup() {
  
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  pinMode(MOTOR_A,OUTPUT);
  pinMode(ENCODER_A_A, INPUT);
  pinMode(ENCODER_A_B, INPUT);
  attachInterrupt(0,readEncoderA,CHANGE);

}
void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(MOTOR_A,150);
  Serial.println(count);
  delay(500);

}

void readEncoderA()
{
  if (digitalRead(ENCODER_A_A) == HIGH) {
    if (digitalRead(ENCODER_A_B) == LOW) {
      count++;
    } else {
      count--;
    }
  } else {
    if (digitalRead(ENCODER_A_B) == LOW) {
      count--;
    } else {
      count++;
    }
  }
}

