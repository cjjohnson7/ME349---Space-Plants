#define MOTOR_A 9
#define ENCODER_A_A 2
#define ENCODER_A_B 5
#define MOTOR_A_D 10
#define ENCODER_B_A 3
#define ENCODER_B_B 6

unsigned long countA = 0;
unsigned long countB = 0;
int changed = 0;
//int temp = 0;
int initial = 0;
void setup() {
  
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  pinMode(MOTOR_A,OUTPUT);
  pinMode(ENCODER_A_A, INPUT);
  pinMode(ENCODER_A_B, INPUT);
  attachInterrupt(0,readEncoderA,CHANGE);
  attachInterrupt(1,readEncoderB,CHANGE);
  initial = digitalRead(ENCODER_A_A);
  Serial.println(initial);
  //Serial.println(digitalRead(ENCODER_A_B));
  Serial.println("Start");
  

}

void loop() {
  // put your main code here, to run repeatedly:
  // 0- 255
  //temp = Srial.read();
  //Serial.println(temp);
  //digitalWrite(MOTOR_A_D,HIGH);
  
  analogWrite(MOTOR_A,50);
  
  //Serial.println(count);
  delay(50000);

}

void readEncoderA()
{
  //changed++;
  //Serial.begin(9600);
  //Serial.print("Changed: ");
  //Serial.println(changed);
  //Serial.println(count);

  //Serial.print("Enc A: ");
  //Serial.println(digitalRead(ENCODER_A_A));
  //Serial.print("Enc B: ");
  //Serial.println(digitalRead(ENCODER_A_B));
  //Serial.println();
  
  if (digitalRead(ENCODER_A_A) == HIGH) {
    if (digitalRead(ENCODER_A_B) == LOW) {
      countA--;
    } else {
      countA++;
    }
  } else {
    if (digitalRead(ENCODER_A_B) == HIGH) {
      countA--;
    } else {
      countA++;
    }
  }

  Serial.print("Enc A: ");
  Serial.println(countA);
}

void readEncoderB()
{
  //changed++;
  //Serial.begin(9600);
  //Serial.print("Changed: ");
  //Serial.println(changed);
  //Serial.println(count);

  //Serial.print("Enc A: ");
  //Serial.println(digitalRead(ENCODER_A_A));
  //Serial.print("Enc B: ");
  //Serial.println(digitalRead(ENCODER_A_B));
  //Serial.println();
  
  if (digitalRead(ENCODER_B_A) == HIGH) {
    if (digitalRead(ENCODER_B_B) == LOW) {
      countB--;
    } else {
      countB++;
    }
  } else {
    if (digitalRead(ENCODER_B_B) == HIGH) {
      countB--;
    } else {
      countB++;
    }
  }

  Serial.print("Enc B: ");
  Serial.println(countB);
}

