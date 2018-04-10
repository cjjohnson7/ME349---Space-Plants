#define MOTOR_A 9
#define MOTOR_B 8
#define ENCODER_A_A 2
#define ENCODER_A_B 5 //not used
#define MOTOR_A_D 10  //not used
#define ENCODER_B_A 3
#define ENCODER_B_B 6
#define SPEEDA A0
#define SPEEDB A1


//double setPoint = 15;

unsigned long countA = 0;
unsigned long countB = 0;
int changed = 0;
int initial = 0;
int motorADirection = 0;
int motorBDirection = 0;
double lastSpeedA = 0;
double lastSpeedB = 0;
int pwmA = 0;
int pwmB = 0;

//control global variables
double iTermA = 0;
double iTermB = 0;

//control parameter
double kPA = 10;
double kIA = 1;
double kDA = 0.5;

double kPB = 10;
double kIB = 1;
double kDB = 0.5;

//speed calculation variables
unsigned long previousTimeA = 0;
unsigned long previousTimeB = 0;
unsigned long previousCountA = 0;
unsigned long previousCountB = 0;


void setup() {
  
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  pinMode(MOTOR_A,OUTPUT);
  pinMode(ENCODER_A_A, INPUT);
  pinMode(ENCODER_A_B, INPUT);
  pinMode(SPEEDA, INPUT);
  attachInterrupt(0,readEncoderA,CHANGE);
  attachInterrupt(1,readEncoderB,CHANGE);
  initial = digitalRead(ENCODER_A_A);
  Serial.println(initial);
  //Serial.println(digitalRead(ENCODER_A_B));
  Serial.println("Start");
  previousTimeA = millis();
  previousTimeB = millis();

}

void loop() {
  // put your main code here, to run repeatedly:
  // 0- 255
  //temp = Srial.read();
  //Serial.println(temp);
  //digitalWrite(MOTOR_A_D,HIGH);
  
  //analogWrite(MOTOR_A,50);
  delay(500);
  //getSpeedA();
  PIDA();

 PIDB();
  

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
  /*
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
  */
  countA++;
  //Serial.print("Enc A: ");
  //Serial.println(countA);
 
 // countA++;
}

void readEncoderB()
{

  /*
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
  */
  countB++;
  //Serial.print("Enc B: ");
  //Serial.println(countB);
}

double getSpeedA()
{
  unsigned long time = millis();
  //Serial.print("time: ");
  //Serial.print(time-previousTimeA);
  //Serial.print(" count: ");
  unsigned long currentCount = countA;
  //Serial.print((currentCount-previousCountA));
  double currentSpeed =  (double) (currentCount-previousCountA)/(double)(time - previousTimeA);
  currentSpeed = currentSpeed*1000*60/24/227;
  previousTimeA = time;
  previousCountA = currentCount;
  //Serial.print(" speed: ");
  //Serial.println(currentSpeed);
  return currentSpeed;
}

double getSpeedB()
{
  unsigned long time = millis();
  return 0;
}

int constrainPWM( int pwm)
{
  if(pwm < 0)
     pwm = 0;
  else if(pwm > 255)
     pwm = 255;
  return pwm;
}

double readSetPointA()
{
  double setPoint = 0;
  Serial.print(analogRead(SPEEDA));
  setPoint = (double)analogRead(SPEEDA)/1024*10;
  return setPoint;
}

double readSetPointB()
{
  double setPoint = 0;
  Serial.print(analogRead(SPEEDB));
  setPoint = (double)analogRead(SPEEDB)/1024*10;
  return setPoint;
}

void PIDA()
{
  
  double speed = getSpeedA(); // motor control returns vector speed
  Serial.print("SpeedA: ");
  Serial.println(speed);
  if (speed < 0) speed *= -1;  // convert speed to scalar
  //Serial.print("Error: ");
  double setPoint = readSetPointA();
  Serial.print(" SetPointA: ");
  Serial.print(setPoint);
  double error = setPoint - speed;  // calculate error
  //Serial.print(error);
  iTermA += (kIA * error); // calculate integral term
  double dInput = speed - lastSpeedA; // calculate derivative
  int adjustment = (kPA * (double)error) + iTermA - (kDA * dInput);
  pwmA += adjustment;
  pwmA = constrainPWM(pwmA); // limit _pwm to the range 0-255
  Serial.print(" PWMA: ");
  Serial.println(pwmA);
  analogWrite(MOTOR_A,pwmA);
  //set motor
  lastSpeedA = speed;
}
void PIDB()
{
  
  double speed = getSpeedB(); // motor control returns vector speed
  Serial.print("SpeedB: ");
  Serial.println(speed);
  if (speed < 0) speed *= -1;  // convert speed to scalar
  //Serial.print("Error: ");
  double setPoint = readSetPointB();
  Serial.print(" SetPointB: ");
  Serial.print(setPoint);
  double error = setPoint - speed;  // calculate error
  //Serial.print(error);
  iTermB += (kIB * error); // calculate integral term
  double dInput = speed - lastSpeedB; // calculate derivative
  int adjustment = (kPB * (double)error) + iTermB - (kDB * dInput);
  pwmB += adjustment;
  pwmB = constrainPWM(pwmB); // limit _pwm to the range 0-255
  Serial.print(" PWMB: ");
  Serial.println(pwmB);
  analogWrite(MOTOR_B,pwmB);
  //set motor
  lastSpeedA = speed;
}

