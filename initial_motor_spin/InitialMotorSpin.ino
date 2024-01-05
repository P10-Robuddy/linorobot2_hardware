//Left Motor
#define encA_L 7
#define encB_L 6
#define enA 5 // Enable pin (PWM)
#define in1 2
#define in2 3


//Right Motor
#define encA_R 16
#define encB_R 15
#define enB 14 // Enable pin (PWM)
#define in3 35
#define in4 36


void setup() {
  // put your setup code here, to run once:
  pinMode(enA,OUTPUT);
  pinMode(enB,OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

}

void demoOne(){
  //Run Both motors at fixed speed

  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);

  analogWrite(enA, 200);

  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  analogWrite(enB, 200);

  delay(2000);

  //Change direction

  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);


  //Disable motors:
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

}

void demoTwo(){
  //Ramp up speed with PWM
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  for (int i = 0; i < 256; i++){
    analogWrite(enA, i);
    analogWrite(enB, i);
    delay(20);
  }

  for (int i = 255; i >= 0; i--){
    analogWrite(enA, i);
    analogWrite(enB, i);
    delay(20);
  }

  //Disable motors:
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

}

void loop() {

  demoOne();
  delay(2000);
  demoTwo();
  delay(2000);

}
