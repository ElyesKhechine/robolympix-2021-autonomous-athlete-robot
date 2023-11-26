#define outputAR 2
#define outputBR 3
#define outputAL 18
#define outputBL 19

volatile unsigned int temp=20;
volatile float Rcmd=5,Lcmd=5,Rerror,Lerror;
volatile float Rdist=0.0;
volatile float Ldist=0.0;
volatile float distUnitR=(float)PI*40/1024/10;
volatile float distUnitL=(float)PI*40/600/10;

void ISRtrackAR(){
  Rdist+= (digitalRead(outputBR)==LOW) ? distUnitR : -distUnitR;

}
void ISRtrackBR(){
  Rdist+= (digitalRead(outputAR)==LOW) ? -distUnitR : distUnitR;
}

void ISRtrackAL(){
  Ldist+= (digitalRead(outputBL)==LOW) ? distUnitL : -distUnitL;
}

void ISRtrackBL(){
  Ldist+= (digitalRead(outputAL)==LOW) ? -distUnitL : distUnitL;
}

void setup() {
Serial.begin(9600);
pinMode(outputAR, INPUT_PULLUP);
pinMode(outputBR, INPUT_PULLUP);
pinMode(outputAL, INPUT_PULLUP);
pinMode(outputBL, INPUT_PULLUP);

attachInterrupt(digitalPinToInterrupt(outputAR),ISRtrackAR,RISING);
attachInterrupt(digitalPinToInterrupt(outputAL),ISRtrackAL,RISING);
attachInterrupt(digitalPinToInterrupt(outputBR),ISRtrackBR,RISING);
attachInterrupt(digitalPinToInterrupt(outputBL),ISRtrackBL,RISING);
}

void loop() {
  if (Rdist/distUnitR!=temp){
    Serial.print("Rdist= ");
    Serial.print(Rdist);
    Rerror=Rcmd-Rdist;  
    Serial.print("\tRerror= ");
    Serial.print(Rerror);
    temp=Rdist/distUnitR;
  }
  if (Ldist/distUnitL!=temp){
    Serial.print("\tLdist= ");
    Serial.print(Ldist);
    Lerror=Lcmd-Ldist;  
    Serial.print("\tLerror= ");
    Serial.println(Lerror);
    temp=Ldist/distUnitL;
  }
}
