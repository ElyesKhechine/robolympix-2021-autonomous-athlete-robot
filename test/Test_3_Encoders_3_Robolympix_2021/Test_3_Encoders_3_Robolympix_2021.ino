#define outputAR 2
#define outputBR 3
#define outputAL 18
#define outputBL 19

/*#define readAR bitRead(PINE,4)
#define readBR bitRead(PINE,5)
#define readAL bitRead(PIND,1)
#define readBL bitRead(PIND,0)
*/
boolean readAR=false,readBR=false,readAL=false,readBL=false;

volatile unsigned int temp=20;
float Rcmd=500,Lcmd=500,Rerror,Lerror;

volatile float Rdist=0.0;
volatile float Ldist=0.0;
float distUnitR=360*40/(1024*4);
float distUnitL=360*40/(600*4);

void ISRtrackAR(){
  readAR=(digitalRead(outputAR)==HIGH);
  Rdist += (readAR==readBR) ? distUnitR : (- distUnitR);
  if (Rdist/distUnitR!=temp){
    Serial.print("Rdist= ");
    Serial.println(Rdist);
    Rerror=Rcmd-Rdist;  
    Serial.print("Rerror= ");
    Serial.println(Rerror);
    temp=Rdist/distUnitR;
  }
}
void ISRtrackBR(){
  readBR=(digitalRead(outputBR)==HIGH);
  Rdist += (readAR!=readBR) ? distUnitR : (- distUnitR);
  if (Rdist/distUnitR!=temp){
    Serial.print("Rdist= ");
    Serial.println(Rdist);
    Rerror=Rcmd-Rdist;  
    Serial.print("Rerror= ");
    Serial.println(Rerror);
    temp=Rdist/distUnitR;
  }
}

void ISRtrackAL(){
  readAL=(digitalRead(outputAL)==HIGH);
  Ldist += (readAL==readBL) ? distUnitL : (- distUnitL);
  if (Rdist/distUnitR!=temp){
    Serial.print("Ldist= ");
    Serial.println(Ldist);
    Lerror=Lcmd-Ldist;  
    Serial.print("Lerror= ");
    Serial.println(Lerror);
}

void ISRtrackBL(){
  readBL=(digitalRead(outputBL)==HIGH);
  Ldist += (readAL!=readBL) ? distUnitL : (- distUnitL);
  Serial.print("Ldist= ");
  Serial.println(Ldist);
  Lerror=Lcmd-Ldist;  
  Serial.print("Lerror= ");
  Serial.println(Lerror);
}

void setup() {
Serial.begin(9600);
pinMode(outputAR, INPUT_PULLUP);
pinMode(outputBR, INPUT_PULLUP);
pinMode(outputAL, INPUT_PULLUP);
pinMode(outputBL, INPUT_PULLUP);

readAR=digitalRead(outputAR);
readBR=digitalRead(outputBR);
readAL=digitalRead(outputAL);
readBL=digitalRead(outputBL);

attachInterrupt(digitalPinToInterrupt(outputAR),ISRtrackAR,CHANGE);
//attachIn  terrupt(digitalPinToInterrupt(outputAL),ISRtrackAL,CHANGE);
attachInterrupt(digitalPinToInterrupt(outputBR),ISRtrackBR,CHANGE);
//attachInterrupt(digitalPinToInterrupt(outputBL),ISRtrackBL,CHANGE);
}

void loop() {
  

}
