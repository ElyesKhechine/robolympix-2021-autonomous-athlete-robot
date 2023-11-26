//Variables encodeurs
#define outputAR 2
#define outputBR 3
#define outputAL 18
#define outputBL 19
volatile float Rdist = 0;
volatile float Ldist = 0;
volatile float distUnitR = (float)PI * 40 / 1024 / 10;
volatile float distUnitL = (float)PI * 40 / 600 / 10;

float Rdistacc=0;
float Ldistacc=0;
float Rdistdec=0;
float Ldistdec=0;

volatile unsigned long firstTime;
volatile boolean Stop=false;

//Variables moteurs
volatile float vitesseR = 0 ;
volatile float vitesseL = 0 ;
volatile float vitesseO = 0 ;
#define forwardR 8
#define backwardR 9
#define forwardL 10
#define backwardL 11

//Variables erreurs/commandes position
float Rcmd = 0;
float Lcmd = 0; 
volatile float Rerror;
volatile float Lerror;
volatile float prevRerror = 0;
volatile float prevLerror = 0;
volatile float sommeRerror = 0;
volatile float sommeLerror = 0;

//Variables erreurs vitesse
volatile float Oerror;
volatile float prevOerror = 0;
volatile float sommeOerror = 0;

//Encoder functions
void ISRtrackAR() {
  Rdist += (digitalRead(outputBR) == LOW) ? distUnitR : -distUnitR;
}
void ISRtrackBR() {
  Rdist += (digitalRead(outputAR) == LOW) ? -distUnitR : distUnitR;
}

void ISRtrackAL() {
  Ldist += (digitalRead(outputBL) == LOW) ? distUnitL : -distUnitL;
}

void ISRtrackBL() {
  Ldist += (digitalRead(outputAL) == LOW) ? -distUnitL : distUnitL;
}

bool inRange(volatile double val, volatile double minimum, volatile double maximum) {
  return ((minimum <= val) && (val <= maximum));
}
double sign (volatile double val){
  if (val>=0)
    return 1;
  else
    return -1;
}

//Motor functions
void motorR(volatile float vR) {
  if (vR > 0) {
    analogWrite(forwardR, vR);
    analogWrite(backwardR, 0);
  }
  else {
    analogWrite(forwardR, 0);
    analogWrite(backwardR, -vR); //5.2
  }
}

void motorL(volatile float vL) {
  if (vL > 0) {
    analogWrite(forwardL, vL);
    analogWrite(backwardL, 0);
  }
  else {
    analogWrite(forwardL, 0);
    analogWrite(backwardL, -vL); //5.2
  }
}

//PID Position Parameters

float kpR = 0.003 ; // 0.01 not great
float kpL = 0.04 ; // 0.01 Good

float kdR = 10; // 50 GOOD
float kdL = 14; // 50 Not great

float kiR = 0.0001 ; // 0.000000000001
float kiL = 0.00015; 

float kpO = 10; // 10 GOOD
float kdO = 10;  // 1 GOOD
float kiO = 0.00001; // 0.003


//Marges d'erreurs
//volatile double margePRdist = 1;
//volatile double margeNRdist = -1;
//volatile double margePLdist = 1;
//volatile double margeNLdist = -1;

volatile double margePVRerror = 1;
volatile double margeNVRerror = 1;
volatile double margePVLerror = 1;
volatile double margeNVLerror = 1;

volatile double margePRerror = 1;
volatile double margeNRerror = -1;
volatile double margePLerror = 1;
volatile double margeNLerror = -1;

volatile double margePOerror = 1;
volatile double margeNOerror = -1;

volatile float kvR=0;
volatile float kvL=0;
volatile float penteR=0;
volatile float penteL=0;

void PIDorientation() {
  Oerror = Rdist - Ldist;
   if (!inRange(Oerror, margeNOerror, margePOerror)) {
  sommeOerror += Oerror;
  vitesseO = (kpO * Oerror) + (kdO * (Oerror - prevOerror)) + kiO * sommeOerror;
  prevOerror = Oerror;
 }
 else
    vitesseO=0;
}

void PIDposition() {

  Rerror = Rcmd - Rdist;
  sommeRerror +=  Rerror ;

  Lerror = Lcmd - Ldist;
  sommeLerror +=  Lerror ;

  //if (!(Rerror<0 || Lerror<0))
  if (Rdist+Ldist<Rdistacc+Ldistacc){
    vitesseR = ((double) (110 / Rdistacc)) * Rdist;
    vitesseL = ((double) (110 / Ldistacc)) * Ldist;
  }
  else if (inRange((Rdist+Ldist)/2,(Rdistacc+Ldistacc)/2,(Rcmd-Rdistdec+Lcmd-Ldistacc)/2)) {
    vitesseR = 110;
    vitesseL = 110;
  }
  else if (inRange((Rdist+Ldist)/2,((Rcmd-Rdist)+(Lcmd-Ldist))/2,(Rcmd+Lcmd)/2)){
    vitesseR=  ((double)(110 / Rcmd)) * Rerror;
    vitesseL = ((double)(110 / Lcmd)) * Lerror;
  }
  else{
    vitesseR = 0;
    vitesseL = 0;
  }

  if (!inRange(Rerror,margeNRerror,margePRerror))
    vitesseR = (kpR * Rerror) + (kdR * (Rerror - prevRerror)) + kiR * sommeRerror;
  if (!inRange(Lerror,margeNLerror,margePLerror))
    vitesseL = (kpL * Lerror) + (kdL * (Lerror - prevLerror)) + kiL * sommeLerror;

  vitesseR = (vitesseR > 180) ? 180 : vitesseR;
  vitesseL = (vitesseL > 180) ? 180 : vitesseL;

  vitesseR = ((vitesseR < 80) && (Rerror > 0)) ? 80 : vitesseR; // min 72
  vitesseL = ((vitesseL < 80) && (Lerror > 0)) ? 80 : vitesseL;// min 72
  vitesseR = ((vitesseR < 80) && (Rerror < 0)) ? -80 : vitesseR; // min 72
  vitesseL = ((vitesseL < 80) && (Lerror < 0)) ? -80 : vitesseL;// min 72

  
  /*if (Rdist>Rcmd){
       penteR=vitesseR / Rerror;
       vitesseR = -(penteR) * Rdist;
       if (Rerror==prevRerror && Rerror!=0)
          kvR+=0.001;
  }
  if (Ldist>Lcmd){
       penteL=vitesseL / Lerror;
       vitesseL = -(penteL) * Ldist;
       if (Rerror==prevRerror && Lerror!=0)
          kvL+=0.001;
  }*/
  PIDorientation();
  if (!inRange(Oerror, margeNOerror, margePOerror)){
      vitesseR -= vitesseO;
      vitesseL += vitesseO;
  }
   
  prevRerror = Rerror;
  prevLerror = Lerror;
}

void Go(float cmd){
  Rcmd=cmd;   //+10.3
  Lcmd=cmd;   //+10.2
  
  Rdistacc=Rcmd*0.15;
  Ldistacc=Lcmd*0.15;
  Rdistdec=Rcmd*0.30;
  Ldistdec=Lcmd*0.30;
  
  PIDposition();
  motorR(vitesseR);
  motorL(vitesseL);
  if ((prevRerror=!0) && (prevLerror!=0) && (prevLerror==Lerror) && (prevRerror==Rerror) && (millis>firstTime))
      Stop=true;
}
void TurnRight(float angle){
  Rcmd=-20.8*angle; 
  Lcmd=20.8*angle;
  
  Rdistacc=Rcmd*0.1;
  Ldistacc=Lcmd*0.1;
  Rdistdec=Rcmd*0.20;
  Ldistdec=Lcmd*0.20;
  
  PIDposition();
  motorR(-vitesseR);
  motorL(vitesseL);
  if ((prevRerror=!0) && (prevLerror!=0) && (prevLerror==Lerror) && (prevRerror==Rerror) && (millis>firstTime))
    Stop=true;
}
void TurnLeft(float angle){
  Rcmd=20.8*angle;
  Lcmd=-20.8*angle;

  Rdistacc=Rcmd*0.1;
  Ldistacc=Lcmd*0.1;
  Rdistdec=Rcmd*0.20;
  Ldistdec=Lcmd*0.20;
  
  PIDposition();
  motorR(vitesseR);
  motorL(-vitesseL);
  if ((prevRerror=!0) && (prevLerror!=0) && (prevLerror==Lerror) && (prevRerror==Rerror) && (millis>firstTime))
    Stop=true;
}

void setup() {
  Serial.begin(9600);

  //Command Encoders
  pinMode(outputAR, INPUT_PULLUP);
  pinMode(outputBR, INPUT_PULLUP);
  pinMode(outputAL, INPUT_PULLUP);
  pinMode(outputBL, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(outputAR), ISRtrackAR, RISING);
  attachInterrupt(digitalPinToInterrupt(outputAL), ISRtrackAL, RISING);
  attachInterrupt(digitalPinToInterrupt(outputBR), ISRtrackBR, RISING);
  attachInterrupt(digitalPinToInterrupt(outputBL), ISRtrackBL, RISING);

  //Command Motors
  pinMode(forwardR, OUTPUT);
  pinMode(backwardR, OUTPUT);
  pinMode(forwardL, OUTPUT);
  pinMode(backwardL, OUTPUT);
}


void loop() {
  firstTime=millis();
  Go(10);
  //if (Stop)
    //Go(-10);  
  //TurnRight(90);
  
  Serial.print("vitesseL= ");
  Serial.print(vitesseL);
  Serial.print(" \tvitesseR= ");
  Serial.print(vitesseR);
  Serial.print(" \tOerror= ");
  Serial.print(Oerror);

  Serial.print(" \tLerror= ");
  Serial.print(Lerror);
  Serial.print(" \tRerror= ");
  Serial.println(Rerror);
  
  /*Serial.print("Ldist= ");
  Serial.print(Ldist);
  Serial.print("\tRdist= ");
  Serial.print(Rdist);*/
  
  /*if (Rerror>0 && Lerror>0){
    motorR(82);
    motorL(92);
    }
    else
     if(Rerror>0){
        motorR(82);
        motorL(0);
      }
      else
        if (Lerror>0){
          motorR(0);
          motorL(92);
        }
        else{
          motorR(0);
          motorL(0);
        }*/
}
