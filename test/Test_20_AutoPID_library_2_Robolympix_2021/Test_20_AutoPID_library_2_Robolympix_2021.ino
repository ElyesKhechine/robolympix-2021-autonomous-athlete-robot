#include <AutoPID.h>

//Variables encodeurs
#define outputAR 2
#define outputBR 3
#define outputAL 18
#define outputBL 19
 double  Rdist = 0;
 double  Ldist = 0;
 double prevRdist= 0;
 double prevLdist= 0;
 double distUnitR = (double)PI * 40 / 1024 / 10;
 double distUnitL = (double)PI * 40 / 600 / 10;

unsigned long lastVitesseRUpdate;
unsigned long lastVitesseLUpdate;

//Variables moteurs
 double vitesseR = 0 ;
 double vitesseL = 0 ;
 double vitesseO = 0 ;
#define forwardR 6
#define backwardR 7
#define forwardL 5
#define backwardL 4

//Variables erreurs/commandes position
double Rcmd = 10;
double Lcmd = 10; 
 double Rerror;
 double Lerror;
 double prevRerror = 0;
 double prevLerror = 0;
 double sommeRerror = 0;
 double sommeLerror = 0;

//Variables erreurs vitesse
 double Oerror;
 double prevOerror = 0;
 double sommeOerror = 0;

//Encoder functions
void ISRtrackAR() {
  Rdist += (digitalRead(outputBR) == LOW) ? distUnitR : -distUnitR;
  prevRdist=Rdist;
}
void ISRtrackBR() {
  Rdist += (digitalRead(outputAR) == LOW) ? -distUnitR : distUnitR;
    prevRdist=Rdist;
}

void ISRtrackAL() {
  Ldist += (digitalRead(outputBL) == LOW) ? distUnitL : -distUnitL;
    prevLdist=Ldist;
}

void ISRtrackBL() {
  Ldist += (digitalRead(outputAL) == LOW) ? -distUnitL : distUnitL;
    prevLdist=Ldist;
}

//Motor functions
void motorR( double vR) {
  if (vR > 0) {
    analogWrite(forwardR, vR);
    analogWrite(backwardR, 0);
  }
  else {
    analogWrite(forwardR, 0);
    analogWrite(backwardR, -vR); //5.2
  }
}

void motorL( double vL) {
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

double kpR =15.12;
double kpL =15.12;

double kdR =1 ;
double kdL =1;

double kiR =.0003;
double kiL =.0003;

double kpO =.12;
double kdO =0;
double kiO =.0003;



//AutoPID myPID(&Rdist, &Ldist, &vitesseO, 0, 180, kpO, kdO, kiO);
AutoPID myPIDR(&Rcmd, &Rdist, &vitesseR, -100, 180, kpR, kdR, kiR);
AutoPID myPIDL(&Ldist, &Ldist, &vitesseL, -100, 180, kpL, kdL, kiL);

bool updateVitesseR() {
  if ((millis() - lastVitesseRUpdate) > 10) {
    lastVitesseRUpdate = millis();
      Rerror=Rcmd-Rdist;
      sommeRerror+=Rerror;
      vitesseR = (kpR * Rerror) + (kdR * (Rerror - prevRerror)) + kiR * sommeRerror;
     /*vitesseR -= vitesseO;
      vitesseL += vitesseO;*/
      prevRerror=Rerror;
    return true;
  }
  return false;
}
bool updateVitesseL() {
  if ((millis() - lastVitesseLUpdate) > 10) {
    lastVitesseLUpdate = millis();
      Lerror=Lcmd-Ldist;
      sommeLerror+=Lerror;
      vitesseL = (kpL * Lerror) + (kdL * (Lerror - prevLerror)) + kiL * sommeLerror;
     /*vitesseR -= vitesseO;
      vitesseL += vitesseO;*/
      prevLerror=Lerror;
    return true;
  }
  return false;
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

  while (!updateVitesseR()) {}
  while (!updateVitesseL()) {}

  /*myPIDR.setBangBang(0.5);
  myPIDL.setBangBang(0.5);
  myPIDR.setTimeStep(100);
  myPIDL.setTimeStep(100);*/

}

void loop() {
  //updateVitesseR();
  updateVitesseL();
  //myPIDR.run();
  myPIDL.run();
  vitesseR = (((vitesseR < 90) && vitesseR>0)  && (Rerror > 0)) ? 90 : vitesseR; // min 72
  vitesseL = (((vitesseL < 90) && vitesseL>0) && (Lerror > 0)) ? 90 : vitesseL;// min 72

  vitesseR = (((vitesseR > -90) && vitesseR<0) && (Rerror < 0)) ? -90 : vitesseR; // min 72
  vitesseL = (((vitesseR > -90) && vitesseR<0) && (Lerror < 0)) ? -90 : vitesseL;// min 72

  vitesseR = (vitesseR > 180) ? 180 : vitesseR;  
  vitesseL = (vitesseL > 180) ? 180 : vitesseL;

  vitesseR = (vitesseR < -180) ? -180 : vitesseR;  
  vitesseL = (vitesseL < -180) ? -180 : vitesseL;

  //motorR(vitesseR);
  motorL(vitesseL);
  
  Serial.print("vitesseL= ");
  Serial.print(vitesseL);
  Serial.print(" \tLdist= ");
  Serial.print(Ldist);
  Serial.print(" \tLerror= ");
  Serial.print(Lerror);
  Serial.print("\tkpL= ");
  Serial.print(kpL);
  Serial.print("\tkdL= ");
  Serial.print(kdL);
  Serial.print("\tkiL= ");
  Serial.println(kiL);
}
