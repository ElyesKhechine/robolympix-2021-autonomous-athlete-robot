#include <AutoPID.h>

//Variables encodeurs
#define outputAR 2
#define outputBR 3
#define outputAL 18
#define outputBL 19
 double  Rdist = 0;
 double  Ldist = 0;
 double distUnitR = (double)PI * 40 / 1024 / 10;
 double distUnitL = (double)PI * 40 / 600 / 10;

double Rdistacc=0;
double Ldistacc=0;
double Rdistdec=0;
double Ldistdec=0;

 unsigned long firstTime;
 boolean Stop=false;

//Variables moteurs
 double vitesseR = 0 ;
 double vitesseL = 0 ;
 double vitesseO = 0 ;
#define forwardR 6
#define backwardR 7
#define forwardL 5
#define backwardL 4

//Variables erreurs/commandes position
double Rcmd = 0;
double Lcmd = 0; 
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

double kpR = 10;
double kpL = 10;

double kdR = 0;
double kdL = 0;

double kiR = 0;
double kiL = 0; 

double kpO = 10;
double kdO = 1; 
double kiO = 1;


void PIDorientation() {
  Oerror = Rdist - Ldist;
  sommeOerror += Oerror;
  vitesseO = (kpO * Oerror) + (kdO * (Oerror - prevOerror)) + kiO * sommeOerror;
  prevOerror = Oerror;
}

void PIDposition() {



  //if (!(Rerror<0 || Lerror<0))
      PIDorientation();

  vitesseR = (kpR * Rerror) + (kdR * (Rerror - prevRerror)) + kiR * sommeRerror;
  vitesseL = (kpL * Lerror) + (kdL * (Lerror - prevLerror)) + kiL * sommeLerror;

  vitesseR = (vitesseR > 180) ? 180 : vitesseR;
  vitesseL = (vitesseL > 180) ? 180 : vitesseL;

  if (Rdist+Ldist<Rdistacc+Ldistacc){
       vitesseR = vitesseR / Rdistacc * Rdist;
       vitesseL = vitesseL / Ldistacc * Ldist;
  }
  else if (Rdist+Ldist>Rdistdec+Ldistdec){
       vitesseR = vitesseR / Rdistdec * Rdist; 
       vitesseL = vitesseL / Ldistdec * Ldist;
  }
  
  vitesseR = ((vitesseR < 90) && (Rerror > 0)) ? 90 : vitesseR; // min 72
  vitesseL = ((vitesseL < 90) && (Lerror > 0)) ? 90 : vitesseL;// min 72

  
  /*if (Rdist>Rcmd){
       vitesseR = -(vitesseR / Rerror) * Rdist;
       if (Rerror==prevRerror && Rerror!=0)
          kvR+=0.001;
  }
  if (Ldist>Lcmd){
       vitesseL = -(vitesseL / Lerror) * Ldist;
       if (Rerror==prevRerror && Lerror!=0)
          kvL+=0.001;
  }*/
  
  if (Oerror!=0){
      vitesseR -= vitesseO;
      vitesseL += vitesseO;
  }
   
  prevRerror = Rerror;
  prevLerror = Lerror;
}

void Go(double cmd){
  Rcmd=cmd;   //+10.3
  Lcmd=cmd;   //+10.2
  
  /*Rdistacc=Rcmd*0.1;
  Ldistacc=Lcmd*0.1;
  Rdistdec=Rcmd*0.15;
  Ldistdec=Lcmd*0.15;*/
  
  /*AutoPID(double *Rdist, double *Rcmd, double *vitesseR, double 80, double 180, double KpR, double KiR, double KdR);
  AutoPID(double *Ldist, double *Lcmd, double *vitesseL, double 80, double 180, double KpL, double KiL, double KdL);*/
  Oerror = Rdist - Ldist;
  AutoPID(&Ldist, &Rdist,  &vitesseO,  80,  180,  kpO,  kiO,  kdO);
  run()

      vitesseR -= vitesseO;
      vitesseL += vitesseO;
  motorR(vitesseR);
  motorL(vitesseL);
}
void TurnRight(double angle){
  Rcmd=-20.8*angle; 
  Lcmd=20.8*angle;
  
  Rdistacc=Rcmd*0.1;
  Ldistacc=Lcmd*0.1;
  Rdistdec=Rcmd*0.20;
  Ldistdec=Lcmd*0.20;
  
  PIDposition();
  motorR(-vitesseR);
  motorL(vitesseL);
}
void TurnLeft(double angle){
  Rcmd=20.8*angle;
  Lcmd=-20.8*angle;

  Rdistacc=Rcmd*0.1;
  Ldistacc=Lcmd*0.1;
  Rdistdec=Rcmd*0.20;
  Ldistdec=Lcmd*0.20;
  
  PIDposition();
  motorR(vitesseR);
  motorL(-vitesseL);
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
  
  Rerror = Rcmd - Rdist;
  Lerror = Lcmd - Ldist;

  
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
   Serial.print("\tkpL= ");
  Serial.print(kpO);
  Serial.print("\tkdL= ");
  Serial.print(kdO);
  Serial.print("\tkiL= ");
  Serial.println(kiO);
  

 
  /*Serial.print("Rerror= ");
  Serial.print(Rerror);
  Serial.print("\tkpR= ");
  Serial.print(kpR);
  Serial.print("\tkdR= ");
  Serial.print(kdR);
  Serial.print("\tkiR= ");
  Serial.print(kiR);
  
  Serial.print("\tLerror= ");
  Serial.print(Lerror);
  Serial.print("\tkpL= ");
  Serial.print(kpL);
  Serial.print("\tkdL= ");
  Serial.print(kdL);
  Serial.print("\tkiL= ");
  Serial.println(kiL);*/
  

  
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
