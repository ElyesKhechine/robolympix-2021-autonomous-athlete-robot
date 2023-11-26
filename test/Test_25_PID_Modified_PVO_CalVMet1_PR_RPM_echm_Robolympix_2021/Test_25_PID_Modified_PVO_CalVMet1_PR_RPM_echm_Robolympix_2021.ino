
#include <SimpleTimer.h>
#include <util/atomic.h>

//Variables temps
volatile double periode = 0.5/1.0e3; // 1000 => 9600 BAUDS
volatile unsigned long t0Ri=0,t0Li=0,t1Ri=0,t1Li=0;
volatile double dtRi=0,dtLi=0;
volatile unsigned long t0R=0,t0L=0,t1R=0,t1L=0,t0=0,t1=0;
volatile double prevdtL=0,prevdtR=0,dtR=0,dtL=0,dt=0;
SimpleTimer timer;

//Variables stop
volatile boolean stopR=true, stopL=true;

//Variables encodeurs
#define outputAR 2
#define outputBR 3
#define outputAL 18
#define outputBL 19
volatile double Rdist = 0;
volatile double Ldist = 0;
volatile double Rdisti = 0;
volatile double Ldisti = 0;
volatile double prevRdist = 0;
volatile double prevLdist = 0;
volatile double prevRdisti = 0;
volatile double prevLdisti = 0;
volatile double distUnitR = (double)PI * 4 / 1024;
volatile double distUnitL = (double)PI * 4 / 600; 
volatile double nb_imp_par_periode_R=0;
volatile double nb_imp_par_periode_L=0;

//Variables moteurs
#define forwardR 8
#define backwardR 9
#define forwardL 10
#define backwardL 11
volatile double pwmR = 0 ;
volatile double pwmL = 0 ;
volatile double vitesseLact = 0;
volatile double vitesseRact = 0;
volatile double vitesseOact = 0;
volatile double vitesseLacti = 0;
volatile double vitesseRacti = 0;
volatile double vitesseLactFilter = 0;
volatile double vitesseRactFilter = 0;
volatile double prevVRact = 0;
volatile double prevVLact = 0;
volatile double pwmRmax = 180;
volatile double pwmLmax = 180;
volatile double pwmRmin = 80;
volatile double pwmLmin = 80;


//Variables erreurs/commandes PIDposition
volatile double Rdistacc=0;
volatile double Ldistacc=0;
volatile double Rdistdec=0;
volatile double Ldistdec=0;
volatile double PIDR=0;
volatile double PIDL=0;
volatile double Rcmd = 0;
volatile double Lcmd = 0; 
volatile double Rerror;
volatile double Lerror;
volatile double prevRerror = 0;
volatile double prevLerror = 0;
volatile double sommeRerror = 0;
volatile double sommeLerror = 0;
volatile double dRerror=0;
volatile double dLerror=0;

//Variables erreurs/commandes PIDvitesse
volatile double PIDVR=0;
volatile double PIDVL=0;
volatile double vitesseRcmd=0;
volatile double vitesseLcmd=0;
volatile double VLerror = 0;
volatile double VRerror = 0;
volatile double prevVLerror = 0;
volatile double prevVRerror = 0;
volatile double sommeVLerror = 0;
volatile double sommeVRerror = 0;
volatile double dVRerror=0;
volatile double dVLerror=0;


//Variables PID orientation
volatile double pwmO = 0 ;
volatile double Oerror=0;
volatile double prevOerror = 0;
volatile double sommeOerror = 0;
volatile double dOerror=0;


bool inRange(volatile double val, volatile double minimum, volatile double maximum){
  return ((minimum <= val) && (val <= maximum));
}

//Encoder functions
void ISRtrackAR() {
  Rdisti += (digitalRead(outputBR) == LOW) ? distUnitR : -distUnitR;
}
void ISRtrackBR() {
  Rdisti += (digitalRead(outputAR) == LOW) ? -distUnitR : distUnitR;
}

void ISRtrackAL() {
  Ldisti += ((digitalRead(outputBL) == LOW)) ? -distUnitL :  distUnitL;
}

void ISRtrackBL() {
  Ldisti += ((digitalRead(outputAL) == LOW)) ? distUnitL :  -distUnitL;
}

//Motor functions
void motorR(volatile double vR) {
  if (vR > 0) {
    analogWrite(forwardR, vR);
    analogWrite(backwardR, 0);
  }
  else {
    analogWrite(forwardR, 0);
    analogWrite(backwardR, -vR); //5.2
  }
}

void motorL(volatile double vL) {
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

volatile double kpR = 0;
volatile double kpL = 0;

volatile double kdR = 0;
volatile double kdL = 0;

volatile double kiR = 0;
volatile double kiL = 0;


//PID vitese Parameters

volatile double kpVR = 0;
volatile double kpVL = 0;

volatile double kdVR = 0; 
volatile double kdVL = 0; 

volatile double kiVR = 0; 
volatile double kiVL = 0;

//PID orientation Parameters

volatile double kpO = 0;
volatile double kdO = 0;
volatile double kiO = 0;

float aR=800000; // cm / s²
float aL=800000; // cm / s²


void PIDvitesseR() {
    /*if (vitesseRcmd-vitesseRact>pwmRmax)
      pwmR=pwmRmax;
    else if(vitesseRcmd-vitesseRact<pwmRmin)
      pwmR=pwmRmin;*/
    sommeVRerror+=(double) VRerror / (2 * dt);
    dVLerror=(double) (vitesseRact - prevVRact) / dt;
    prevVRerror = VRerror;
    PIDVR = (double) (kpVR * VRerror) + (kdVR * dVRerror) + kiVR * sommeVRerror; 
    pwmR= constrain(PIDVR,pwmRmin,pwmRmax);
}

void PIDvitesseL() {
    /*if (vitesseLcmd-vitesseLact>pwmLmax)
      pwmL=pwmLmax;
    else if(vitesseLcmd-vitesseLact<pwmLmin)
      pwmL=pwmLmin;*/
    sommeVLerror+=(double) VLerror / (2 * dt);
    dVLerror=(double) (vitesseLact - prevVLact) / dt;
    prevVLerror = VLerror;
    PIDVL =(double) (kpVL * VLerror) + (kdVL * dVLerror) + kiVL * sommeVLerror;
    pwmL= constrain(PIDVL,pwmLmin,pwmLmax);
}

void PIDorientation() {
  Oerror = Rdist - Ldist;
  vitesseOact=vitesseRact-vitesseLact;
  if (dt>=periode && !inRange(Oerror,-0.2,0.2)){
      sommeOerror+=(double) Oerror / (2 * dt);
      prevOerror = Oerror;
      pwmO =(double) (kpO * Oerror) - (kdO * vitesseOact) + kiO * sommeOerror;
      if (Oerror>0)
        pwmL+=pwmO;
      else if(Oerror<0)
        pwmR+=pwmO;
      pwmR=constrain(pwmR,pwmRmin,pwmRmax);
      pwmL=constrain(pwmL,pwmLmin,pwmLmax);
    }
}

void PIDposition(){
  prevLdist=Ldist;
  prevRdist=Rdist;
  prevVLact=vitesseLact;
  prevVRact=vitesseRact;
  t1=micros();
  dt=((double) (t1-t0))/1.0e6;
  if (dt>=periode){
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
        Rdist=Rdisti;
    }
    Rerror = Rcmd - Rdist;
    if (prevRdist==Rdist)
      vitesseRact=0;
    else{
      vitesseRact= (Rdist-prevRdist) / distUnitR / dt;
      // CONVERT nb_imp_par_sec => motor_wheel_rotation/min (RPM)
      vitesseRact=vitesseRact/((double) ((1024/2)*(6.7/8.0)))*60.0; // RPML
      vitesseRactFilter=0.854*vitesseRactFilter+0.0728*vitesseRact+0.0728*prevVRact;
    }
    vitesseRcmd=((double) ((Rcmd/100)*60) / ((double)(2*PI*(6.7/2/100)))); // RPMR
    VRerror=vitesseRcmd-vitesseRact;
  
      //Trapèze Right
    if (inRange(Rerror,-0.2,0.2))
        pwmR=0;
    /*else if (Rdist<Rdistacc){
        pwmR = (double) aR *  ((double)(dt/1.0e6));  //pwmR = (double) 2 * ( 100 / Rdistacc) * Rdist;
              Serial.print("vLi= ");
        Serial.println(pwmR);
        //PIDvitesseR();
        stopR=false;
    }*/
    else if(inRange(Rdist,-0.5,Rcmd-Rdistdec)) {
      pwmR = 100;
      stopR=false;
    }
    else if (Rdist>Rcmd-Rdistdec){
            sommeRerror+=(double) Rerror / (2 * dt);
            prevRerror = Rerror;
            PIDR = (double) (kpR * Rerror) - (kdR * vitesseRact) + kiR * sommeRerror;
            pwmR= constrain(PIDR,pwmRmin,pwmRmax);
            PIDvitesseR();
            stopR=false;    // remove this if you don't want PIDorientation in this part
       }
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
        Ldist=Ldisti;
      }
    Lerror = Lcmd - Ldist;
    if (prevLdist==Ldist)
      vitesseLact=0;
    else{
      vitesseLact= (Ldist-prevLdist) / distUnitL / dt;
      // CONVERT nb_imp_par_sec => motor_wheel_rotation/min (RPM)
      vitesseLact=vitesseLact/((double) ((600/2)*(6.7/8.0)))*60.0; // RPML
      vitesseLactFilter=0.854*vitesseLactFilter+0.0728*vitesseLact+0.0728*prevVLact;
    }
    vitesseLcmd=((double) ((Lcmd/100)*60) / ((double)(2*PI*(6.7/2/100))));  // RPM
    VLerror=vitesseLcmd-vitesseLact;
  
      //Trapèze Left
    if (inRange(Lerror,-0.2,0.2))
        pwmL=0;
    /*else if (Ldist<Ldistacc){
        pwmR = (double) aL *  ((double)(dt/1.0e6)); //pwmL = (double) 2 * (100 / Ldistacc) * Ldist;
        Serial.print("vRi= ");
        Serial.println(pwmR);
        //PIDvitesseL();
        stopL=false;
    }*/
    else if(inRange(Ldist,-0.5,Lcmd-Ldistdec)) {
      pwmL = 100;
      stopL=false;
    }
    else if (Ldist>Lcmd-Ldistdec){
          sommeLerror+=(double) Lerror / (2 * dt);
          prevLerror = Lerror;
          PIDL = (double) (kpL * Lerror) - (kdL * vitesseLact) + kiL * sommeLerror;
          pwmL= constrain(PIDL,pwmLmin,pwmLmax);
          PIDvitesseL();
          stopL=false;     // remove this if you don't want PIDorientation in this part
    }
    if (!stopR && !stopL)
        PIDorientation();
  }
  t0=t1;
}

void Go(volatile double cmd){
  Rcmd=cmd;   //+10.3
  Lcmd=cmd;   //+10.2

  Rdistacc=Rcmd*0.1;
  Ldistacc=Lcmd*0.1;
  Rdistdec=Rcmd*0.15;
  Ldistdec=Lcmd*0.15;
  
  PIDposition();
  motorR(0);
  motorL(pwmL);
}

void TurnRight(volatile double angle){
  Rcmd=-20.8*angle; 
  Lcmd=20.8*angle;
  
  Rdistacc=Rcmd*0.1;
  Ldistacc=Lcmd*0.1;
  Rdistdec=Rcmd*0.15;
  Ldistdec=Lcmd*0.15;
  
  PIDposition();
  motorR(-pwmR);
  motorL(pwmL);
}
void TurnLeft(volatile double angle){
  Rcmd=20.8*angle;
  Lcmd=-20.8*angle;

  Rdistacc=Rcmd*0.1;
  Ldistacc=Lcmd*0.1;
  Rdistdec=Rcmd*0.15;
  Ldistdec=Lcmd*0.15;
  
  PIDposition();
  motorR(pwmR);
  motorL(-pwmL);
}

void setup() {
  Serial.begin(250000);

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

  timer.setInterval(periode,PIDposition);
}

void loop() {
  timer.run();
  Go(0);
 
//  Serial.print("pwmL= ");
//  Serial.print(pwmL);
//  Serial.print(" \tvitesseLcmd= ");
//  Serial.print(vitesseLcmd);
  Serial.print("vitesseLact= ");
  Serial.print(vitesseLact);
  Serial.print(" \tLdist= ");
  Serial.print(Ldist);
  Serial.print(" \tprevLdist= ");
  Serial.print(prevLdist);
  Serial.print(" \tVLerror= ");
  Serial.print(VLerror);
  Serial.print(" \tLerror= ");
  Serial.println(Lerror);
//    Serial.print(" \tVRerror= ");
//  Serial.print(VRerror);
//  Serial.print(" \tRerror= ");
//  Serial.println(Rerror);
  /*Serial.print(" \tRdist= ");
  Serial.println(Rdist);
  Serial.print(" \tRerror= ");
  Serial.print(Rerror);*/
  
}
