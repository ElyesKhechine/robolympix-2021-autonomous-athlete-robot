#include <SimpleTimer.h>
#include <util/atomic.h>

//Variables temps
volatile double periode = 100/1.0e3; // 1000 => 9600 BAUDS
volatile unsigned long t0R=0,t0L=0,t0=0,t1R=0,t1L=0,t1=0;
volatile double dtR=0,dtL=0,dt=0;
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
volatile double dist = 0;
volatile double Rdisti = 0;
volatile double Ldisti = 0;
volatile double prevRdist = 0;
volatile double prevLdist = 0;
volatile double prevdist = 0;
volatile double prevRdisti = 0;
volatile double prevLdisti = 0;
volatile double distUnitR = (double)PI * 4 / 1024;
volatile double distUnitL = (double)PI * 4 / 600; 
volatile double nb_imp_par_periode_R=0;
volatile double nb_imp_par_periode_L=0;

//Variables moteurs
#define forwardR 6
#define backwardR 7
#define forwardL 5
#define backwardL 4
volatile double pwmR = 0 ;
volatile double pwmL = 0 ;
volatile double vitesseLact = 0;
volatile double vitesseRact = 0;
volatile double vitesseOact = 0;
volatile double vitesseact = 0;
volatile double vitesseLacti = 0;
volatile double vitesseRacti = 0;
volatile double vitesseLactFilter = 0;
volatile double vitesseRactFilter = 0;
volatile double prevVRact = 0;
volatile double prevVLact = 0;
volatile double prevVact = 0;
volatile double pwmRmax = 180;
volatile double pwmLmax = 180;
volatile double pwmmax = 180;
volatile double pwmRmin = 80;
volatile double pwmLmin = 80;
volatile double pwmmin = 80;

//Variables erreurs/commandes PIDposition
volatile double Rdistacc=0;
volatile double Ldistacc=0;
volatile double distacc=0;
volatile double Rdistdec=0;
volatile double Ldistdec=0;
volatile double distdec=0;
volatile double PIDR=0;
volatile double PIDL=0;
volatile double PID=0;
volatile double Rcmd = 0;
volatile double Lcmd = 0; 
volatile double cmd = 0; 
volatile double Rerror;
volatile double Lerror;
volatile double error;
volatile double prevRerror = 0;
volatile double prevLerror = 0;
volatile double preverror = 0;
volatile double sommeRerror = 0;
volatile double sommeLerror = 0;
volatile double sommeerror = 0;
volatile double dRerror=0;
volatile double dLerror=0;
volatile double derror=0;

//Variables erreurs/commandes PIDvitesse
volatile double PIDVR=0;
volatile double PIDVL=0;
volatile double PIDV=0;
volatile double vitesseRcmd=0;
volatile double vitesseLcmd=0;
volatile double vitessecmd=0;
volatile double VLerror = 0;
volatile double VRerror = 0;
volatile double Verror = 0;
volatile double prevVLerror = 0;
volatile double prevVRerror = 0;
volatile double prevVerror = 0;
volatile double sommeVLerror = 0;
volatile double sommeVRerror = 0;
volatile double sommeVerror = 0;
volatile double dVRerror=0;
volatile double dVLerror=0;
volatile double dVerror=0;


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
  t1R=micros();
  dtR=((double) (t1R-t0R))/1.0e6;
  if (dtR==0)
    vitesseRacti=0;
  else{
    vitesseRacti= (Rdisti-prevRdisti) / distUnitR / dtR;
      // CONVERT nb_imp_par_sec => motor_wheel_rotation/min (RPM)
    vitesseRacti=vitesseRacti/((double) ((1024/2)*(6.7/8.0)))*60.0; // RPMR
    vitesseRactFilter=0.854*vitesseRactFilter+0.0728*vitesseRacti+0.0728*prevVRact;
  }
  prevVRact=vitesseRacti;
  prevRdisti=Rdisti;
  t0R=t1R;
}
void ISRtrackBR() {
  Rdisti += (digitalRead(outputAR) == LOW) ? -distUnitR : distUnitR;
  t1R=micros();
  dtR=((double) (t1R-t0R))/1.0e6;
  if (dtR==0)
    vitesseRacti=0;
  else{
    vitesseRacti= (Rdisti-prevRdisti) / distUnitR / dtR;
      // CONVERT nb_imp_par_sec => motor_wheel_rotation/min (RPM)
    vitesseRacti=vitesseRacti/((double) ((1024/2)*(6.7/8.0)))*60.0; // RPMR
    vitesseRactFilter=0.854*vitesseRactFilter+0.0728*vitesseRacti+0.0728*prevVRact;
  }
  prevVRact=vitesseRacti;
  prevRdisti=Rdisti;
  t0R=t1R;
}

void ISRtrackAL() {
  Ldisti += (digitalRead(outputBL) == LOW) ? distUnitL : -distUnitL;
  t1L=micros();
  dtL=((double) (t1L-t0L))/1.0e6;
  if (dtL==0)
    vitesseLacti=0;
  else{
    vitesseLacti= (Ldisti-prevLdisti) / distUnitL / dtL;
    // CONVERT nb_imp_par_sec => motor_wheel_rotation/min (RPM)
    vitesseLacti=vitesseLacti/((double) ((600/2)*(6.7/8.0)))*60.0; // RPML
    vitesseLactFilter=0.854*vitesseLactFilter+0.0728*vitesseLacti+0.0728*prevVLact;
  }
  prevVLact=vitesseLacti;
  prevLdisti=Ldisti;
  t0L=t1L;
}

void ISRtrackBL() {
  Ldisti += (digitalRead(outputAL) == LOW) ? -distUnitL : distUnitL;
    t1L=micros();
  dtL=((double) (t1L-t0L))/1.0e6;
  if (dtL==0)
    vitesseLacti=0;
  else{
    vitesseLacti= (Ldisti-prevLdisti) / distUnitL / dtL;
    // CONVERT nb_imp_par_sec => motor_wheel_rotation/min (RPM)
    vitesseLacti=vitesseLacti/((double) ((600/2)*(6.7/8.0)))*60.0; // RPML
    vitesseLactFilter=0.854*vitesseLactFilter+0.0728*vitesseLacti+0.0728*prevVLact;
  }
  prevVLact=vitesseLacti;
  prevLdisti=Ldisti;
  t0L=t1L;
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

volatile double kp = 0;

volatile double kd = 0;

volatile double ki = 0;


//PID vitese Parameters

volatile double kpV = 0;

volatile double kdV = 0; 

volatile double kiV = 0; 


//PID orientation Parameters

volatile double kpO = 0;
volatile double kdO = 0;
volatile double kiO = 0;

float aR=800000; // cm / s²
float aL=800000; // cm / s²

void PIDvitesse() {
    /*if (vitessecmd-vitesseact>pwmmax)
      pwm=pwmmax;
    else if(vitessecmd-vitesseact<pwmmin)
      pwm=pwmmin;*/
    sommeVerror+=(double) Verror / (2 * dt);
    dVerror=(double) (vitesseact - prevVact) / dt;
    prevVerror = Verror;
    PIDV =(double) (kpV * Verror) + (kdV * dVerror) + kiV * sommeVerror;
    pwmR= constrain(PIDV,pwmRmin,pwmRmax);
    pwmL= constrain(PIDV,pwmLmin,pwmLmax);
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
  t1=micros();
  dt=((double) (t1-t0))/1.0e6;
  
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      Rdist=Rdisti;
      Ldist=Ldisti;
      vitesseRact=vitesseRacti;
      vitesseLact=vitesseLacti;
    }
  dist=(Rdist+Ldist)/2;
  cmd=(Rcmd+Lcmd)/2;
  error=cmd-dist;
  vitesseRcmd=((double) (Rcmd*60) / ((double)(2*PI*(6.7/2/100)))); // RPMR
  vitesseLcmd=((double) ((Lcmd/100)*60) / ((double)(2*PI*(6.7/2/100))));  // RPM
  vitessecmd=(vitesseRcmd+vitesseLcmd)/2;
  vitesseact=(vitesseRact+vitesseLact)/2;
  Verror=vitessecmd-vitesseact;
  
    //Trapèze Right
  if (inRange(error,-0.1,0.1)){
      pwmR=0;
      pwmL=0;
  }
  /*else if (dist<distacc){
      pwmR = (double) aR *  ((double)(dt/1.0e6));  //pwmR = (double) 2 * ( 100 / Rdistacc) * Rdist;
      pwmL = (double) aL *  ((double)(dt/1.0e6));  //pwmL = (double) 2 * ( 100 / Ldistacc) * Ldist;
      //PIDvitesse();
      stopR=false;
      stopL=false;
  }*/
  else if(inRange(dist,-0.5,cmd-distdec)) {
    pwmR = 100;
    pwmL=0;
    stopR=false;
    stopL=false;
  }
  else if (dist>cmd-distdec){
          sommeerror+=(double) error / (2 * dt);
          preverror = error;
          PID = (double) (kp * error) - (kd * vitesseact) + ki * sommeerror;
          pwmR= constrain(PID,pwmRmin,pwmRmax);
          pwmL= constrain(PID,pwmLmin,pwmLmax);
          PIDvitesse();
          stopR=false;    // remove this if you don't want PIDorientation in this part
          stopL=false;     // remove this if you don't want PIDorientation in this part
  }
  if (!stopR && !stopL)
      PIDorientation();
  prevdist=dist;
  t0=t1;
}

void Go(volatile double cmd){
  Rcmd=cmd;   //+10.3
  Lcmd=cmd;   //+10.2

  distacc=cmd*0.1;
  distdec=cmd*0.15;
  
  PIDposition();
  motorR(0);
  motorL(pwmL);
}

void TurnRight(volatile double angle){
  Rcmd=-20.8*angle; 
  Lcmd=20.8*angle;

  distacc=cmd*0.1;
  distdec=cmd*0.15;
  
  PIDposition();
  motorR(-pwmR);
  motorL(pwmL);
}
void TurnLeft(volatile double angle){
  Rcmd=20.8*angle;
  Lcmd=-20.8*angle;

  distacc=cmd*0.1;
  distdec=cmd*0.15;
  
  PIDposition();
  motorR(pwmR);
  motorL(-pwmL);
}

void setup() {
  Serial.begin(115200);

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
  Go(10);
 
  Serial.print("pwmL= ");
  Serial.print(pwmL);
  Serial.print(" \tvitesseLcmd= ");
  Serial.print(vitesseLcmd);
  Serial.print(" \tvitesseLact= ");
  Serial.print(vitesseLact);
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
