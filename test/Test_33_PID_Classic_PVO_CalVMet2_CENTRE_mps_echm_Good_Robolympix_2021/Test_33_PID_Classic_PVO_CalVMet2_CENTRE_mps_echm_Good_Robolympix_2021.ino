
#include <SimpleTimer.h>
#include <util/atomic.h>

//Variables temps
volatile double Te = 0.1; //ms
volatile unsigned long t0Ri = 0, t0Li = 0, t1Ri = 0, t1Li = 0;
volatile double dtRi = 0, dtLi = 0;
volatile unsigned long t0R = 0, t0L = 0, t1R = 0, t1L = 0, t0 = 0, t1 = 0, t2 = 0;
volatile double prevdtL = 0, prevdtR = 0, dtR = 0, dtL = 0, dt = 0, mindtL = 88 / 1.0e6, mindtR = 68 / 1.0e6, mindt = 96 / 1.0e6, maxdtL = 31620 / 1.0e6, maxdtR = 39072 / 1.0e6, maxdt = 136884 / 1.0e6;
SimpleTimer timer;

//Variables stop
volatile boolean stopR = true, stopL = true;

//Variables encodeurs
#define outputAR 2
#define outputBR 3
#define outputAL 18
#define outputBL 19
volatile double Rdist = 0;
volatile double Ldist = 0;
volatile double dist = 0;
volatile double margePdist = 0.3;
volatile double margeNdist = -0.3;
volatile double margePerror = 0.6;
volatile double margeNerror = -0.6;
volatile double margePVerror = 1;
volatile double margeNVerror = -1;
volatile double margePOerror = 0.5;
volatile double margeNOerror = -0.5;
volatile double Rdisti = 0;
volatile double Ldisti = 0;
volatile double prevRdist = 0;
volatile double prevLdist = 0;
volatile double KR = (1024 * 2) / (PI * (8 / 1.0e2));
volatile double KL = (600 * 2) / (PI * (8 / 1.0e2));
volatile double distUnitR = (double)PI * 4 / 1024;
volatile double distUnitL = (double)PI * 4 / 600;

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
volatile double vitesseact = 0;
volatile double vitesseLactFilter = 0;
volatile double vitesseRactFilter = 0;
volatile double vitesseactFilter = 0;
volatile double prevVact = 0;
volatile double pwmRmax = 180;
volatile double pwmLmax = 180;
volatile double pwmRmin = 72;
volatile double pwmLmin = 72;


//Variables erreurs/commandes PIDposition
volatile double Rdistacc = 0;
volatile double Ldistacc = 0;
volatile double distacc = 0;
volatile double Rdistdec = 0;
volatile double Ldistdec = 0;
volatile double distdec = 0;
volatile double PIDP = 0;
volatile double Rcmd = 0;
volatile double Lcmd = 0;
volatile double cmd = 0;
volatile double error = 0;
volatile double preverror = 0;
volatile double sommeerror = 0;
volatile double derror = 0;

//Variables erreurs/commandes PIDvitesse
volatile double PIDV = 0;
volatile double vitesseRcmd = 0;
volatile double vitesseLcmd = 0;
volatile double vitesseOcmd = 0;
volatile double vitessecmd = 0;
volatile double vitesseRcmdmax = pwmRmax / 35;
volatile double vitesseRcmdmin = pwmRmin / 35;
volatile double vitesseLcmdmax = pwmLmax / 45;
volatile double vitesseLcmdmin = pwmLmin / 35;
volatile double Verror = 0;
volatile double prevVerror = 0;
volatile double sommeVerror = 0;
volatile double dVerror = 0;


//Variables PID orientation
volatile double pwmO = 0 ;
volatile double Oerror = 0;
volatile double prevOerror = 0;
volatile double sommeOerror = 0;
volatile double dOerror = 0;

bool inRange(volatile double val, volatile double minimum, volatile double maximum) {
  return ((minimum <= val) && (val <= maximum));
}

//Encoder functions
void ISRtrackAR() {
  Rdisti += (digitalRead(outputBR) == LOW) ? distUnitR : -distUnitR;
  t1Ri = micros();
  dtRi = ((double) (t1Ri - t0Ri) / 1.0e6);
  t0Ri = t1Ri;
}
void ISRtrackBR() {
  Rdisti += (digitalRead(outputAR) == LOW) ? -distUnitR : distUnitR;
  t1Ri = micros();
  dtRi = ((double) (t1Ri - t0Ri) / 1.0e6);
  t0Ri = t1Ri;
}

void ISRtrackAL() {
  Ldisti += ((digitalRead(outputBL) == LOW)) ? -distUnitL :  distUnitL;
  t1Li = micros();
  dtLi = ((double) (t1Li - t0Li) / 1.0e6);
  t0Li = t1Li;
}

void ISRtrackBL() {
  Ldisti += ((digitalRead(outputAL) == LOW)) ? distUnitL :  -distUnitL;
  t1Li = micros();
  dtLi = ((double) (t1Li - t0Li) / 1.0e6);
  t0Li = t1Li;
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

//PID vitesse Parameters


volatile double kpV = 0.1;        //153
volatile double kdV = 1.5;         //60
volatile double kiV = 0.000000001;      //0.001
//PID orientation Parameters

volatile double kpO = 2;
volatile double kdO = 8;
volatile double kiO = 0.000000001;


//2nd
//volatile double kpV = 0.0001;        //153
//volatile double kdV = 5;         //60
//volatile double kiV = 0.00000001;      //0.001
//volatile double kpO = 3;
//volatile double kdO = 5;
//volatile double kiO = 0.00000001;


///1st try
//volatile double kpO = 17;
//volatile double kdO = 33;
//volatile double kiO = 0.00000000001;

//Acceleration Parameters

float aR = 15; // cm / s²
float aL = 15; // cm / s²


void PIDvitesse() {
  if (!inRange(Verror, margeNVerror, margePVerror)) {
    sommeVerror += (double) (Verror / ((double)((dtR + dtL) / 2)));
    dVerror = (double) ((Verror - prevVerror) / ((double)((dtR + dtL) / 2)));
    prevVerror = Verror;
    PIDV = (double) (kpV * Verror) + (kdV * dVerror) + kiV * sommeVerror;
    vitesseRcmd = constrain(PIDV, vitesseRcmdmin, vitesseRcmdmax);
    vitesseLcmd = constrain(PIDV, vitesseLcmdmin, vitesseLcmdmax);
  }
  else {
    vitesseRcmd = 0;
    vitesseLcmd = 0;
  }
  vitessecmd = (vitesseRcmd + vitesseLcmd) / 2;
}

void PIDorientation() {
  Oerror = Rdist - Ldist;
  if (!inRange(error, margeNerror, margePerror) && !inRange(Oerror, margeNOerror, margePOerror)) {
    sommeOerror += (double) (Oerror / ((double)((dtR + dtL) / 2)));
    dOerror = (double) ((Oerror - prevOerror) / ((double)((dtR + dtL) / 2)));
    prevOerror = Oerror;
    vitesseOcmd = (double) (kpO * Oerror) + (kdO * dOerror) + kiO * sommeOerror;
  }
  else
    vitesseOcmd = 0;
  if (Oerror > margePOerror) {
    vitesseLcmd += vitesseOcmd;
    vitesseLcmd = constrain(vitesseLcmd, vitesseLcmdmin, vitesseLcmdmax);
  }
  else if (Oerror < margeNOerror) {
    vitesseRcmd += vitesseOcmd;
    vitesseRcmd = constrain(vitesseRcmd, vitesseRcmdmin, vitesseRcmdmax);
  }
  vitessecmd = (vitesseRcmd + vitesseLcmd) / 2;
}

void PIDposition() {
  prevLdist = Ldist;
  prevRdist = Rdist;
  prevVact = vitesseact;
  t1 = micros();
  dt = ((double) (t1 - t0)) / 1.0e6;
  if (!inRange(dt, mindtR, maxdt)) {
    dt = 0;
    vitesseRact = 0;
    vitesseLact = 0;
  }
  else {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      Rdist = Rdisti;
      Ldist = Ldisti;
      dtR = dtRi;
      dtL = dtLi;
    }
    dist = (Rdist + Ldist) / 2;
    error = cmd - dist;

    //Calcul vitesseRact
    if (!inRange(dtR, mindtR, maxdtR)) {
      dtR = 0;
      vitesseRact = 0;
    }
    else {
      vitesseRact = ((double) (((double) ((Rdist - prevRdist) / distUnitR)) / (dtR * KR / Te)) * 1.0e2); // m/s
    }
    //Calcul vitesseLact
    if (!inRange(dtL, mindtL, maxdtL)) {
      dtL = 0;
      vitesseLact = 0;
    }
    else {
      vitesseLact = ((double) (((double) ((Ldist - prevLdist) / distUnitL)) / (dtL * KL / Te)) * 1.0e2); // m/s
    }
    vitesseact = (vitesseRact + vitesseLact) / 2;

    vitesseactFilter = 0.854 * vitesseactFilter + 0.0728 * vitesseact + 0.0728 * prevVact;
    vitesseact = vitesseactFilter;
    vitessecmd = (vitesseRcmd + vitesseLcmd) / 2;
    Verror = vitessecmd - vitesseact;

    //Trapèze
    if (inRange(error, margeNerror, margePerror) && inRange(Oerror, margeNOerror, margePOerror)) {
      vitesseRcmd = 0;
      vitesseLcmd = 0;
    }
    //    else if (dist<distacc){
    //        vitesseRcmd = 10*Te;
    //        vitesseLcmd = 10*Te;
    //        if (vitesseact!=prevVact && (micros()/1.0e6)>0.2){
    //          if (stopL && stopR)
    //            t2=micros();
    //          else{
    //            vitesseRcmd = (double) aR *  ((double)((micros()-t2)/1.0e6));
    //            vitesseLcmd = (double) aL *  ((double)((micros()-t2)/1.0e6));
    //            stopR=false;
    //            stopL=false;
    //          }
    //        }
    //    }
    else if (dist < cmd - distdec) {
      vitesseRcmd = 20 * Te;
      vitesseLcmd = 20 * Te;
      stopR = false;
      stopL = false;
    }
    else if ((dist >= cmd - distdec) && (inRange(dtL, mindtL, maxdtL) || inRange(dtR, mindtR, maxdtR))) {
      sommeerror += (double) error / ((double)((dtR + dtL) / 2));
      derror = (double) ((error - preverror) / ((double)((dtR + dtL) / 2)));
      preverror = error;
      PIDP = (double) (kp * error) + (kd * derror) + ki * sommeerror;
      vitessecmd = constrain(PIDP, vitesseRcmdmin, vitesseRcmdmax);
      if (vitesseRcmd != 0 && vitesseLcmd != 0)
        PIDvitesse();
      stopR = false;   // remove this if you don't want PIDorientation in this part
      stopL = false;   // remove this if you don't want PIDorientation in this part
    }
    if (!stopR && !stopL)
      PIDorientation();
  }
  t0 = t1;
}

void Go(volatile double Leftcmd, volatile double Rightcmd) {
  Rcmd = Rightcmd;
  Lcmd = Leftcmd;
  cmd = (Rcmd + Lcmd) / 2;
  Rdistacc = Rcmd * 0.15;
  Ldistacc = Lcmd * 0.15;
  distacc = (Rdistacc + Ldistacc) / 2;
  Rdistdec = Rcmd * 0.40;
  Ldistdec = Lcmd * 0.40;
  distdec = (Rdistdec + Ldistdec) / 2;
  PIDposition();
  pwmR = vitesseRcmd * 35;
  pwmL = vitesseLcmd * 35;
  motorR(pwmR);
  motorL(pwmL);
}

void TurnRight(volatile double angle) {
  Rcmd = -20.8 * angle;
  Lcmd = 20.8 * angle;

  Rdistacc = Rcmd * 0.15;
  Ldistacc = Lcmd * 0.15;
  distacc = (Rdistacc + Ldistacc) / 2;
  Rdistdec = Rcmd * 0.35;
  Ldistdec = Lcmd * 0.35;
  distdec = (Rdistdec + Ldistdec) / 2;

  PIDposition();
  motorR(-pwmR);
  motorL(pwmL);
}
void TurnLeft(volatile double angle) {
  Rcmd = 20.8 * angle;
  Lcmd = -20.8 * angle;

  Rdistacc = Rcmd * 0.15;
  Ldistacc = Lcmd * 0.15;
  distacc = (Rdistacc + Ldistacc) / 2;
  Rdistdec = Rcmd * 0.25;
  Ldistdec = Lcmd * 0.25;
  distdec = (Rdistdec + Ldistdec) / 2;

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

  timer.setInterval(Te, PIDposition);
}
void loop() {
  timer.run();
  Go(5, 5);
  Serial.print("vitessecmd= ");
  Serial.print(vitessecmd);
  Serial.print(" \tvitesseact= ");
  Serial.print(vitesseact);
  Serial.print(" \tVerror= ");
  Serial.print(Verror);
  Serial.print(" \terror= ");
  Serial.print(error);
  Serial.print(" \tOerror= ");
  Serial.print(Oerror);
  Serial.print(" \tLdist= ");
  Serial.print(Ldist);
  Serial.print(" \tRdist= ");
  Serial.println(Rdist);

}
