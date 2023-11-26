#include <SimpleTimer.h>
#include <util/atomic.h>

//Variables temps
volatile double n = 0;
volatile double Te =15/1.0e3;
volatile double t0R=0, t0L=0,t0Rmarche=0,t0Lmarche=0,t0Rarret=0,t0Larret=0,t=0, dtRarret=0, dtLarret=0;
SimpleTimer timer;

//Variables stop
volatile boolean stopR = true, stopL = true;

//Variables encodeurs
#define outputAR 2
#define outputBR 3
#define outputAL 18
#define outputBL 19
volatile double xcmd=0;
volatile double ycmd=0;
volatile double anglecmd=0;
volatile double Oact = 0;
volatile double Oint = 0;
volatile double Rdist = 0;
volatile double Ldist = 0;
volatile double dist = 0;
volatile double dx = 0;
volatile double dy = 0;
volatile double x = 0;
volatile double y = 0;
volatile double Rdisti = 0;
volatile double Ldisti = 0;
volatile double prevRdist = 0;
volatile double prevLdist = 0;
volatile double prevRdisti = 0;
volatile double prevLdisti = 0;
volatile double prevdist = 0;
volatile double distUnitR = (double)PI * 40 / 600;
volatile double distUnitL = (double)PI * 40 / 1024;

//Variables moteurs
#define forwardR 8
#define backwardR 9
#define forwardL 10
#define backwardL 11
volatile double pwmRcmd = 0;
volatile double pwmLcmd = 0;
volatile double pwmOcmd = 0;
volatile double vitesseLact = 0;
volatile double vitesseRact = 0;
volatile double vitesseOact = 0;
volatile double vitesseact = 0;
volatile double vitesseLactFilter = 0;
volatile double vitesseRactFilter = 0;
volatile double prevVRact = 0;
volatile double prevVLact = 0;
volatile double prevRact = 0;
volatile double prevLact = 0;


//Variables erreurs/commandes PIDposition
volatile double Rdistacc = 0;
volatile double Ldistacc = 0;
volatile double Rdistdec = 0;
volatile double Ldistdec = 0;

volatile double Rcmd = 0;
volatile double Lcmd = 0;
volatile double Ocmd = 0;
volatile double cmd = 0;
volatile double Rerror = 0;
volatile double Lerror = 0;
volatile double sommeRerror = 0;
volatile double sommeLerror = 0;
volatile double prevRerror = 0;
volatile double prevLerror = 0;
int nbosciVR = 0;
int nbosciVL = 0;

//Variables erreurs/commandes PIDvitesse
volatile double PIDVR = 0;
volatile double PIDVL = 0;
volatile double vitesseRcmd = 0;
volatile double vitesseLcmd = 0;
volatile double prevvitesseRcmd = 0;
volatile double prevvitesseLcmd = 0;
volatile double vitesseOcmd = 0;
volatile double vitessecmd = 0;
volatile double VRerror = 0;
volatile double VLerror = 0;
volatile double prevVRerror = 0;
volatile double prevVLerror = 0;
volatile double sommeVRerror = 0;
volatile double sommeVLerror = 0;
volatile double dVRerror = 0;
volatile double dVLerror = 0;


//Variables PID orientation
volatile double pwmO = 0 ;
volatile double Oerror = 0;
volatile double prevOerror = 0;
volatile double sommeOerror = 0;
volatile double dOerror = 0;

bool inRange(volatile double val, volatile double minimum, volatile double maximum) {
  return ((minimum <= val) && (val <= maximum));
}
double sign (volatile double val){
  if (val>=0)
    return 1;
  else
    return -1;
}
//Encoder functions
void ISRtrackAR() {
  Rdisti += (digitalRead(outputBR) == LOW) ? distUnitR : -distUnitR;
}
void ISRtrackBR() {
  Rdisti += (digitalRead(outputAR) == LOW) ? -distUnitR : distUnitR;
}

void ISRtrackAL() {
  Ldisti += ((digitalRead(outputBL) == LOW)) ? distUnitL :  -distUnitL;
}

void ISRtrackBL() {
  Ldisti += ((digitalRead(outputAL) == LOW)) ? -distUnitL :  distUnitL;
}

//Motor functions
void motorR(volatile double vR) {
  if (vR > 0) {
    analogWrite(forwardR, vR);
    analogWrite(backwardR, 0);
  }
  else {
    analogWrite(forwardR, 0);
    analogWrite(backwardR, -vR);
  }
}
void motorL(volatile double vL) {
  if (vL > 0) {
    analogWrite(forwardL, vL);
    analogWrite(backwardL, 0);
  }
  else {
    analogWrite(forwardL, 0);
    analogWrite(backwardL, -vL);
  }
}
////////////////////////////////////////////////////////////////////////
// PID Position Parameters

#define kpR 1.9
#define kpL 1.9 //  5.43

#define kdR 0
#define kdL 0

#define kiR 0
#define kiL 0

//PID vitesse Parameters

#define kpVR 21 //21
#define kpVL 13.75 // 13.75 // 150

#define kdVR 11 //11
#define kdVL 3.7  //3.7

#define kiVR 0.0143 //0.0143  //0.012
#define kiVL 0.005065  //0.005065

//PID orientation Parameters

#define kpO 0    // 3
#define kdO 0     // 2.5
#define kiO 0 

//Marges d'erreurs

#define margePVRerror 0.8
#define margeNVRerror -0.8
#define margePVLerror 0.8
#define margeNVLerror -0.8

#define margePRerror 20
#define margeNRerror -20
#define margePLerror 20
#define margeNLerror -20

#define margePOerror 10
#define margeNOerror -10

//Vitesses/Acc max min
#define pwmRcmdmax 200
#define pwmRcmdmin 80
#define pwmLcmdmax 200
#define pwmLcmdmin 80

#define vitesseRcmdmax 6   // mm/s
#define vitesseRcmdmin 6  // mm/s  
#define vitesseLcmdmax 6   // mm/s
#define vitesseLcmdmin 6  // mm/s

#define ARcmdmax 3.7    // mm/s²
#define ALcmdmax 3.7    // mm/s²

int tolosciVR =5;
int tolosciVL=5;

//////////////////////////////////////////////////////
void PIDvitesseR() {
    vitesseRact = (double) (Rdist - prevRdist); // mm/s
    stopR=!(vitesseRact!=0 && inRange(Rdist,Rdist+margePRerror,Rcmd-margePRerror));
    VRerror=vitesseRcmd-vitesseRact;
    prevRdist=Rdist;

    prevVRerror=VRerror;
    if (sign(vitesseRact)==-sign(prevVRact))
       nbosciVR++;
     if (nbosciVR<tolosciVR){
      sommeVRerror += VRerror;
      prevVRerror = VRerror;
      PIDVR = (double) (kpVR * VRerror) + (kdVR * (VRerror - prevVRerror)) + kiVR * sommeVRerror;
      pwmRcmd = (pwmRcmd<-pwmRcmdmax) ? -pwmRcmdmax : PIDVR;
      pwmRcmd = (pwmRcmd>-pwmRcmdmin && pwmRcmd<0) ? -pwmRcmdmin : PIDVR;
      pwmRcmd = (pwmRcmd>0 && pwmRcmd<pwmRcmdmin) ? pwmRcmdmin : PIDVR;
      pwmRcmd = (pwmRcmd>pwmRcmdmax) ? pwmRcmdmax : PIDVR;
    }
        prevVRact = vitesseRact;
}

void PIDvitesseL() {
    vitesseLact = (double) (Ldist - prevLdist); // mm/s
    stopL=!(vitesseLact!=0 && inRange(Ldist,Ldist+margePLerror,Lcmd-margePLerror));
    prevLdist=Ldist;
    prevVLerror=VLerror;
    VLerror=vitesseLcmd-vitesseLact;
    if (sign(vitesseLact)==-sign(prevVLact))
       nbosciVL++;
     if (nbosciVL<tolosciVL){
      sommeVLerror += (double) VLerror;
      prevVLerror = VLerror;
      PIDVL = (double) (kpVL * VLerror) + (kdVL * (VLerror - prevVLerror)) + kiVL * sommeVLerror;
  
      pwmLcmd = (pwmLcmd<-pwmLcmdmax) ? -pwmLcmdmax : PIDVL;
      pwmLcmd = (pwmLcmd>-pwmLcmdmin && pwmLcmd<0) ? -pwmLcmdmin : PIDVL;
      pwmLcmd = (pwmLcmd>0 && pwmLcmd<pwmLcmdmin) ? pwmLcmdmin : PIDVL;
      pwmLcmd = (pwmLcmd>pwmLcmdmax) ? pwmLcmdmax : PIDVL;
    prevVLact = vitesseLact;
     }
}

void PIDorientation() {
  Oerror = -Ocmd + Oact;
  sommeOerror += (double) Oerror;
  prevOerror = Oerror;
  pwmOcmd = (double) (kpO * Oerror) + (kdO * (Oerror - prevOerror)) + kiO * sommeOerror;
  if (Ocmd>=0){
    pwmRcmd -= pwmOcmd;
    pwmLcmd += pwmOcmd;
  }
  else{
    pwmRcmd += pwmOcmd;
    pwmLcmd -= pwmOcmd;
  }
    pwmRcmd = (pwmRcmd<-pwmRcmdmax) ? -pwmRcmdmax : pwmRcmd;
    pwmRcmd = (pwmRcmd>-pwmRcmdmin && pwmRcmd<0) ? -pwmRcmdmin : pwmRcmd;
    pwmRcmd = (pwmRcmd>0 && pwmRcmd<pwmRcmdmin) ? pwmRcmdmin : pwmRcmd;
    pwmRcmd = (pwmRcmd>pwmRcmdmax) ? pwmRcmdmax : pwmRcmd;
    pwmLcmd = (pwmLcmd<-pwmLcmdmax) ? -pwmLcmdmax : pwmLcmd;
    pwmLcmd = (pwmLcmd>-pwmLcmdmin && pwmLcmd<0) ? -pwmLcmdmin : pwmLcmd;
    pwmLcmd = (pwmLcmd>0 && pwmLcmd<pwmLcmdmin) ? pwmLcmdmin : pwmLcmd;
    pwmLcmd = (pwmLcmd>pwmLcmdmax) ? pwmLcmdmax : pwmLcmd;
}

void PIDposition() {
   if (nbosciVR<tolosciVR && nbosciVL<tolosciVL ){
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      Rdist = Rdisti;
      Ldist = Ldisti;
    }
//PID Position Right
    prevvitesseRcmd=vitesseRcmd;
    prevRerror=Rerror;
    Rerror = Rcmd - Rdist;
    sommeRerror+=Rerror;
    vitesseRcmd=(double) (kpR*Rerror) + (kdR*(Rerror-prevRerror)) + (kiR*sommeRerror);
//Limite vitesseRcmd
    vitesseRcmd = (vitesseRcmd<-vitesseRcmdmax) ? -vitesseRcmdmax : vitesseRcmd;
    vitesseRcmd = (vitesseRcmd>-vitesseRcmdmin && vitesseRcmd<0) ? -vitesseRcmdmin : vitesseRcmd;
    vitesseRcmd = (vitesseRcmd>0 && vitesseRcmd<vitesseRcmdmin) ? vitesseRcmdmin : vitesseRcmd;
    vitesseRcmd = (vitesseRcmd>vitesseRcmdmax) ? vitesseRcmdmax : vitesseRcmd;
//Limite AccelerationR
    vitesseRcmd = (vitesseRcmd-prevvitesseRcmd<-ARcmdmax) ? prevvitesseRcmd-ARcmdmax : vitesseRcmd;
    vitesseRcmd = (vitesseRcmd-prevvitesseRcmd>ARcmdmax) ? prevvitesseRcmd+ARcmdmax : vitesseRcmd;
    PIDvitesseR();
//PID Position Left
    prevvitesseLcmd=vitesseLcmd;
    prevLerror=Lerror;
    Lerror = Lcmd - Ldist;
    sommeLerror+=Rerror;
    vitesseLcmd=(double) (kpL*Lerror) + (kdL*(Lerror-prevLerror)) + (kiL*sommeLerror);
//Limite vitesseLcmd 
    vitesseLcmd = (vitesseLcmd<-vitesseLcmdmax) ? -vitesseLcmdmax : vitesseLcmd;
    vitesseLcmd = (vitesseLcmd>-vitesseLcmdmin && vitesseLcmd<0) ? -vitesseLcmdmin : vitesseLcmd;
    vitesseLcmd = (vitesseLcmd>0 && vitesseLcmd<vitesseLcmdmin) ? vitesseLcmdmin : vitesseLcmd;
    vitesseLcmd = (vitesseLcmd>vitesseLcmdmax) ? vitesseLcmdmax : vitesseLcmd;
//Limite AccelerationL
    vitesseLcmd = (vitesseLcmd-prevvitesseLcmd<-ALcmdmax) ? prevvitesseLcmd-ALcmdmax : vitesseLcmd;
    vitesseLcmd = (vitesseLcmd-prevvitesseLcmd>ALcmdmax) ? prevvitesseLcmd+ALcmdmax : vitesseLcmd;
    PIDvitesseL();
//Odométrie
    dist=(double) ((Rdist+Ldist)/2);
    Oact= Oint+(double) (Rdist-Ldist);
    vitesseact=dist-prevdist;
    prevdist=dist;
    dx=-vitesseact*sin(Oact);
    dy=vitesseact*cos(Oact);
    x+=dx;
    y+=dy;

        PIDorientation();
   }
    else{
  motorR(0);
  motorL(0);
  pwmRcmd=0;
  pwmLcmd=0;
      delay(3000);
    }
//    Rcmd+=-sign(Rdist-prevRdist)*sqrt(dx*dx+dy*dy);
//    Lcmd+=-sign(Ldist-prevLdist)*sqrt(dx*dx+dy*dy);
}
void moveto(volatile double xconsigne,volatile double yconsigne,volatile double Oconsigne){
  xcmd=xconsigne;
  ycmd=yconsigne;
  Ocmd=Oconsigne;
  if (xcmd>=0 && ycmd>=0){
    Rcmd=sqrt(xcmd*xcmd+ycmd*ycmd);
    Lcmd=sqrt(xcmd*xcmd+ycmd*ycmd);
  }
  else if (xcmd<0 && ycmd<0){
    Rcmd=-sqrt(xcmd*xcmd+ycmd*ycmd);
    Lcmd=-sqrt(xcmd*xcmd+ycmd*ycmd);
  }
}
void Go0() {
  moveto(0,100,0);
  PIDposition();
  motorR(pwmRcmd);
  motorL(pwmLcmd);
}
void Go1() {
  moveto(0,0,Oact+190*PI);
  PIDposition();
  motorR(pwmRcmd);
  motorL(pwmLcmd);
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
 
  timer.setInterval(Te,Go0);
}
void loop() {
//  if (n==0){
//    timer.run();
//    if (inRange(Rerror,margeNRerror,margePRerror) && inRange(Lerror,margeNLerror,margeNLerror)){
//      n++;
//      if (n==1){
//        timer.disable(0);
//        timer.restartTimer(0);
//        timer.setInterval(Te,Go1);
//        timer.run();
//      }    
//  }
// }
timer.run();
  Serial.print("pwmLcmd= ");
  Serial.print(pwmLcmd);
  Serial.print(" vitesseLcmd= ");
  Serial.print(vitesseLcmd);
  Serial.print(" vitesseLact= ");
  Serial.print(vitesseLact);
  Serial.print(" VLerror= ");
  Serial.print(VLerror);
  Serial.print(" Lerror= ");
  Serial.print(Lerror);
  Serial.print(" nbosciVL= ");
  Serial.print(nbosciVL);

  
  Serial.print("\t\tpwmRcmd= ");
  Serial.print(pwmRcmd);
  Serial.print(" vitesseRcmd= ");
  Serial.print(vitesseRcmd);
  Serial.print(" vitesseRact= ");
  Serial.print(vitesseRact);
  Serial.print(" VRerror= ");
  Serial.print(VRerror);
  Serial.print(" Rerror= ");
  Serial.print(Rerror);
  Serial.print(" nbosciVR= ");
  Serial.print(nbosciVR);
  Serial.print(" Oerror= ");
  Serial.println(Oerror);

}
