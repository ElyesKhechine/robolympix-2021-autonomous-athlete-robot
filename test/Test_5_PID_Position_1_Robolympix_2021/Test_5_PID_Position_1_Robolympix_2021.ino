#include<SimpleTimer.h>

SimpleTimer timer;
int frequence = 20;


// motors

int forwardR = 1;//pin?
int backwardR =2;//pin?
int forwardL = 5;//pin?
int backwardL = 4;//pin?


//encoders
int encoderAL = 1;//pin?
int encoderBL = 2;//pin?
int encoderAR = 3;//pin?
int encoderBR = 4;//pin?

// etat encoders
int etatAR = 0;
int etatBR = 0;
int etatAL = 0;
int etatBL = 0;

float distanceR = 0;
float distanceL = 0;
float distUnit = (float)PI * 40 / 1024 / 10 ; // Rayon?

//Erreurs
float erreurR = 0;
float erreurL = 0;
float erreurPrecedantR = 0;
float erreurPrecedantL = 0;
float sommeErreurR = 0;
float sommeErreurL = 0;

//PID
float k = 0.3; // a modifier
float kpR = 1; // a modifier
float kiR = 0; // a modifier
float kdR = 0; // a modifier
float kpL = 1; // a modifier
float kiL = 0; // a modifier
float kdL = 0; // a modifier
float targetDistance=0;


float vitesseR = 0 ;
float vitesseL = 0 ;


//***************functions**************** 
void changeAR() {
  etatAR = digitalRead(2);//pinAR
  distanceR += (etatAR == etatBR) ? distUnit : (- distUnit);  
}

void changeAL() {
  etatAL = digitalRead(3);//pinAR
  distanceR += (etatAL == etatBL) ? distUnit : (- distUnit);  
}

void changeBR() {
  etatBR = digitalRead(4);//pinAR
  distanceR += (etatBR != etatAR) ? distUnit : (- distUnit);  
}

void changeBL() {
  etatBR = digitalRead(5);//pinAR
  distanceR += (etatBL != etatAL) ? distUnit : (- distUnit);  
}

void tournerR() {
  forwardR = (vitesseR >= 0) ? vitesseR : 0;
  backwardR = (vitesseR >= 0) ? 0 : -vitesseR;
}

void tournerL() {
 forwardL = (vitesseL >= 0) ? vitesseL : 0;
 backwardL = (vitesseL >= 0) ? 0 : -vitesseL;
}

// asservissement
void asservissement() {
  erreurR = targetDistance - distanceR ;
  sommeErreurR =  erreurR ;
  
  erreurL = targetDistance - distanceL ;
  sommeErreurL +=  erreurL ;

  vitesseR = (kpR * erreurR) + (kdR * (erreurR - erreurPrecedantR)) + kiR * sommeErreurR;
  vitesseL = (kpL * erreurL) + (kdL * (erreurL - erreurPrecedantL)) + kiL * sommeErreurL;

  vitesseR = (vitesseR > 255) ? vitesseR * k *distanceR : vitesseR;
  vitesseL = (vitesseL > 255) ? vitesseL * k *distanceL : vitesseL;

  erreurPrecedantR = erreurR;
  erreurPrecedantL = erreurL; 


  tournerR();
  tournerL();
}




//***************SETUP**************** 

void setup() {
  
  Serial.begin(9600);
  pinMode(forwardR,OUTPUT);
  pinMode(backwardR, OUTPUT);
  pinMode(forwardL, OUTPUT);
  pinMode(backwardL, OUTPUT);
  
  pinMode(encoderAL, INPUT);
  pinMode(encoderBL, INPUT);
  pinMode(encoderAR, INPUT);
  pinMode(encoderBR, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderAR),changeAR , CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderBR),changeBR , CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderAL),changeAL , CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderBL),changeBL , CHANGE);

  timer.setInterval(frequence , asservissement);

}

//**************LOOP**************

void loop() {
  
}
