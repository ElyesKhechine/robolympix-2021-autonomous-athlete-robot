//Variables encodeurs
#define outputAR 2
#define outputBR 3
#define outputAL 18
#define outputBL 19
volatile unsigned int temp=20;
volatile float Rdist=0.0;
volatile float Ldist=0.0;
volatile float distUnitR=360*40/(1024*4);
volatile float distUnitL=360*40/(600*4);

//Variables moteurs
float vitesseR = 0 ;
float vitesseL = 0 ;
#define forwardR 4 ;
#define backwardR 5 ;
#define forwardL 6 ;
#define backwardL 7 ;

//Variables erreurs/commandes
volatile float Rcmd=500;
volatile float Lcmd=500;
volatile float Rerror;
volatile float Lerror;
float prevRerror = 0;
float prevLerror = 0;
float sommeRerror = 0;
float sommeLerror = 0;

//PID Position Parameters
float k = 0.3 ;
float kpR = 1 ; 
float kiR = 0 ; 
float kdR = 0 ;
float kpL = 1 ;
float kiL = 0 ;
float kdL = 0 ;

//Encoder functions
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

//Motor functions
void turnR() {
  forwardR = (vitesseR >= 0) ? vitesseR : 0;
  backwardR = (vitesseR >= 0) ? 0 : -vitesseR;
}

void turnL() {
 forwardL = (vitesseL >= 0) ? vitesseL : 0;
 backwardL = (vitesseL >= 0) ? 0 : -vitesseL;
}

//PID Position + Asservissement Vitesse
void asservissement() {
  //PID Position
  Rerror = Rcmd - Rdist ;
  sommeRerror =  Rerror ;
  
  Lerror = Lcmd - Ldist ;
  sommeLerror +=  Lerror ;

  vitesseR = (kpR * Rerror) + (kdR * (Rerror - prevRerror)) + kiR * sommeRerror;
  vitesseL = (kpL * Lerror) + (kdL * (Lerror - prevLerror)) + kiL * sommeLerror;

  //Asservissement Vitesse
  vitesseR = (vitesseR > 255) ? vitesseR * k *Rdist : vitesseR;
  vitesseL = (vitesseL > 255) ? vitesseL * k *Ldist : vitesseL;

  //Update errors
  prevRerror = Rerror;
  prevLerror = Lerror; 

  turnR();
  turnL();
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
  asservissement();
  if (Rdist/distUnitR!=temp){
    Serial.print("Rdist= ");
    Serial.print(Rdist);
    Serial.print("\tRerror= ");
    Serial.print(Rerror);
    temp=Rdist/distUnitR;
  }
  if (Ldist/distUnitL!=temp){
    Serial.print("\tLdist= ");
    Serial.print(Ldist);
    Serial.print("\tLerror= ");
    Serial.println(Lerror);
    temp=Ldist/distUnitL;
  }
}
