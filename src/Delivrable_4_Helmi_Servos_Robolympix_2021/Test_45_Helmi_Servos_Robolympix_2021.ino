#include <Servo.h>
Servo servo_drapeau;
Servo servo_statu;
Servo servo_shoot; 
Servo servo_bras_1;
Servo servo_bras_2;

#define pin_servo_drapeau 5
#define pin_servo_statu 26
#define pin_servo_shoot 28
#define pin_servo_bras_1 24
#define pin_servo_bras_2 22


boolean bras_front_disque = false;
boolean bras_front_goal = false;
boolean shoot = false;
boolean laisse  = false;

 
long now;
void setup() {
//Serial.begin(9600);
//servo_drapeau.attach(pin_servo_drapeau);
//servo_statu.attach(pin_servo_statu);
//servo_shoot.attach(pin_servo_shoot);
//servo_bras_1.attach(pin_servo_bras_1);
//servo_bras_2.attach(pin_servo_bras_2);


//servo_shoot.write(180);

 now  = millis();
  if (now > 1000) {
    servo_drapeau.attach(pin_servo_drapeau);
    servo_drapeau.write(90);
    
  }

}
void loop() {
 

//  if ( laisse == true) {
//    servo_statu.attach(pin_servo_statu);
//    laisse = false;
//  }
//
//  if ( shoot == true ) {
//  servo_shoot.attach(pin_servo_shoot);
//  shoot = false ;
//  }
//
//  if ( bras_front_disque == true ) {
//    
//    for( int pos1=0 ; pos1 <= 80; pos1 ++) {
//      servo_bras_1.write(pos1);
//      delay(20);
//    }
//    
//    for( int pos2=0 ; pos2 <= 80; pos2 ++) {
//      servo_bras_2.write(pos2);
//      delay(20);
//    }
//    bras_front_disque = false; 
//  }
//
//  if ( bras_front_goal == true ) {
//    
//    for( int pos1=80 ; pos1 >= 0; pos1 --) {
//      servo_bras_1.write(pos1);
//      delay(20);
//    }
//    
//    for( int pos2=80 ; pos2 >= 0; pos2 --) {
//      servo_bras_2.write(pos2);
//      delay(20);
//    }
//  bras_front_goal = false;
//  }
//  
}
