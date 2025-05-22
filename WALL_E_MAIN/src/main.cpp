#include "Arduino.h"
#include <Adafruit_PWMServoDriver.h>
#include <Ps3Controller.h>
#include <Wire.h>
#define TX_PIN 17 // TX
#define RX_PIN 16 // RX
HardwareSerial mySerial(1);
///////////////////// ROUES //////////////////////////
const int in1 = 12;
const int in2 = 14;
const int conA = 13;
const int in3 = 27;
const int in4 = 26;
const int conB = 25;
#define PWM_CHANNEL_A 0
#define PWM_CHANNEL_B 1
//////////////////////////////////////////////////////

///////////////////// TETE ///////////////////////////
const int TOURNER_TETE = 12;   // PULSE MIN: 230  PULSE MAX: 470
const int LEVER_TETE = 10;     // PULSE MIN: 270  PULSE MAX: 370
const int SOURCIL_DROIT = 11;  // PULSE MIN: 460  PULSE MAX: 550 (fermé)
const int SOURCIL_GAUCHE = 13; // PULSE MIN: 340  PULSE MAX: 280 (fermé)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
//////////////////////////////////////////////////////

///////////////////// BRAS GAUCHE ////////////////////
const int EPAULE_X_GAUCHE = 15;    //  EPAULE INTERNE // PULSE MIN: 100 (HAUT)  PULSE MAX: 500 (Tout droit)  Diag: 290
const int EPAULE_Y_GAUCHE = 7;     // PULSE MIN:   PULSE MAX:
const int POIGNEE_GAUCHE = 6;      // PULSE MIN: 120 (PAUME VERS LE HAUT)  PULSE MAX: 540   MID : 340
const int PIVOTER_MAIN_GAUCHE = 8; // PULSE MIN: 150  PULSE MAX: 390 (Intérieur) TENDU : 280
const int BOUGER_DOIT_GAUCHE = 9;  // PULSE MIN: 360 (Ouvert)   PULSE MAX: 460 (Fermé)
//////////////////////////////////////////////////////

///////////////////// BRAS DROIT /////////////////////
const int EPAULE_X_DROIT = 14;    // EPAULE INTERNE // PULSE MIN: 80 (tout droit   PULSE MAX: 550 (HAUT) Diag : 330
const int EPAULE_Y_DROIT = 3;     // PULSE MIN:   PULSE MAX:
const int POIGNEE_DROIT = 4;      // PULSE MIN: 110   PULSE MAX:  520 (PAUME VERS LE HAUT) MID : 320
const int PIVOTER_MAIN_DROIT = 5; // PULSE MIN: 160 (Intérieur)  PULSE MAX: 400     TENDU : 300
//////////////////////////////////////////////////////
bool emoteEnCours = false;
bool emoteCoucou = false;
bool emoteGiveMe = false;
bool emoteRock = false;
bool emoteChokbar = false;
bool emoteDance = false;

int joystickToPulse(int valJoy, int joyMin, int joyMax, int pulseMin, int pulseMax)
{
  // Sert à convertir la valeur lue d'un joystick en une autre donnée exploitable
  int pulse = map(valJoy, joyMin, joyMax, pulseMin, pulseMax);
  return pulse;
}

int suppZoneMorte(int valJoy)
{
  // Sert à simplifier la zone morte d'un joystick pour éviter d'avoir des données résiduelles
  int val = 0;
  if (valJoy > 10 || valJoy < -10)
  {
    val = valJoy;
  }
  return val;
}

int appui(int valBouton)
{
  int val = 0;
  if (valBouton > 150)
  {
    val = 1;
  }
  return val;
}

int appuiCroix()
{
  return appui(Ps3.data.analog.button.cross);
}
int appuiCarre()
{
  return appui(Ps3.data.analog.button.square);
}
int appuiRond()
{
  return appui(Ps3.data.analog.button.circle);
}
int appuiTriangle()
{
  return appui(Ps3.data.analog.button.triangle);
}

int appuiUp()
{
  return appui(Ps3.data.analog.button.up);
}
int appuiDown()
{
  return appui(Ps3.data.analog.button.down);
}
int appuiLeft()
{
  return appui(Ps3.data.analog.button.left);
}
int appuiRight()
{
  return appui(Ps3.data.analog.button.right);
}

int appuiL1()
{
  return appui(Ps3.data.analog.button.l1);
}
int appuiL2()
{
  return appui(Ps3.data.analog.button.l2);
}
int appuiR1()
{
  return appui(Ps3.data.analog.button.r1);
}
int appuiR2()
{
  return appui(Ps3.data.analog.button.r2);
}
int appuiL3()
{
  return Ps3.data.button.l3;
}
int appuiR3()
{
  return Ps3.data.button.r3;
}
///////////////////// SONS ///////////////////////////
void ecrireSon(int nombre)
{
  mySerial.println(nombre);
}
//////////////////////////////////////////////////////

void gestionSourcilGauche(bool lever)
{
  int sourcilGauchePos = 290;
  if (lever)
  {
    sourcilGauchePos = 330;
  }
  pwm.setPWM(SOURCIL_GAUCHE, 0, sourcilGauchePos);
}

void gestionSourcilDroit(bool lever)
{
  int sourcilDroitPos = 500;
  if (lever)
  {
    sourcilDroitPos = 460;
  }
  pwm.setPWM(SOURCIL_DROIT, 0, sourcilDroitPos);
}

void gestionTete()
{
  static int oldJoyX = suppZoneMorte(Ps3.data.analog.stick.rx);
  static int oldJoyY = suppZoneMorte(Ps3.data.analog.stick.ry);
  int newJoyX = suppZoneMorte(Ps3.data.analog.stick.rx);
  int newJoyY = suppZoneMorte(Ps3.data.analog.stick.ry);
  static int TeteX = joystickToPulse(suppZoneMorte(Ps3.data.analog.stick.rx), -128, 127, 470, 230);
  static int TeteY = joystickToPulse(suppZoneMorte(Ps3.data.analog.stick.ry), -128, 127, 350, 270);
  if (abs(newJoyX - oldJoyX) > 0)
  {
    TeteX = joystickToPulse(suppZoneMorte(Ps3.data.analog.stick.rx), -128, 127, 470, 230);
  }
  if (abs(newJoyY - oldJoyY) > 0)
  {
    TeteY = joystickToPulse(suppZoneMorte(Ps3.data.analog.stick.ry), -128, 127, 350, 270);
  }
  static int xVal = 250;
  static int yVal = 320;
  xVal = (12 * xVal + TeteX) / 13;
  yVal = (10 * yVal + TeteY) / 11;
  pwm.setPWM(TOURNER_TETE, 0, xVal);
  pwm.setPWM(LEVER_TETE, 0, yVal);

  if (emoteChokbar == false && emoteRock == false)
  {
    if (appuiL1())
    {
      gestionSourcilGauche(true);
    }
    else if (emoteEnCours == false)
    {
      gestionSourcilGauche(false);
    }
    if (appuiR1())
    {
      gestionSourcilDroit(true);
    }
    else if (emoteEnCours == false)
    {
      gestionSourcilDroit(false);
    }
  }
}

void gestionRoues()
{
  int vitesseGauche;
  int vitesseDroite;
  int joyX = Ps3.data.analog.stick.lx;
  int joyY = Ps3.data.analog.stick.ly;
  if (appuiL2() == 0 && appuiR2()==0)
  {

    if (joyY < -10)
    {
      digitalWrite(in3, LOW); // Switch between this HIGH and LOW to change direction
      digitalWrite(in4, HIGH);
      digitalWrite(in1, HIGH); // Switch between this HIGH and LOW to change direction
      digitalWrite(in2, LOW);
      int vitesseMax = -joyY * 255 / 128;
      if (joyX < -10)
      {
        vitesseGauche = map(joyX, -128, 0, 0, vitesseMax);
        vitesseDroite = vitesseMax;
      }
      else if (joyX > 10)
      {
        vitesseDroite = map(joyX, 0, 127, vitesseMax, 0);
        vitesseGauche = vitesseMax;
      }
      else
      {
        vitesseDroite = vitesseMax;
        vitesseGauche = vitesseDroite;
      }
    }
    else if (joyY > 10)
    {
      digitalWrite(in3, HIGH); // Switch between this HIGH and LOW to change direction
      digitalWrite(in4, LOW);
      digitalWrite(in1, LOW); // Switch between this HIGH and LOW to change direction
      digitalWrite(in2, HIGH);
      vitesseGauche = map(joyY, 0, 127, 0, 255);
      vitesseDroite = vitesseGauche;
    }
    else
    {
      vitesseGauche = 0;
      vitesseDroite = vitesseGauche;
    }
  }
  else
  {
    if (appuiL2())
    {

      digitalWrite(in1, HIGH); // Switch between this HIGH and LOW to change direction
      digitalWrite(in2, LOW);
      digitalWrite(in3, HIGH); // Switch between this HIGH and LOW to change direction
      digitalWrite(in4, LOW);
      vitesseGauche = 255;
      vitesseDroite = vitesseGauche;
    }
    else if (appuiR2())
    {
      digitalWrite(in1, LOW); // Switch between this HIGH and LOW to change direction
      digitalWrite(in2, HIGH);
      digitalWrite(in3, LOW); // Switch between this HIGH and LOW to change direction
      digitalWrite(in4, HIGH);
      vitesseGauche = 255;
      vitesseDroite = vitesseGauche;
    }
    else
    {
      vitesseGauche = 0;
      vitesseDroite = vitesseGauche;
    }
  }
  ledcWrite(PWM_CHANNEL_A, vitesseDroite);
  ledcWrite(PWM_CHANNEL_B, vitesseGauche);
}

void fermerDoigt()
{
  // PULSE MIN: 360 (Ouvert)   PULSE MAX: 460 (Fermé)
  //smoothServo(BOUGER_DOIT_GAUCHE, 360, 460);
  pwm.setPWM(BOUGER_DOIT_GAUCHE, 0, 460);

}

void ouvrirDoigt()
{
  // PULSE MIN: 360 (Ouvert)   PULSE MAX: 460 (Fermé)
  //smoothServo(BOUGER_DOIT_GAUCHE, 460, 360);
  pwm.setPWM(BOUGER_DOIT_GAUCHE, 0, 360);
}

void switchMains()
{
  static int posGauche = 540; // On commence vers le bas
  static int posDroite = 110;
  static int etat = 0;
  static bool etatPrecedent = 0;
  if (appuiRight() && etatPrecedent == 0)
  {
    if (etat == 0)
    {
      posGauche = 340;
      posDroite = 320; // Vers le  mid
      etat = 1;
    }
    else if (etat == 1) {
      etat = 2;
      posGauche = 120;
      posDroite = 520; // Vers le haut
    }
    else
    {
      etat = 0;
      posGauche = 540;
      posDroite = 110; // Vers le bas
    }
  }
  pwm.setPWM(POIGNEE_DROIT, 0, posDroite);
  pwm.setPWM(POIGNEE_GAUCHE, 0, posGauche);
  etatPrecedent = appuiRight();

  if (appuiTriangle()){
    ouvrirDoigt();
    pwm.setPWM(PIVOTER_MAIN_GAUCHE, 0, 280);
  }
  else{
    fermerDoigt();
    pwm.setPWM(PIVOTER_MAIN_GAUCHE, 0, 390);
  }
}

void smoothServo(int pin, int oldPos, int posCible)
{
  static int posCibleSmooth = oldPos;
  posCibleSmooth = (10 * posCibleSmooth + posCible) / 11;
  pwm.setPWM(pin, 0, posCibleSmooth);
}

void faireEmoteCoucou()
{

  if (emoteCoucou)
  {
    static int oldTime = 0;
    int newTime = millis();
    static int step = 0;
    static int conditionTemps = 600;
    switch (step)
    {
    case 0:
      conditionTemps = 600;
      break;
    case 1:
      conditionTemps = 2000;
      pwm.setPWM(PIVOTER_MAIN_DROIT, 0, 400);
      pwm.setPWM(PIVOTER_MAIN_GAUCHE, 0, 150);
      pwm.setPWM(EPAULE_X_DROIT, 0, 330);
      pwm.setPWM(EPAULE_X_GAUCHE, 0, 290);
      break;
    case 2:
      conditionTemps = 500;
      pwm.setPWM(POIGNEE_GAUCHE, 0, 540);
      pwm.setPWM(POIGNEE_DROIT, 0, 110);
      // poigneeGauche.write(180);
      // poigneeDroit.write(0);
      break;
    case 3:
      pwm.setPWM(POIGNEE_GAUCHE, 0, 340);
      pwm.setPWM(POIGNEE_DROIT, 0, 320);
      conditionTemps = 500;
      // poigneeGauche.write(90);
      // poigneeDroit.write(90);
      break;
    case 4:
      conditionTemps = 500;
      pwm.setPWM(POIGNEE_GAUCHE, 0, 540);
      pwm.setPWM(POIGNEE_DROIT, 0, 110);
      // poigneeGauche.write(180);
      // poigneeDroit.write(0);
      break;
    case 5:
      pwm.setPWM(POIGNEE_GAUCHE, 0, 340);
      pwm.setPWM(POIGNEE_DROIT, 0, 320);
      // poigneeGauche.write(90);
      conditionTemps = 1000;
      // poigneeDroit.write(90);
    }
    if (newTime - oldTime > conditionTemps)
    {

      step++;
      oldTime = newTime;
    }
    if (step > 5)
    {
      step = 0;
      conditionTemps = 800;
      emoteCoucou = false;
      oldTime = 0;
    }
  }
}

void faireEmoteGiveMe()
{

  if (emoteGiveMe)
  {
    static int oldTime = 0;
    int newTime = millis();
    static int step = 0;
    static int conditionTemps = 1000;
    switch (step)
    {
    case 0:
      conditionTemps = 1000;
      break;
    case 1:
      conditionTemps = 1000;
      ecrireSon(1);
      // pwm.setPWM(POIGNEE_DROIT, 0, 520);
      // pwm.setPWM(PIVOTER_MAIN_DROIT, 0, 300);
      pwm.setPWM(POIGNEE_GAUCHE, 0, 120);
      pwm.setPWM(PIVOTER_MAIN_GAUCHE, 0, 280);
      break;
    case 2:
      conditionTemps = 500;
      ouvrirDoigt();
      //pwm.setPWM(PIVOTER_MAIN_DROIT, 0, 400);
      pwm.setPWM(PIVOTER_MAIN_GAUCHE, 0, 150);
      break;
    case 3:
      conditionTemps = 500;
      fermerDoigt();
      //pwm.setPWM(PIVOTER_MAIN_DROIT, 0, 300);
      pwm.setPWM(PIVOTER_MAIN_GAUCHE, 0, 280);
      break;
    case 4:
      //pwm.setPWM(PIVOTER_MAIN_DROIT, 0, 400);
      pwm.setPWM(PIVOTER_MAIN_GAUCHE, 0, 150);
      conditionTemps = 2000;
      ouvrirDoigt();
    }
    if (newTime - oldTime > conditionTemps)
    {
      step++;
      oldTime = newTime;
    }
    if (step > 4)
    {
      step = 0;
      conditionTemps = 1000;
      emoteGiveMe = false;
      oldTime = 0;
    }
  }
}

void faireEmoteDance()
{

  if (emoteDance)
  {
    static int oldTime = 0;
    int newTime = millis();
    static int step = 0;
    static int conditionTemps = 400;
    switch (step)
    {
    case 0:
      conditionTemps = 400;
      break;
    case 1:
      pwm.setPWM(POIGNEE_GAUCHE, 0, 340);
      pwm.setPWM(POIGNEE_DROIT, 0, 110);
      //poigneeDroit.write(90);
      //poigneeGauche.write(180);

      break;
    case 2:
      pwm.setPWM(POIGNEE_GAUCHE, 0, 540);
      pwm.setPWM(POIGNEE_DROIT, 0, 320);
      //poigneeDroit.write(0);
      //poigneeGauche.write(90);
      break;
    case 3:
      pwm.setPWM(POIGNEE_GAUCHE, 0, 340);
      pwm.setPWM(POIGNEE_DROIT, 0, 110);
      //poigneeDroit.write(90);
      //poigneeGauche.write(180);
      break;
    case 4:
      pwm.setPWM(POIGNEE_GAUCHE, 0, 540);
      pwm.setPWM(POIGNEE_DROIT, 0, 320);
      // poigneeDroit.write(0);
      // poigneeGauche.write(90);
      break;
    case 5:
      pwm.setPWM(POIGNEE_GAUCHE, 0, 340);
      pwm.setPWM(POIGNEE_DROIT, 0, 110);
      //poigneeDroit.write(90);
      //poigneeGauche.write(180);
    }
    if (newTime - oldTime > conditionTemps)
    {

      step++;
      oldTime = newTime;
    }
    if (step > 5)
    {
      step = 0;
      conditionTemps = 400;
      emoteDance = false;
      oldTime = 0;
    }
  }
}

void faireEmoteRock()
{
  static int oldTime = millis();
  if (emoteRock)
  {

    int newTime = millis();

    if (newTime - oldTime < 500)
    {
      ecrireSon(6);
    }
    else
    {
      ecrireSon(0);
    }
    if (newTime - oldTime > 700)
    {
      gestionSourcilGauche(1);
    }
    if (newTime - oldTime > 2700)
    {
      oldTime = newTime;
      emoteRock = false;
    }
  }
  else
  {
    oldTime = millis();
  }
}

void faireEmoteChokbar()
{
  static int oldTime = millis();
  if (emoteChokbar)
  {
    int newTime = millis();

    if (newTime - oldTime < 500)
    {
      ecrireSon(7);
    }
    else
    {
      ecrireSon(0);
    }
    if (newTime - oldTime > 700)
    {
      gestionSourcilDroit(1);
      gestionSourcilGauche(1);
    }
    if (newTime - oldTime > 2700)
    {
      oldTime = newTime;
      emoteChokbar = false;
    }
  }
  else
  {
    oldTime = millis();
  }
}

void sonRandom()
{
  static int son = random(9, 15);
  int newSon = random(9, 15);
  while (newSon == son)
  {
    newSon = random(9, 15);
  }
  ecrireSon(newSon);
  son = newSon;
}

void gestionEmotes()
{
  if (emoteCoucou == false && emoteGiveMe == false && emoteRock == false && emoteChokbar == false && emoteDance == false)
  {
    emoteEnCours = false;
  }
  else
  {
    emoteEnCours = true;
  }
  if (appuiRond() && emoteEnCours == false)
  {
    emoteCoucou = true;
  }
  if (appuiCarre() && emoteEnCours == false)
  {
    emoteDance = true;
  }
  if (appuiTriangle() && emoteEnCours == false)
  {
    emoteGiveMe = false;
  }
  if (appuiL2() && emoteEnCours == false)
  {
    emoteRock = false;
  }
  if (appuiUp() && emoteEnCours == false)
  {
    emoteChokbar = true;
  }
  if (appuiLeft())
  {
    ecrireSon(3);
  }
  faireEmoteCoucou();
  //faireEmoteGiveMe();
  //faireEmoteRock();
  faireEmoteChokbar();
  faireEmoteDance();
  if (emoteEnCours == false)
  {
    pwm.setPWM(PIVOTER_MAIN_DROIT, 0, 160);
    pwm.setPWM(EPAULE_X_DROIT, 0, 80);
    pwm.setPWM(EPAULE_X_GAUCHE, 0, 500);
    switchMains();
    if (appuiCroix())
    {
      sonRandom();
    }
  }
}

void gestionModeAuto(bool &modeAuto)
{
  static bool oldState = 0;
  bool state=appuiDown();
  if (state!=oldState && state==1 && modeAuto == false)
  {

    modeAuto = true;
    pwm.setPWM(LEVER_TETE, 0, 350);
    pwm.setPWM(LEVER_TETE, 0, 320);
  }
  else if (state!=oldState && state==1 && modeAuto == true)
  {
    modeAuto = false;
  }
  oldState = state;

  if (modeAuto)
  {

    static int oldTime = millis();
    static int oldTime2 = millis();
    int newTime = millis();
    int newTime2 = millis();
    static int conditionTemps = 3000;
    static int conditionTempsSon = 10000;
    static int oldPosX = 350;
    static int newPosX = random(230, 470);
    smoothServo(TOURNER_TETE, oldPosX, newPosX);
    if (newTime - oldTime > conditionTemps)
    {
      oldTime = newTime;
      oldPosX = newPosX;
      newPosX = random(230, 470);
      conditionTemps = random(2500, 5000);
    }
    if (newTime2 - oldTime2 > conditionTempsSon)
    {
      sonRandom();
      conditionTempsSon = random(20000, 25000);
      oldTime2 = newTime2;
    }
  }
}

void notify()
{
}

void onConnect()
{
  Serial.println("Connected.");
}

void calibrationServo()
{
  static int val = 300;
  static bool etatUPprecedent = appuiUp();
  bool etatUP = appuiUp();
  static bool etatDOWNprecedent = appuiDown();
  bool etatDOWN = appuiDown();
  if (etatUP != etatUPprecedent && etatUP == true)
  {
    val += 10;
  }
  if (etatDOWN != etatDOWNprecedent && etatDOWN == true)
  {
    val -= 10;
  }
  Serial.println(val);
  pwm.setPWM(EPAULE_X_GAUCHE, 0, val);
  etatUPprecedent = etatUP;
  etatDOWNprecedent = etatDOWN;
}

void setup()
{
  Serial.begin(9600);
  mySerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  ledcAttachPin(conA, PWM_CHANNEL_A);
  ledcAttachPin(conB, PWM_CHANNEL_B);
  ledcSetup(PWM_CHANNEL_A, 5000, 8);
  ledcSetup(PWM_CHANNEL_B, 5000, 8);
  Wire.begin(21, 22);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(60);
  randomSeed(analogRead(34));
  Ps3.attach(notify);
  Ps3.attachOnConnect(onConnect);
  Ps3.begin("00:00:00:00:00:00");
  Ps3.setPlayer(0);
  delay(300);
}

void loop()

{
  static bool modeAuto = false;
  if (modeAuto == false)
  {
    gestionTete();
    gestionRoues();
    gestionEmotes();
  }
  gestionModeAuto(modeAuto);
  delay(20);
  
  // pwm.setPWM(BOUGER_DOIT_GAUCHE,0,460);
  // delay(2000);
  // pwm.setPWM(BOUGER_DOIT_GAUCHE,0,360);
  // delay(2000);

// static int freq=360;
// Serial.println(freq);
// if(appuiUp()){
//   freq=freq+10;
// }
// if(appuiDown()){
//   freq=freq-10;
// }
// //pwm.setPWM(7, 0, freq);
// pwm.setPWM(BOUGER_DOIT_GAUCHE, 0, freq);
// delay(100);

}