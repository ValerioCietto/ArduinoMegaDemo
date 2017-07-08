/* Sketck  RISICO_BIKE_SIMULATORE  ----  VER.15 del : 13-febbraio-2017  */
/* Software - by IK1IFQ Gabriele -  */
//----------------------------------------------------------------------------------
#include <Wire.h>
#include <Keypad.h>
#include <EEPROM.h>
#include "PinChangeInt.h"

#include <FastIO.h>
#include <I2CIO.h>
#include <LCD.h>
#include <LiquidCrystal.h>
#include <LiquidCrystal_I2C.h>
#include <LiquidCrystal_SR.h>
#include <LiquidCrystal_SR2W.h>
#include <LiquidCrystal_SR3W.h>

//-----------------------------------------------------------------------------------
#if ! ( defined __AVR_ATmega2560__ || defined __AVR_ATmega1280__ || defined __AVR_ATmega1281__ || defined __AVR_ATmega2561__ || defined __AVR_ATmega640__ )
#error "This sketch only works on chips in the ATmega2560 family."
#endif

#define I2C_ADDR      0x27 // I2C address of PCF8574A
#define BACKLIGHT_PIN 3
#define En_pin        2
#define Rw_pin        1
#define Rs_pin        0
#define D4_pin        4
#define D5_pin        5
#define D6_pin        6
#define D7_pin        7

LiquidCrystal_I2C lcd(I2C_ADDR,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin, BACKLIGHT_PIN, POSITIVE);
//LiquidCrystal_I2C lcd (0x20,2,1,0,4,5,6,7,3,POSITIVE);  // I2C address of PCF8574A (LCM1602 IIC A0 A1 A2) 
//LiquidCrystal_I2C lcd (0x27,2,1,0,4,5,6,7,3,POSITIVE);  // I2C address of PCF8574A  
//--------------------------------------------------------------------------------------------

#define TOTAL_PINS 69

#define encoder_X_Pin1 53   // input > Encoder 
#define encoder_X_Pin2 52
#define encoder_Y_Pin1 51
#define encoder_Y_Pin2 50
#define encoder_Z_Pin1 49
#define encoder_Z_Pin2 48
#define encoder_W_Pin1 47
#define encoder_W_Pin2 46

volatile uint8_t latest_interrupted_pin;
volatile uint8_t interrupt_count[TOTAL_PINS]={0}; // possible arduino pins
//volatile uint8_t pin3Count=0;
 
void quicfunc() {
  latest_interrupted_pin=PCintPort::arduinoPin;
  interrupt_count[latest_interrupted_pin]++;
};

/* // È possibile assegnare qualsiasi numero di funzioni a diversi pin. Quant'è fico?
void pin3func() {
  pin3Count++;
}
*/
//  ------------------- dichiara  Sub_Routin -----------------
void SuB_Clock();
void SuB_Display();
void SuB_Keypad_Code();
void SuB_Asse_X();
void SuB_Asse_Y();
void SuB_Asse_Z();
void SuB_Asse_W();
void SuB_Encoder_X();
void SuB_Encoder_Y();
void SuB_Encoder_Z();
void SuB_Encoder_W();
void SuB_Brake();
void updateEncoder_X();
void updateEncoder_Y();
void updateEncoder_Z();
void updateEncoder_W();
void home_axis();
void home_X();

//-------------------- clock lampeggio -----------------------------------------------------

#define LED 13          //LED connesso al pin digitale 13
#define interval 500  //Intervallo di tempo per  far lampeggiare il LED(millisecondi)
  int valueLed = LOW;    //Valore precedente del LED
   long previousMills = 0;  //Memorizza l'ultima volta che il led è stato aggiornato 

   int Clear=0;  int t=0;   int r=0;
   int clock_lcd=0;   int clock_lcd_A=0;  long clock_A=0;    long time_A=1000;
 
//-------------------Variabili Tempi & conteggi -------------------------------------------------------
   
   static int iSpeed_Asse_X = 0;
   static int iSpeed_Asse_Y = 0;
   static int iSpeed_Asse_Z = 0;
   static int iSpeed_Asse_W = 0;
   static int iAcc_Asse_X = 0;
   static int iAcc_Asse_Y = 0;
   static int iAcc_Asse_Z = 0;
   static int iAcc_Asse_W = 0;
   
   int PWS_B = 25 ;
   int PWS_Brake;
 
//------------------------------------------------------------------------------------------------------
//----- Variabili globali Menu --------------
char menu = 0x01;
char set1 = 0x00, set2 = 0x00 , set3 = 0x00;
//----------------------------------------------

int menuOpz     = 0x00;
int menuAsse_X  = 0x00;
int menuAsse_Y  = 0x00;
int Key_Reset   = 0x00;
int Key_Set     = 0x00;
int Key_A_Up    = 0x00;  
int Key_B_Down  = 0x00;
int Key_C_On    = 0x00;
int Key_D_Off   = 0x00;
int Key_Asse    = 0x00;
int Key_Val     = 0x00;
int Key         = 0;

//------ Variabili globali KeyPad     -------------------------------------
const byte ROWS = 4; //quattro righe
const byte COLS = 4; //quattro colonne
char valore[4];
char keyInsert[4];
//---------- Queste variabili verifica  PassWord ---------------------
int i = 0; int j = 0; int s = 0; int x = 0; int b ;
int R_X=0; int R_Y=0; int R_Z=0; int R_W=0;
int A_X=0; int A_Y=0; int A_Z=0; int A_W=0;

char code[5]= "5555";   // PassWord 
//---------------------------------------------------------------------
char Keys[ROWS][COLS]=  // creo la matrice dei tasti della tastiera.
{
{'1','2','3','A'} ,
{'4','5','6','B'},
{'7','8','9','C'},
{'*','0','#','D'}
};
byte colPins[4] = {30,32,34,36}; // Pin a cui sono connesse le colonne
byte rowPins[4] = {22,24,26,28}; // Pin a cui sono connesse le righe
 
Keypad keypad = Keypad( makeKeymap(Keys), rowPins, colPins, ROWS, COLS); 

//-------------------------------------// Mappatura in/out ------------------------

 byte StartMotor=44;
 byte Rele_Asse_X = 23;    byte Rele_Asse_Y = 25;     
 byte Rele_Asse_Z = 27;    byte Rele_Asse_W = 29;
  
 int SwU_X = 33;   int SwD_X = 31;  // switch Asse X 
 int SwU_Y = 37;   int SwD_Y = 35;  // switch Asse Y
 int SwU_Z = 41;   int SwD_Z = 39;  // switch Asse Z
 int SwU_W = 45;   int SwD_W = 43;  // switch Asse W
 
 int pinPwm_Asse_X = 3;   // PWM 
 int pinDir_Asse_X = 2;   // DIR 
 int pinPwm_Asse_Y = 5;   // PWM
 int pinDir_Asse_Y = 4;   // DIR 
 int pinPwm_Asse_Z = 7;   // PWM 
 int pinDir_Asse_Z = 6;   // DIR 
 int pinPwm_Asse_W = 9;   // PWM
 int pinDir_Asse_W = 8;   // DIR 
 
//------------------------------------// mappatura PWM Brake---------------------------------------------
 int ENA = 10;
 int IN1 = 11; 
 int IN2 = 12; 
//------------------------------------// mappatura Variabili Encoder -----------------------------------------------   

long Zero = 0;
int lastMSB = 0;
int lastLSB = 0;

volatile int lastEncoded_X = 0;
volatile int lastEncoded_Y = 0;
volatile int lastEncoded_Z = 0;
volatile int lastEncoded_W = 0;

volatile long encoderValue_X;
volatile long encoderValue_Y;
volatile long encoderValue_Z;
volatile long encoderValue_W;

long lastencoderValue_X = 0;
long lastencoderValue_Y = 0;
long lastencoderValue_Z = 0;
long lastencoderValue_W = 0;

long encoder_X ;
long encoder_Y ;
long encoder_Z ;
long encoder_W ;

const long EEind_X = 10 ;  // definisce Celle EEPROM
const long EEind_Y = 15 ;  // definisce Celle EEPROM
const long EEind_Z = 20 ;  // definisce Celle EEPROM
const long EEind_W = 25 ;  // definisce Celle EEPROM

//---------------------------------------------------------------------------------------------------

   void setup() {
 
//------------------------------- setup EEProm ------------
EEPROM.get(EEind_X,encoder_X);
EEPROM.get(EEind_Y,encoder_Y);
EEPROM.get(EEind_Z,encoder_Z);
EEPROM.get(EEind_W,encoder_W);

EEPROM.update(EEind_X,(encoder_X + encoderValue_X));
EEPROM.update(EEind_Y,(encoder_Y + encoderValue_Y));
EEPROM.update(EEind_Z,(encoder_Z + encoderValue_Z));
EEPROM.update(EEind_W,(encoder_W + encoderValue_W));

// ---------------------------- Definisce I/O Encoder -----------------------------------------------

     pinMode(encoder_X_Pin1, INPUT_PULLUP);
     pinMode(encoder_X_Pin2, INPUT_PULLUP);
     pinMode(encoder_Y_Pin1, INPUT_PULLUP);
     pinMode(encoder_Y_Pin2, INPUT_PULLUP);
     pinMode(encoder_Z_Pin1, INPUT_PULLUP);
     pinMode(encoder_Z_Pin2, INPUT_PULLUP);
     pinMode(encoder_W_Pin1, INPUT_PULLUP);
     pinMode(encoder_W_Pin2, INPUT_PULLUP);

//--------------------------------setup Interrupt --------

 PCintPort::attachInterrupt(encoder_X_Pin1,updateEncoder_X, CHANGE);
 PCintPort::attachInterrupt(encoder_X_Pin2,updateEncoder_X, CHANGE);
 PCintPort::attachInterrupt(encoder_Y_Pin1,updateEncoder_Y, CHANGE);
 PCintPort::attachInterrupt(encoder_Y_Pin2,updateEncoder_Y, CHANGE);
 PCintPort::attachInterrupt(encoder_Z_Pin1,updateEncoder_Z, CHANGE);
 PCintPort::attachInterrupt(encoder_Z_Pin2,updateEncoder_Z, CHANGE);
 PCintPort::attachInterrupt(encoder_W_Pin1,updateEncoder_W, CHANGE);
 PCintPort::attachInterrupt(encoder_W_Pin2,updateEncoder_W, CHANGE);
 
//-----------------------------------------------------

          Serial.begin(9600);
          delay(100);
          //Serial3.begin(9600);
          //Serial.println("Hello1");
          //Serial3.println("Hello3");
          //muovi_motore();
         
    menuOpz     = 0x00;
    Key_Reset   = 0x00;
    Key_Set     = 0x00;
    Key_A_Up    = 0x00;       //   Inizializza le variabili = ZERO
    Key_B_Down  = 0x00;
    Key_C_On    = 0x00;
    Key_D_Off   = 0x00;     
    Key_Val     = 0x00;  
    menuAsse_X  = 0x00;   
    menuAsse_Y  = 0x00;   
       
//---------------------------------------------------------------------------------------------------
    lcd.begin( 20, 4 );  lcd.backlight(); lcd.home(); // initialize the lcd
  
//----------------------------- Definisce I/O -------------------------------------------------------
 
     pinMode(LED, OUTPUT);  
     pinMode(Rele_Asse_X,OUTPUT);       pinMode(Rele_Asse_Y,OUTPUT);  
     pinMode(Rele_Asse_Z,OUTPUT);       pinMode(Rele_Asse_W,OUTPUT);
     
//----------------------------- Devinisce I/O Brake ------------------------------------------------

     pinMode(ENA, OUTPUT);           
     pinMode(IN1, OUTPUT);
     pinMode(IN2, OUTPUT);
     pinMode(StartMotor,INPUT_PULLUP);


//----------------------------- Definisce I/O Pwm Sw Assi ---------------------------------------------        
     pinMode(pinPwm_Asse_X, OUTPUT);   // Initialize the PWM and DIR pins as digital outputs.
     pinMode(pinDir_Asse_X, OUTPUT);
     pinMode(SwU_X, INPUT_PULLUP);
     pinMode(SwD_X, INPUT_PULLUP);
     
     pinMode(pinPwm_Asse_Y, OUTPUT);
     pinMode(pinDir_Asse_Y, OUTPUT);
     pinMode(SwU_Y, INPUT_PULLUP);
     pinMode(SwD_Y, INPUT_PULLUP);

     pinMode(pinPwm_Asse_Z, OUTPUT);
     pinMode(pinDir_Asse_Z, OUTPUT);
     pinMode(SwU_Z, INPUT_PULLUP);
     pinMode(SwD_Z, INPUT_PULLUP);

     pinMode(pinPwm_Asse_W, OUTPUT);
     pinMode(pinDir_Asse_W, OUTPUT);
     pinMode(SwU_W, INPUT_PULLUP);
     pinMode(SwD_W, INPUT_PULLUP);
//------------------------------ inizializza I/O -----------------------------------------------------    
 /*
    digitalWrite(encoder_X_Pin1, HIGH); //turn pullup resistor on
    digitalWrite(encoder_X_Pin2, HIGH); //turn pullup resistor on
    digitalWrite(encoder_Y_Pin1, HIGH); //turn pullup resistor on
    digitalWrite(encoder_Y_Pin2, HIGH); //turn pullup resistor on
    digitalWrite(encoder_Z_Pin1, HIGH); //turn pullup resistor on
    digitalWrite(encoder_Z_Pin2, HIGH); //turn pullup resistor on
    digitalWrite(encoder_W_Pin1, HIGH); //turn pullup resistor on
    digitalWrite(encoder_W_Pin2, HIGH); //turn pullup resistor on
*/
// --------------------------- inizializza Output ---------------------------------------------------
     digitalWrite(Rele_Asse_X,HIGH);   digitalWrite(Rele_Asse_Y,HIGH);
     digitalWrite(Rele_Asse_Z,HIGH);   digitalWrite(Rele_Asse_W,HIGH);

//---------------------------- presentazione -------------------------------------------------------

              lcd.setCursor( 5,0 );   lcd.print( "Software by" );
              lcd.setCursor( 8,1 );   lcd.print( "Mecki" );
              lcd.setCursor( 0,3 );   lcd.print( "SW.ver. 6/07/2017.4 " );
                  delay(1000);        lcd.clear();      
//--------------------------------------------------------------------------------------
              lcd.setCursor( 3,1 );   lcd.print( "Hello   RISICO" );
              lcd.setCursor( 0,3 );   lcd.print( "********************" );
                  delay(1000);        lcd.clear();    
//--------------------------------------------------------------------------------------

uint8_t i;

  }    //----End Setup 

//------------------------------------------------------------------------------------------------

void loop() {


      uint8_t count;
 
   for (i=0; i < TOTAL_PINS; i++) {
    if (interrupt_count[i] != 0) {            // guardare tutti i perni interrotti//look at all the interrupted pins
      count=interrupt_count[i];               // conservare il suo conteggio dall'ultima iterazione//store its count since the last iteration
      interrupt_count[i]=0;                   // and reset it to 0.
      
      
    } 
   }
      
      
      SuB_Clock();
      SuB_Display(); 
      SuB_KeyPad_Code();

          String bufferString = ""; /*     
          if (Serial3.available() > 0) {
                
                // read the incoming byte:
                bufferString = Serial3.readString();

                //int mm_x = map(encoder_X  ,0,40500,0,300)
                //int mm_y = map(encoder_X  ,0,40500,0,300)
                // say what you got:
                Serial.print("I received3: ");
                Serial.println(bufferString);

                if(bufferString.equals("home all")){
                  Serial.println("home all");
                  home_X();
                  home_Y();
                  home_Z();
                  home_W();
                }
                else if(bufferString.indexOf("set")!=-1){
                  //set x:512 y:712 z:112 w:212
                  Serial.println("set positions");
                  String bufferString1 = bufferString;
                  String bfx=bufferString1.substring(bufferString1.indexOf('x')+2,bufferString1.indexOf('y')-1);
                  String bfy=bufferString1.substring(bufferString1.indexOf('y')+2,bufferString1.indexOf('z')-1);
                  String bfz=bufferString1.substring(bufferString1.indexOf('z')+2,bufferString1.indexOf('w')-1);
                  String bfw=bufferString1.substring(bufferString1.indexOf('w')+2,bufferString1.length());
                  int goal_X=bfx.toInt(); int pos_X=0;
                  int goal_Y=bfy.toInt(); int pos_Y=0;
                  int goal_Z=bfz.toInt(); int pos_Z=0;
                  int goal_W=bfw.toInt(); int pos_W=0;
                  Serial.print("x=");
                  Serial.print(bfx);
                  Serial.print("; y=");
                  Serial.print(bfy);
                  Serial.print("; z=");
                  Serial.print(bfz);
                  Serial.print("; w=");
                  Serial.println(bfw);
                  
                  Serial.print("x=");
                  Serial.print(goal_X);
                  Serial.print("; y=");
                  Serial.print(goal_Y);
                  Serial.print("; z=");
                  Serial.print(goal_Z);
                  Serial.print("; w=");
                  Serial.println(goal_W);
                  
                  int axis_position=0;
                  
                  while(axis_position<4){
                    Serial.println("scattoz");
                    if(pos_X<goal_X){
                      muovi_asse_X(10);
                      pos_X+=1;
                      
                    }
                    else{
                      axis_position+=1;
                      stop_X();
                    }
                    if(pos_Y<goal_Y){
                      muovi_asse_Y(10);
                      pos_Y+=1;
                    }
                    else{
                      axis_position+=1;
                      stop_Y();
                    }
                    if(pos_Z<goal_Z){
                      muovi_asse_Z(10);
                      pos_Z+=1;
                    }
                    else{
                      axis_position+=1;
                      stop_Z();
                    }
                    if(pos_W<goal_W){
                      muovi_asse_W(10);
                      pos_W+=1;
                    }
                    else{
                      axis_position+=1;
                      stop_W();
                    }
                    
                  }
                  
                  
                  //muovi_asse_X(4);
                  //muovi_asse_Y(3);
                  //muovi_asse_Z(2);
                  //muovi_asse_W(8);
                }
                
                
        }
*/
        
        String bufferString1 = "";
        if (Serial.available() > 0) {
            //delay(100);
            Serial.println("serial1 has a message");
            // read the incoming byte:
            bufferString1 = Serial.readString();
            Serial.flush();
            // say what you got:
            Serial.print("I received1: ");
            Serial.println(bufferString1);

        }
             
          if (false) {
            //delay(100);
            Serial.println("serial1 has a message");
            // read the incoming byte:
            bufferString1 = Serial.readString();
            Serial.flush();
            // say what you got:
            Serial.print("I received1: ");
            Serial.println(bufferString1);
            
            
                if(bufferString1.equals("avanti")){
                  muovi_motore();
                  Serial.println("muovi");
                }
                else if(bufferString1.equals("readx")){
                  int mm_x = map(encoder_X  ,0,40500,0,300);
                  Serial.println(mm_x);
                }
                else if(bufferString1.equals("read switch down x")){
                  //int mm_x = map(encoder_X  ,0,40500,0,300);
                  Serial.println(digitalRead(SwD_X)==HIGH);
                }
                else if(bufferString1.equals("read switch up x")){
                  //int mm_x = map(encoder_X  ,0,40500,0,300);
                  Serial.println(digitalRead(SwU_X)==HIGH);
                }
                else if(bufferString1.equals("home x")){
                  //int mm_x = map(encoder_X  ,0,40500,0,300);
                  Serial.println("homing x");
                  home_X();
                  Serial.println(digitalRead(SwD_X)==HIGH);
                }
                else if(bufferString1.equals("home y")){
                  //int mm_x = map(encoder_X  ,0,40500,0,300);
                  Serial.println("homing y");
                  home_Y();
                  Serial.println(digitalRead(SwD_Y)==HIGH);
                }
                else if(bufferString1.equals("home z")){
                  //int mm_x = map(encoder_X  ,0,40500,0,300);
                  Serial.println("homing z");
                  home_Z();
                  Serial.println(digitalRead(SwD_Z)==HIGH);
                }
                else if(bufferString1.equals("home w")){
                  //int mm_x = map(encoder_X  ,0,40500,0,300);
                  Serial.println("homing w");
                  home_W();
                  Serial.println(digitalRead(SwD_W)==HIGH);
                }
                else if(bufferString1.equals("move")){
                  Serial.println("move x");
                  String bfx=bufferString1.substring(bufferString1.indexOf('x')+2,bufferString1.length());
                  int number=bfx.toInt();
                  if(number<10) number=10;
                  muovi_asse_X(number);
                  stop_X();
                }
                else if(bufferString1.equals("home all")){
                  Serial.println("home all");
                  home_X();
                  home_Y();
                  home_Z();
                  home_W();
                }
                else if(bufferString1.indexOf("set")!=-1){
                  //set x:512 y:712 z:112 w:212
                  Serial.println("set positions");
                  
                  String bfx=bufferString1.substring(bufferString1.indexOf('x')+2,bufferString1.indexOf('y')-1);
                  String bfy=bufferString1.substring(bufferString1.indexOf('y')+2,bufferString1.indexOf('z')-1);
                  String bfz=bufferString1.substring(bufferString1.indexOf('z')+2,bufferString1.indexOf('w')-1);
                  String bfw=bufferString1.substring(bufferString1.indexOf('w')+2,bufferString1.length());
                  int goal_X=bfx.toInt(); int pos_X=0;
                  int goal_Y=bfy.toInt(); int pos_Y=0;
                  int goal_Z=bfz.toInt(); int pos_Z=0;
                  int goal_W=bfw.toInt(); int pos_W=0;
                  Serial.print("x=");
                  Serial.print(bfx);
                  Serial.print("; y=");
                  Serial.print(bfy);
                  Serial.print("; z=");
                  Serial.print(bfz);
                  Serial.print("; w=");
                  Serial.println(bfw);
                  
                  Serial.print("x=");
                  Serial.print(goal_X);
                  Serial.print("; y=");
                  Serial.print(goal_Y);
                  Serial.print("; z=");
                  Serial.print(goal_Z);
                  Serial.print("; w=");
                  Serial.println(goal_W);
                  
                  int axis_position=0;
                  
                  while(axis_position<4){
                    Serial.println("scattoz");
                    if(pos_X<goal_X){
                      muovi_asse_X(10);
                      pos_X+=1;
                      
                    }
                    else{
                      axis_position+=1;
                      stop_X();
                    }
                    if(pos_Y<goal_Y){
                      muovi_asse_Y(10);
                      pos_Y+=1;
                    }
                    else{
                      axis_position+=1;
                      stop_Y();
                    }
                    if(pos_Z<goal_Z){
                      muovi_asse_Z(10);
                      pos_Z+=1;
                    }
                    else{
                      axis_position+=1;
                      stop_Z();
                    }
                    if(pos_W<goal_W){
                      muovi_asse_W(10);
                      pos_W+=1;
                    }
                    else{
                      axis_position+=1;
                      stop_W();
                    }
                    
                  }
                  
                  
                  //muovi_asse_X(4);
                  //muovi_asse_Y(3);
                  //muovi_asse_Z(2);
                  //muovi_asse_W(8);
                }
                
        }
           
          if(menuOpz == 0x01)   
          if(menuOpz == 0x11) ;      { SuB_Asse_X();  SuB_Encoder_X();      }  
          if(menuOpz == 0x12) ;      { SuB_Asse_Y();  SuB_Encoder_Y();      }  
          if(menuOpz == 0x13) ;      { SuB_Asse_Z();  SuB_Encoder_Z();      } 
          if(menuOpz == 0x14) ;      { SuB_Asse_W();  SuB_Encoder_W();      }             
          if(menuOpz == 0x15) ;      { SuB_Brake();    }  
          if(menuOpz == 0x16) ;      { SuB_Aux();      }  
       

 
 }      //----End Loop

//----------------end---------------------------------------------------------------------------- 
