#include <ezButton.h>
#include <HX711_ADC.h>
#if defined(ESP8266)|| defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif


///Sets up limit switches

int LS_rht=A5,LS_frnt=A4,LS_bck=A3,LS_lft=A2,debounce_time=75;
ezButton Front_LS(LS_frnt);
ezButton Back_LS(LS_bck);
ezButton Right_LS(LS_rht);
ezButton Left_LS(LS_lft);



///Sets up pins and variables for finger linear actuators
int left_finger_pin=12, right_finger_pin=13,right_brake=8,left_brake=9;
int open_speed=255,close_speed=255,close_time=1000, open_time=1000;
bool expanded, Front_pressed, Back_pressed, Right_pressed, Left_pressed, override_expand, override_collapse;
bool printed=false, De_Bug_Button=false;
int skip;

int Front_state;

char serial_data;


/////Load Cells//////

//pins:
const int HX711_dout_1 = 6; //mcu > HX711 no 1 dout pin
const int HX711_sck_1 = 7; //mcu > HX711 no 1 sck pin
const int HX711_dout_2 = 4; //mcu > HX711 no 2 dout pin
const int HX711_sck_2 = 5; //mcu > HX711 no 2 sck pin

//HX711 constructor (dout pin, sck pin)
HX711_ADC LoadCell_1(HX711_dout_1, HX711_sck_1); //HX711 1
HX711_ADC LoadCell_2(HX711_dout_2, HX711_sck_2); //HX711 2

unsigned long t = 0;
unsigned long timer;
float loop_time=0.015;

float load_1_reading,load_2_reading;
int mating_safety_factor=1.1;
double push_thresh_1=936.1, push_thresh_2=879.8;

#define period_us 10000

///////Serial Output/////////////////////////////////
//#define MATLAB_SERIAL_READ
//#define Serial_Print
#ifdef Serial_Print
//De_Bug_Button=true;
#endif
//#define Python_Serial
void setup() {
  Serial.begin(115200);

////set debounce time for limit switches
  Front_LS.setDebounceTime(debounce_time);
  Back_LS.setDebounceTime(debounce_time);
  Right_LS.setDebounceTime(debounce_time);
  Left_LS.setDebounceTime(debounce_time);

////Set pins for linear actuator
  pinMode(left_finger_pin,OUTPUT);
  pinMode(left_brake,OUTPUT); //brake left
  pinMode(right_finger_pin,OUTPUT);  
  pinMode(right_brake, OUTPUT); //brake right
  collapsefingers();
  delay(500);
  silencefingers();


////SetUp Load Cells///
  push_thresh_1=900, push_thresh_2=900;
  float calibrationValue_1=-205.38; // calibration value load cell 1
  float calibrationValue_2=-218.79; // calibration value load cell 2
  LoadCell_1.begin();
  LoadCell_2.begin();

  unsigned long stabilizingtime = 2000; // tare preciscion can be improved by adding a few seconds of stabilizing time
  boolean _tare = true; //set this to false if you don't want tare to be performed in the next step
  byte loadcell_1_rdy = 0;
  byte loadcell_2_rdy = 0;
  while ((loadcell_1_rdy + loadcell_2_rdy) < 2) { //run startup, stabilization and tare, both modules simultaniously
    if (!loadcell_1_rdy) loadcell_1_rdy = LoadCell_1.startMultiple(stabilizingtime, _tare);
    if (!loadcell_2_rdy) loadcell_2_rdy = LoadCell_2.startMultiple(stabilizingtime, _tare);
  }
  if (LoadCell_1.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 no.1 wiring and pin designations");
  }
  if (LoadCell_2.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 no.2 wiring and pin designations");
  }
  LoadCell_1.setCalFactor(calibrationValue_1); // user set calibration value (float)
  LoadCell_2.setCalFactor(calibrationValue_2); // user set calibration value (float)

  delay(50);
}

void loop() {

  if (micros() - timer >= period_us) {
  timer = micros();
  
  // put your main code here, to run repeatedly:
  Front_LS.loop();//needed for checking if pressed
  Back_LS.loop();
  Right_LS.loop();
  Left_LS.loop();

  // read serial data if available
  if(Serial.available() > 0){
    serial_data = Serial.read();

    if(serial_data == '1' && expanded == false){
      override_expand = true;
    }

    else if(serial_data == '0' && expanded == true){
      override_collapse = true;
    }
  }

////////Load Cells ready to get data then grabs data/////
  static boolean newDataReady=0;
  if (LoadCell_1.update()) newDataReady = true;
  LoadCell_2.update();

  if (newDataReady){
    load_1_reading=LoadCell_1.getData();
    load_2_reading=LoadCell_2.getData();
  }
  
//////////Print to check Limit Switches/////////////
//  Serial.print(Front_LS.getState());
//  Serial.print("  ");
//  Serial.print(Back_LS.getState());
//  Serial.print("  ");
//  Serial.print(Right_LS.getState());
//  Serial.print("  ");
//  Serial.println(Left_LS.getState());
//  Serial.println(expanded);
//  Serial.print("press check");
//  Serial.println(Front_LS.isPressed());
////////////////////////////////////////////////


      if (Front_LS.isPressed()){
        Front_pressed=true;

      ///Printing Purposes//////////
        if (De_Bug_Button){  
          Serial.print("Front ");
          printed=true;
          }
      }

      if (Front_LS.isReleased()){
        Front_pressed=false;
      }

      if (Back_LS.isPressed()){
        Back_pressed=true;
      ///Printing Purposes//////////
        if (De_Bug_Button){  
          Serial.print("Back ");
          printed=true;
          }
      }

      if (Back_LS.isReleased()){
        Back_pressed=false;
      }

      if (Right_LS.isPressed()){
        Right_pressed=true;
      ///Printing Purposes//////////
        if (De_Bug_Button){  
          Serial.print("Right ");
          printed=true;
          }
      }

      if (Right_LS.isReleased()){
        Right_pressed=false;
      }

      if (Left_LS.isPressed()){
        Left_pressed=true;
      ///Printing Purposes//////////
        if (De_Bug_Button){  
          Serial.print("Left ");
          printed=true;
          }
      }

      if (Left_LS.isReleased()){
        Left_pressed=false;
      }

      ///Printing Purposes//////////
        if (De_Bug_Button){  
          if (printed){
          Serial.println(" ");}
          printed=false;
          }
      
      if ((Front_pressed && Back_pressed && Right_pressed && Left_pressed && !expanded) || override_expand){
        #ifdef Serial_Print
        Serial.println("We've been pressed");
        #endif
        ////Resets Pressed Parameter now that all of have been pressed
        Front_pressed=false;
        Back_pressed=false;
        Right_pressed=false;
        Left_pressed=false;
        expandfingers();
        expanded=true;
        override_expand = false;
        delay(1200);
    }
    if ((expanded && load_1_reading>push_thresh_1*mating_safety_factor&& load_2_reading>push_thresh_2*mating_safety_factor ) || override_collapse){
      #ifdef Serial_Print
      Serial.println("Freedom");
      #endif
      collapsefingers();
      expanded=false;
      override_collapse = false;
      delay(1000); //allow time for fingers to collapse
    }
#ifdef MATLAB_SERIAL_READ
    Serial.print(timer / 1000000.0 - loop_time);
    Serial.print("\t");
    Serial.print(load_1_reading);
    Serial.print("\t");
    Serial.println(load_2_reading);
#endif

#ifdef Serial_Print
    Serial.print("load 1 ");
    Serial.print(load_1_reading);
    Serial.print(" load 2 ");
    Serial.println(load_2_reading);
#endif

#ifdef Python_Serial
    Serial.print(load_1_reading);
    Serial.print(' ');
    Serial.print(load_2_reading);
    Serial.print(' ');
    Serial.println(expanded);
    Serial.flush();
#endif
  }
  loop_time = (micros() - timer) / 1000000.0;
 }


///////FINGERS OPENING AND CLOSING FUNCTIONS//////////////
//////////////////////////////////////////////////////////
void expandfingers(){
  digitalWrite(left_finger_pin, LOW);
  digitalWrite(left_brake,LOW);

  digitalWrite(right_finger_pin, LOW);
  digitalWrite(right_brake,LOW);
  analogWrite(3,open_speed);
  analogWrite(11,open_speed);
  expanded=true;
}

void collapsefingers(){
  digitalWrite(left_finger_pin, HIGH);
  digitalWrite(right_finger_pin, HIGH);
  digitalWrite(left_brake,LOW);
  digitalWrite(right_brake,LOW);
  analogWrite(3,close_speed);
  analogWrite(11,close_speed);
  expanded=false;
}

void silencefingers(){
  analogWrite(3,0);
  analogWrite(11,0);
}
////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
