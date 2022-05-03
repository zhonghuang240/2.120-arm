
float load_1_reading = 2001.1; 
float load_2_reading = 3322.1;
bool gripped = false;
bool switches_pressed = true;
char data;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

  if(Serial.available() > 0){
    data = Serial.read();

    if(data == '1'){
      digitalWrite(LED_BUILTIN, HIGH);
    }

    if(data == '0'){
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
  

/*
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
  */
  /*
  Serial.print(load_1_reading);
  Serial.print(' ');
  Serial.print(load_2_reading);
  Serial.print(' ');
  Serial.print(gripped);
  Serial.print(' ');
  Serial.print(switches_pressed); 
  Serial.print('\r'); 
  Serial.print('\n'); */
  
}
