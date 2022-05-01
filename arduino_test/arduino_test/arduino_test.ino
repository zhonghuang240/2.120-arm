
float load_1_reading = 2001.1; 
float load_2_reading = 3322.1;
bool gripped = false;
bool switches_pressed = true;
String data;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:

  if(Serial.available()){
    data = Serial.readString();
    int b = data.toInt();
    Serial.print(b);
    Serial.print('\r'); 
    Serial.print('\n');
  }
  //Serial.print(data);
  /*
  Serial.print(load_1_reading);
  Serial.print(' ');
  Serial.print(load_2_reading);
  Serial.print(' ');
  Serial.print(gripped);
  Serial.print(' ');
  Serial.print(switches_pressed); */
  Serial.print('\r'); 
  Serial.print('\n');
  
}
