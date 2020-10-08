#include<SoftwareSerial.h>
#include "DHT.h"
#include <MQUnifiedsensor.h>
#include <WiseChipHUD.h>
#define RX 10
#define TX 11 
#define DHTPIN_INTERN 6
#define DHTPIN_EXTERN 8  
#define DHTTYPE DHT11

#define board "Arduino MEGA"
#define Voltage_Resolution 5
#define pin_intern A1
#define pin_extern A2 
#define type_2 "MQ-9"
#define type_1 "MQ-135" 
#define ADC_Bit_Resolution 10 
#define RatioMQ135CleanAir 3.6     //According to MQ-135 datasheet
#define RatioMQ9CleanAir   9.6     //According to MQ-9 datasheet but error 

MQUnifiedsensor MQ135_Intern(board, Voltage_Resolution, ADC_Bit_Resolution, pin_intern, type_1);
MQUnifiedsensor MQ135_extern(board, Voltage_Resolution, ADC_Bit_Resolution, pin_extern, type_2);
SoftwareSerial BT(RX,TX);
DHT dht_intern(DHTPIN_INTERN, DHTTYPE);
DHT dht_extern(DHTPIN_EXTERN, DHTTYPE);
WiseChipHUD myHUD;
static const int fan_pin=5;
static const int peltier_pin=4;
static const int peltier_pot_voltage=A0;
static const int fan_pot_voltage=A3;
static const int switch_button=7;
static const int O2_level_intern_pin = A2;
static const int O2_level_extern_pin = A4;
static const int head_massage_pin=9;
static const int head_massage_pot_pin=A6;

int button_state=LOW;
int button_state2=LOW;
int button_state3=LOW;
int Count_1=1;
int Count_2=1;
int Count_3=1;
int button_event;
int fan_voltage=0;
int peltier_voltage=0;
int fan_speed=0;
int peltier_intensity=0;
int vibrate_intensity=0;
int vibrate_voltage=0;
void setup() {
 pinMode(fan_pin,OUTPUT);
 pinMode(peltier_pin,OUTPUT);
 pinMode(head_massage_pin,OUTPUT);
 pinMode(head_massage_pot_pin,INPUT);
 Serial.begin(9600);
 BT.begin(9600);
 while(BT.available()){}
 dht_intern.begin();
 dht_extern.begin();
 myHUD.begin();
  _init_hud();
 MQ135_Intern.setRegressionMethod(1); 
  MQ135_Intern.init(); 
  float calcR0 = 0;
  for(int i = 1; i<=10; i ++)
  {
    MQ135_Intern.update(); 
    calcR0 += MQ135_Intern.calibrate(RatioMQ135CleanAir);
    
  }
  MQ135_Intern.setR0(calcR0/10);
  if(isinf(calcR0)){
    Serial.println("");
  }
  if(calcR0 == 0){
    Serial.print("");
  }

  MQ135_extern.setRegressionMethod(1);
  MQ135_extern.init();
  float calcR1 = 0;
  for(int i = 1; i<=10; i ++)
  {
    MQ135_extern.update();                             // Update data, the arduino will be read the voltage on the analog pin
    calcR1 += MQ135_Intern.calibrate(RatioMQ9CleanAir);//You must change this line if you change  the sensor MQ9==>MQ135
    
  }
  MQ135_extern.setR0(calcR1/10);
}

void loop() {
  
  fan_Speed_regulator();
  head_massage_controller();
  peltier_heating_regulator();
  humidityandTempSen();
  hyper_controller();

  
  
}
void  _init_hud(){
  /************************* Compass Group *************************/
  myHUD.compassCircle(9); // 0 = All Off; 1-8 = All Off Except Selected; 9 = All On; 10-17 = All On Except Selected
  myHUD.compassArrows(9); // 0 = All Off; 1-8 = All Off Except Selected; 9 = All On; 10-17 = All On Except Selected
  //myHUD.setHeading(188);  // Max 199

  /************************* Radar Detector Group *************************/
  myHUD.radarDetector(8); // 0 = No Radar Gun Icon; 1 = Radar Gun Only; 2-8 = Distance Meter
  //myHUD.setRadarDistance(888,0); // Max 999
  myHUD.radarDistanceUnits(0); // 0 = Blank; 1 = "m"

  /************************* Destination/Waypoint Group *************************/
  myHUD.flag(1); // 0 = Blank; 1 = flag icon
  //myHUD.setDestinationDistance(888,2); // Max 999
  myHUD.destinationDistanceUnits(2); // 0 = Blank; 1 = "h"; 2 = "m"; 3 = "km"
  myHUD.H01(0); // 0 = Blank; 1 = "h"

  /************************* Exit Group *************************/
  myHUD.leftTunnel(1); // 0 = Blank; 1 = Tunnel; Also try leftRoad();
  myHUD.middleTunnel(1); // 0 = Blank; 1 = Tunnel; Also try middleRoad();
  myHUD.rightTunnel(1); // 0 = Blank; 1 = Tunnel; Also try rightRoad();

  /************************* Navigation Group *************************/
  myHUD.nav_Group(1); // 0 = Entire Nav Group Off; 1 = Entire Nav Group On
  //myHUD.setTurnDistance(888, 1); // Max 999
  myHUD.turnDistanceUnits(0); // 0 = Blank; 1 = "m"; 2 = "km"

  /************************* Call Group *************************/
  myHUD.setCallIcon(3); // 0 = Blank; 1 = Outline; 2 = Outline + Phone Icon; 3 = All Segments

  /************************* TPMS Group *************************/
  myHUD.tirePressureAlert(3); // 0 = Blank; 1 = "TPMS"; 2 = "TIRE"; 3 = All Segments
  //myHUD.setTirePressure(88,1); // Max 99
  /************************* Speedometer Group *************************/
  myHUD.setSpeedometer(188); // Max 199
  myHUD.speedometerUnits(0); // 0 = Blank; 1 = "km/h"
}

void fan_Speed_regulator(){
  int button_event=digitalRead(switch_button);
  if(button_event !=button_state){
    if(button_event == HIGH){
      Count_1++;
    }
    
  }
  
  button_state=button_event;
  
  
  if(Count_1 %2==0){
    BT.listen();
    if(BT.available() >0){
      char voi=BT.read();
      if(voi =='A'){
        fan_speed +=85;
        if(fan_speed>255){
          fan_speed=255;
        }
      }
      if(voi =='B'){
        fan_speed -=85;
        if(fan_speed<0){
          fan_speed=0;
        }
      }
    }
    analogWrite(fan_pin,fan_speed);
  }
  else if(Count_1 !=0){
    
    int pot_fan_value=analogRead(fan_pot_voltage);
    
    //Serial.println(pot_fan_value);
    fan_voltage=map(pot_fan_value,0,1023,0,255);
    analogWrite(fan_pin,fan_voltage);
  }
  

}

void head_massage_controller(){
  int button_event=digitalRead(switch_button);
  if(button_event !=button_state2){
    if(button_event == HIGH){
      Count_3++;
    }
    
  }
  
  button_state2=button_event;
  if(Count_3 %2==0){
    if(BT.available() >0){
      char vibrate=BT.read();
      if(vibrate =='C'){
        vibrate_intensity+=85;
        if(vibrate_intensity>255){
          vibrate_intensity=255;
        }
      }
      else if(vibrate=='D'){
        vibrate_intensity -=85;
        if(vibrate_intensity<0){
          vibrate_intensity=0;
        }
      }
    }
      analogWrite(head_massage_pin,vibrate_intensity);
    }
    else if(Count_3 %2 !=0){
      int read_byte=analogRead(head_massage_pot_pin);
      vibrate_voltage=map(read_byte,0,1023,0,255);
      analogWrite(head_massage_pin,vibrate_voltage);
      
      
    }
  
}


void peltier_heating_regulator(){
  int button_event=digitalRead(switch_button);
  if(button_event !=button_state3){
    if(button_event == HIGH){
      Count_2++;
    }
    
  }
  
  button_state3=button_event;
  if(Count_2 %2==0){
    if(BT.available() >0){
      char peltier=BT.read();
      if(peltier =='C'){
        peltier_intensity+=85;
        if(peltier_intensity>255){
          peltier_intensity=255;
        }
      }
      if(peltier =='D'){
        peltier_intensity -=85;
        if(peltier_intensity<0){
          peltier_intensity=0;
        }
      }
    }
    analogWrite(peltier_pin,peltier_intensity);
  }
  else if(Count_2 !=0){
    int pot_peltier_value=analogRead(peltier_pot_voltage);
    peltier_voltage=map(pot_peltier_value,0,1023,0,255);
    analogWrite(peltier_pin,peltier_voltage);
  }
}

void humidityandTempSen() {
  float h_intern = dht_intern.readHumidity();
  float t_intern = dht_intern.readTemperature();
  float f_intern = dht_intern.readTemperature(true);//Read Temperature on Fahrenheit(isFahrenheit=true);
  float h_extern = dht_extern.readHumidity();
  float t_extern = dht_extern.readTemperature();
  float f_extern = dht_extern.readTemperature(true);//Read Temperature on Fahrenheit(isFahrenheit=true);
  if (isnan(h_intern) || isnan(t_intern) || isnan(f_intern)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }
  
  Serial.print(h_intern);
  Serial.print(",");
  Serial.print(t_intern);
  Serial.print(",");
  Serial.print(f_intern);
  Serial.print(",");
  Serial.print(h_extern);
  Serial.print(",");
  Serial.print(t_extern);
  Serial.print(",");
  Serial.print(f_extern);
  Serial.print(",");
  myHUD.setRadarDistance(int(t_intern),0);
  myHUD.setDestinationDistance(int(t_extern),2);
  myHUD.setTurnDistance(int(f_intern), 1);
  myHUD.setTirePressure(int(f_extern),1);
}

void hyper_controller() {
 MQ135_Intern.update();  
 

 MQ135_Intern.setA(605.18); MQ135_Intern.setB(-3.937); 
  float CO = MQ135_Intern.readSensor(); 
 MQ135_Intern.setA(77.255); MQ135_Intern.setB(-3.18); 
  float Alcohol = MQ135_Intern.readSensor();

  MQ135_Intern.setA(110.47); MQ135_Intern.setB(-2.862); 
  float CO2 = MQ135_Intern.readSensor(); 
  MQ135_Intern.setA(44.947); MQ135_Intern.setB(-3.445); 
  float Tolueno = MQ135_Intern.readSensor();

  MQ135_Intern.setA(102.2 ); MQ135_Intern.setB(-2.473); 
  float NH4 = MQ135_Intern.readSensor();

  MQ135_Intern.setA(34.668); MQ135_Intern.setB(-3.369); 
  float Acetona = MQ135_Intern.readSensor();
  MQ135_extern.setA(599.65); MQ135_extern.setB(-2.244);
  float CO_extern=MQ135_extern.readSensor();
  
  Serial.print(CO2);
  Serial.print(",");
  Serial.print(CO);
  Serial.print(",");
  Serial.print(CO_extern);
  Serial.print(",");
  Serial.println("");
  myHUD.setHeading(int(CO2));
  myHUD.setSpeedometer(int(CO));
  
  

  
}
