#include <PID_v1.h>

double lightLevel;         // retine nivelul de lumina (=treapta)
// parametrii PID
float Kp = 5;              // Proportional gain
float Ki = 1;             // Integral gain
float Kd = 0.02;              // Differential gain
// retine valorile pentru treapta si pt controller input si output
double Setpoint, Input, Output;
// creeaza un controller PID pt valorile de Input(=lumina de la led), Output(=pwm) si Setpoint(=treapta)
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
const int sampleRate = 1;        // rata de instantiere a controllerului PID
const long serialPing = 500;     // de cate ori se primeste data de la Arduino
unsigned long now = 0;           // retine timpul care a trecut
unsigned long lastMessage = 0;   // ultima data cand au fost primite date

class LightAnsable // clasa ansablului Led-Fotorezistor
{
private:
 int photores; //pin fotorezistor
 int   pot; //pin potentiometru
 int   led; // pin pwm led
double lightLevel; //nivel lumina = treapta
  
public:
  LightAnsable() 
  {
    photores=A0;
    pot=A1;
    led=9;
    lightLevel=0;
  }
 LightAnsable(int ph,int po,int l,double LL)
  {
    photores=ph;
    pot=po;
    led=l;
    lightLevel=LL;
  }
  int getPhotores(){return photores;}
  int getPot(){return pot;}
  int getLed(){return led;}
  double getLightLevel()
  {
    lightLevel = analogRead(photores); //citeste treapta
    return lightLevel;
  }
  double getInput()
  {
    lightLevel = analogRead(photores);
    return  map(lightLevel, 0, 1023, 0, 255)  ;  //scaleaza inputul, Arduino are o resolutie de 10 biti pt analogueRead() si de 8 biti pt analogueWrite()
  }
  double getSetpoint()
  {
    return map(analogRead(pot), 0, 1023, 0, 255); //scaleaza treapta
  }
  
};

LightAnsable led;//(A0,A1,9);
 
void setup()
 {

lightLevel = led.getLightLevel();

 Input = led.getInput();

 Serial.begin(9600);                                // initializeaza comunicarea seriala la 9600 bps
 myPID.SetMode(AUTOMATIC);                          // porneste PID-ul
 myPID.SetSampleTime(sampleRate);                   // seteaza rata de instantiere a controllerului
 Serial.println("Begin");                           // setup-ul e gata
 lastMessage = millis();                            // data va fi primita incepand de aici
 }
  
void loop()
 {

 Setpoint= led.getSetpoint();
 Input=led.getInput();                               //citeste input-ul (=lumina de la led)
 myPID.Compute();                                   // calculeaza outputul PId-ului la timpul de instatiere specificat
 analogWrite(led.getLed(), Output);                          // porneste led-ul
 now = millis();                                    // Keep track of the elapsed time
 if(now - lastMessage > serialPing)                 //  daca a trecut suficient timp, trimite data
 {
 Serial.print("Setpoint = ");
 Serial.print(Setpoint);
 Serial.print(" Input = ");
 Serial.print(Input);
 Serial.print(" Output = ");
 Serial.print(Output);
 Serial.print("\n");
 //Serial.print(" Kp,Ki,Kd = "); 
// Serial.print(Kp);
// Serial.print(",");
// Serial.print(Ki);
// Serial.print(",");
// Serial.println(Kd);
 
 // Parametrii PID-ului pot fi setati din serial monitor : 0,0.5,0 seteaza Ki la 0.5.
 // virgulele sunt ingnorate de Serial.parseFloat()
 if (Serial.available() > 0)
 {
 for (int x = 0; x < 4; x++)
 {
 switch(x)
 {
 case 0:
 Kp = Serial.parseFloat();
 break;
 case 1:
 Ki = Serial.parseFloat();
 break;
 case 2:
 Kd = Serial.parseFloat();
 break;
 case 3:
 for (int y = Serial.available(); y == 0; y--)
 {
 Serial.read();
 }
 break;
 }
 }
 Serial.print(" Kp,Ki,Kd = ");  // afisaza noii parametrii ai PID-ului
 Serial.print(Kp);
 Serial.print(",");
 Serial.print(Ki);
 Serial.print(",");
 Serial.println(Kd);
 myPID.SetTunings(Kp, Ki, Kd);  // seteaza noii parametrii ai PID-ului
 }
 lastMessage = now;               // referinta pentru urmatoarea comunicare seriala
 }
 }
