/////////////// achter is moeten de pinnen komen waarop zijn aangesloten//////////////////
#define linkssensor 15
#define rechtssensor 19

void setup(){
Pinmode(linkssensor, INPUT);
Pinmode(rechtssenosr, INPUT);
}

void loop(){
int statusled= digitalRead (linkssensor);
int statusled2= digitalRead (rechtssensor);

if(linkssensor == 1){
Serial.println(linkssensor);}
if(rechtssensor == 1){
Serial.println(rechtssensor);}

}
