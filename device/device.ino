#include <Arduino.h>


#define HORIZONTAL_INTERVAL 1000
#define VERTICAL_INTERVAL 300
#define VERTICAL_STEP_PIN 5
#define VERTICAL_DIR_PIN 6
#define HORIZONTAL_STEP_PIN 3
#define HORIZONTAL_DIR_PIN 4
#define VERTICAL_MOVE_STEPS 15000


void setup() {
  pinMode(VERTICAL_DIR_PIN,OUTPUT);
  pinMode(VERTICAL_STEP_PIN,OUTPUT);
  pinMode(HORIZONTAL_STEP_PIN,OUTPUT);
  pinMode(HORIZONTAL_DIR_PIN,OUTPUT);
  digitalWrite(VERTICAL_DIR_PIN,LOW);
  digitalWrite(VERTICAL_STEP_PIN,LOW);
  digitalWrite(HORIZONTAL_DIR_PIN,LOW);
  digitalWrite(HORIZONTAL_STEP_PIN,LOW);

  Serial.begin(115200);
}

void loop() {
    int index = 0; 
    while(!Serial.available()){}
    String signal = Serial.readStringUntil('\n');
    Serial.println(signal);
    char* signalChar = signal.c_str();
    char command;
    int loop;
    sscanf(signalChar,"%s %d",&command,&loop);
    


    if(command == 'c') {
        move('s',VERTICAL_MOVE_STEPS);
        if(loop > 0) move('a',loop);
        else move('d',-loop);
        move('w',VERTICAL_MOVE_STEPS);
    } else {
        move(command,loop);
    }

}

void move(char direction, int loop) {
    int pin,delay;
    switch(direction) {
        case 'd':
            digitalWrite(HORIZONTAL_DIR_PIN,HIGH);
            pin = HORIZONTAL_STEP_PIN;
            delay = HORIZONTAL_INTERVAL;
            break;
        case 'a':
            digitalWrite(HORIZONTAL_DIR_PIN,LOW);
            pin = HORIZONTAL_STEP_PIN;
            delay = HORIZONTAL_INTERVAL;
            break;
        case 's':
            digitalWrite(VERTICAL_DIR_PIN,LOW);
            pin = VERTICAL_STEP_PIN;
            delay = VERTICAL_INTERVAL;
            break;
        case 'w':
            digitalWrite(VERTICAL_DIR_PIN,HIGH);
            pin = VERTICAL_STEP_PIN;
            delay = VERTICAL_INTERVAL;
            break;
        default:
            Serial.println("command not found");
            return;
    }
    char prt[30];
    sprintf(prt,"command: %c, loop: %d",direction,loop);
    Serial.println(prt);
    for(int i = 0;i < loop;++i) {
        digitalWrite(pin,HIGH);
        delayMicroseconds(delay);
        digitalWrite(pin,LOW);
        delayMicroseconds(delay);
    }
    return;
}
