#include <Servo.h>

byte last_channel_1,last_channel_2,last_channel_3,last_channel_4,last_channel_5;
int rollPin = 3, throttlePin = 6, pitchPin = 5, yawPin=9,auxPin1=11,auxPin2=12;
Servo roll,throttle,pitch,yaw,aux1,aux2;
int throttle_chan,yaw_chan,roll_chan,pitch_chan,aux_chan;
unsigned long timer_1,timer_2,timer_3,timer_4,timer_5;
unsigned long current_time;
int count=0;


void setup() {
  // put your setup code here, to run once:
  PCICR |= (1 << PCIE0);                                                    //Set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= (1 << PCINT0);                                                  //Set PCINT0 (digital input 53) to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT1);                                                  //Set PCINT1 (digital input 52)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT2);                                                  //Set PCINT2 (digital input 51)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT3);                                                  //Set PCINT3 (digital input 50)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT4);                                                  //Set PCINT4 (digital input 10)to trigger an interrupt on state change.

  pinMode(rollPin,OUTPUT);
  roll.attach(rollPin);
  
  pinMode(throttlePin,OUTPUT);
  throttle.attach(throttlePin);

  pinMode(pitchPin,OUTPUT);
  pitch.attach(pitchPin);
  
  pinMode(yawPin,OUTPUT);
  yaw.attach(yawPin);
  
  pinMode(auxPin1,OUTPUT);
  aux1.attach(auxPin1);

  pinMode(auxPin2,OUTPUT);
  aux2.attach(auxPin2);
  
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
 // delay(250);
  //print_values();

  //disarm();
  

  roll.write(1500);
  pitch.write(1500);
  yaw.write(1500);

  if(count <1)
  {
    aux2.write(1000);
    aux1.write(1000);
    throttle.write(885);
  }else{
    throttle.write(throttle_chan);
    yaw.write(yaw_chan);
    roll.write(roll_chan);
    pitch.write(pitch_chan);
   
  }
  
  delay(1050);
  
  if(count <1)//arming step
  {
    //aux1.write(1600);
    aux2.write(1800);//moves from out of range to into range
    aux1.write(1400);
    delay(3000);
    count+=1;
  }

   
}
void disarm()
{
  delay(1050);
  aux2.write(1200);
}

void print_values()
{
  Serial.print("Throttle:");
  Serial.print(throttle_chan);

  Serial.print("Yaw:");
  Serial.print(yaw_chan);

  Serial.print("Roll:");
  Serial.print(roll_chan);

  Serial.print("Pitch:");
  Serial.print(pitch_chan);

  Serial.print("Aux:");
  Serial.println(aux_chan);

  
}

ISR(PCINT0_vect){
  current_time = micros();
  //Channel 1=========================================
  if(PINB & B00000001){                                                     //Is input 8 high?
    if(last_channel_1 == 0){                                                //Input 8 changed from 0 to 1.
      last_channel_1 = 1;                                                   //Remember current input state.
      timer_1 = current_time;                                               //Set timer_1 to current_time.
    }
  }
  else if(last_channel_1 == 1){                                             //Input 8 is not high and changed from 1 to 0.
    last_channel_1 = 0;                                                     //Remember current input state.
    throttle_chan = current_time - timer_1;                             //Channel 1 is current_time - timer_1.
  }
  //Channel 2=========================================
  if(PINB & B00000010 ){                                                    //Is input 9 high?
    if(last_channel_2 == 0){                                                //Input 9 changed from 0 to 1.
      last_channel_2 = 1;                                                   //Remember current input state.
      timer_2 = current_time;                                               //Set timer_2 to current_time.
    }
  }
  else if(last_channel_2 == 1){                                             //Input 9 is not high and changed from 1 to 0.
    last_channel_2 = 0;                                                     //Remember current input state.
    yaw_chan = current_time - timer_2;                             //Channel 2 is current_time - timer_2.
  }
  //Channel 3=========================================
  if(PINB & B00000100 ){                                                    //Is input 10 high?
    if(last_channel_3 == 0){                                                //Input 10 changed from 0 to 1.
      last_channel_3 = 1;                                                   //Remember current input state.
      timer_3 = current_time;                                               //Set timer_3 to current_time.
    }
  }
  else if(last_channel_3 == 1){                                             //Input 10 is not high and changed from 1 to 0.
    last_channel_3 = 0;                                                     //Remember current input state.
    roll_chan = current_time - timer_3;                             //Channel 3 is current_time - timer_3.

  }
  //Channel 4=========================================
  if(PINB & B00001000 ){                                                    //Is input 11 high?
    if(last_channel_4 == 0){                                                //Input 11 changed from 0 to 1.
      last_channel_4 = 1;                                                   //Remember current input state.
      timer_4 = current_time;                                               //Set timer_4 to current_time.
    }
  }
  else if(last_channel_4 == 1){                                             //Input 11 is not high and changed from 1 to 0.
    last_channel_4 = 0;                                                     //Remember current input state.
    pitch_chan = current_time - timer_4;                             //Channel 4 is current_time - timer_4.
  }

   //Channel 5=========================================
  if(PINB & B00010000 ){                                                    //Is input 11 high?
    if(last_channel_5 == 0){                                                //Input 11 changed from 0 to 1.
      last_channel_5 = 1;                                                   //Remember current input state.
      timer_5 = current_time;                                               //Set timer_4 to current_time.
    }
  }
  else if(last_channel_5 == 1){                                             //Input 11 is not high and changed from 1 to 0.
    last_channel_5 = 0;                                                     //Remember current input state.
    aux_chan = current_time - timer_5;                             //Channel 4 is current_time - timer_4.
  }
}
