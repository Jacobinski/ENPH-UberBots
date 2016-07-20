
#include <phys253.h>
#include <avr/interrupt.h>
#include <QueueList.h>  /* http://playground.arduino.cc/Code/QueueList */
#include <StackList.h>  /* http://playground.arduino.cc/Code/StackList */
#include <LiquidCrystal.h>

void setup() {
  #include <phys253setup.txt>
  enableExternalInterrupt(INT1,LOW);
  nav_init();
  Serial.begin(9600); Serial.println("Start code");
}

void loop() {
  navigate();  
}
