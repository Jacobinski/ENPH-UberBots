/*  Enables an external interrupt pin
INTX: Which interrupt should be configured?
    INT0    - will trigger ISR(INT0_vect)
    INT1    - will trigger ISR(INT1_vect)
    INT2    - will trigger ISR(INT2_vect)
    INT3    - will trigger ISR(INT3_vect)
mode: Which pin state should trigger the interrupt?
    LOW     - trigger whenever pin state is LOW
    FALLING - trigger when pin state changes from HIGH to LOW
    RISING  - trigger when pin state changes from LOW  to HIGH 
*/
volatile unsigned int INT_1 = 0; //Stop button interupts -> Opens menu

void enableExternalInterrupt(unsigned int INTX, unsigned int mode)
{
  if (INTX > 51 || mode > 3 || mode == 1) return; //Return if unreasonable input
  cli();
  /* Allow pin to trigger interrupts        */
  EIMSK |= (1 << INTX);
  /* Clear the interrupt configuration bits */
  EICRA &= ~(1 << (INTX*2+0));
  EICRA &= ~(1 << (INTX*2+1));
  /* Set new interrupt configuration bits   */
  EICRA |= mode << (INTX*2);
  sei();
}

/* Disables an external interrupt pin */
void disableExternalInterrupt(unsigned int INTX)
{
  if (INTX > 51) return;
  EIMSK &= ~(1 << INTX);
}

/* Configures the functionality of Digital Pin 1 Interrupt */
ISR(INT1_vect)
{
  int t;
  motor.speed(2,0); //stop left motor
  motor.speed(3,0); //stop right motor
  while(!stopbutton()){  //Stop button resumes operation
    if(t == 3600){
         LCD.clear();
         LCD.setCursor(0,0);
         LCD.print("lm:");
         LCD.print(analogRead(5));
         LCD.print("rm:");
         LCD.print(analogRead(3)); 
         LCD.print("kd:");
         LCD.print(knob(6)/4); 
         LCD.setCursor(0,1);
         LCD.print("li:");
         LCD.print(analogRead(0));
         LCD.print("ri:");
         LCD.print(analogRead(1));
          LCD.print("kp:");
         LCD.print(knob(7)/4);
      t = 0;
    }
    t = t + 1;
  }
}

