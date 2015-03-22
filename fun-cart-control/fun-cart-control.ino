// fun-cart Makerspace.lt Arduino based controller v1.0
// last update 2015.03.22
//removed delay() from last 20s counter, now using milis() to blink led and buzzer

#include <EEPROM.h>  // We are going to read and write user data from/to EEPROM


//#define COMMON_ANODE

#ifdef COMMON_ANODE
#define LED_ON LOW
#define LED_OFF HIGH
#else
#define LED_ON HIGH
#define LED_OFF LOW
#endif

#define redLed 2
#define buzzer 4
#define resetB 3 // Button pin for enter programing mode at startup

boolean programMode = false; // initialize programming mode to false

int level=1800;


// using a 16-bit timer in arduino is a bit more complex because arduinos standad setup stets them to 8 bit
// the higher the resolution is the slower is the timer .. (it counts always in the same speed .. so it takes 3* longer to count till 30 then it takes to count to10)
// because of this and the fact that MultiiWii dont uses 16-bit for the motor calculations we dont need to set them to thair full resolution
// we set it to 0 - 16383 with that value and the  phase and frequency correct mode we dont need a prescaler.
// to use the MWC's values of 1000-2000 we need to multiply the value by 8 .. (1000*8 = 8000, 2000*8 = 16000)  
// we do it also with bit shifting .. but in the other direction
  int greitis=1040; 
   int dutytime = (greitis<<3); // this would give us a minimal starting throttle signal
// X<<1 = X*2, X<<2 = X*4 and X<<3 = X*8.... 
  int pot_pin=A0;
  int led_pin=2;
  int motor_pin=9;
  const int numReadings = 3;
  int readings[numReadings];      // the readings from the analog input
  int index = 0;                  // the index of the current reading
  int total = 0;                  // the running total
  int average = 0;                // the average
  int max_ppm=1900;
  int incomingByte =0;
  long currentMillis=0;
  long mins=0;
  long secs=0;
  long laikas=0;
  int runtime=0;
  unsigned long BeepTimer=0;
  unsigned long interval = 500; 
  int ledState = LOW; 
  unsigned long previousMillis = 0; // will store last time LED was updated
  long OnTime = 250; // milliseconds of on-time
  long OffTime = 750; // milliseconds of off-time
  
//  int buttonPushCounter = 0;   // counter for the number of button presses
//int buttonState = 0;         // current state of the button
////int lastButtonState = 0;     // previous state of the button
///////////////////////////////////////// Setup ///////////////////////////////////
void setup() {
  //Arduino Pin Configuration
  pinMode(redLed, OUTPUT);
  pinMode(buzzer, OUTPUT);
  digitalWrite(buzzer, LOW); // Make sure buzzer is off
  digitalWrite(redLed, LED_OFF); // Make sure led is off
   
  Serial.begin(9600);	 // Initialize serial communications with PC

   pinMode(motor_pin, OUTPUT); // set pin 9 to output
   pinMode(pot_pin, INPUT);
   TCCR1A |= (1<<WGM11); TCCR1A &= ~(1<<WGM10); TCCR1B |= (1<<WGM13);  // phase correct mode.. to know how to set which register please see the datasheets of the MCU's
   TCCR1B &= ~(1<<CS11); // no prescaler
   ICR1 |= 0x3FFF; // set timers to count to 16383 (hex 3FFF)
   TCCR1A |= _BV(COM1A1); // connect pin 9 to timer 1 channel A
   OCR1A = dutytime;
    cycleLeds();
   delay(3000);
    cycleLeds();
   for (int thisReading = 0; thisReading < numReadings; thisReading++) readings[thisReading] = 0;
   
  //Enter programing mode if Button Pressed while setup run (powered on)
  pinMode(resetB, INPUT_PULLUP);  // Enable pin's pull up resistor
  if (digitalRead(resetB) == LOW) {     // when button pressed pin should get low, button connected to ground
    digitalWrite(redLed, LED_ON);   // Red Led stays on to inform user we are going to enter programing mode
    delay(5000);    // Give user enough time to cancel operation
    if (digitalRead(resetB) == LOW) {  // If button still be pressed, enter programing mode
        programMode=true;
        // visualize successful entering to programing mode
       cycleLeds();
       cycleLeds();
       cycleLeds();
       }
        else {
              digitalWrite(redLed, LED_OFF);
             }
  }

int sk=EEPROM.read(199);
runtime=EEPROM.read(200+sk);
max_ppm=8*runtime;
   if(runtime<=120) {runtime=280-runtime+40;} else runtime=280-runtime;
    Serial.println(runtime);
}


///////////////////////////////////////// Main Loop ///////////////////////////////////
void loop () {
        
    if (programMode) 
        {
          if (digitalRead(resetB) == LOW) // when button pressed pin should get low, button connected to ground
            {     
                delay(2000);
                if (digitalRead(resetB) == LOW) // when button pressed pin should get low, button connected to ground
                    {
                      int sk=EEPROM.read(199);
                       if (sk==5)
                         { 
                                 EEPROM.write(199, 0);
                         }  else
                           {
                            sk+=1;
                            EEPROM.write(199, sk); // if not write 0, it takes 3.3mS
                            for ( int i = 0; i < sk; i++ ) {     // Read ride level from EEPROM
                                                         cycleLeds();
                              }
                          }
                   }
            }
        } 
 
   //*************************************************************************************
  // filling array with Pot value and counting average from stored array elements ********
   total= total - readings[index];         
  // read from the sensor:  
  readings[index] = analogRead(pot_pin); 
  // add the reading to the total:
  total= total + readings[index];       
  // advance to the next position in the array:  
  index = index + 1;                    
  // if we're at the end of the array...
  if (index >= numReadings)              
    // ...wrap around to the beginning: 
    index = 0;                           
  // calculate the average:
  average = total / numReadings;         
 //****************************************************************************************

  average=map(average, 0, 860, 1020, max_ppm);
  currentMillis=millis();
   
  if(secs==0){
    dutytime=(1040<<3);
  }
  else {
        secs =laikas- currentMillis/1000; //convect milliseconds to seconds
        dutytime=(average<<3);  
  }
   if((secs<20)&&(secs>0)){
        blinker();
  }
/*
  Serial.println("Left Time");
  Serial.println(secs);
  Serial.print("ESC reiksme: ");
  Serial.print(average,DEC);
  Serial.print("  ESC dutytime: ");
  Serial.println(dutytime,DEC);
  */
 // if(dutytime<8200) digitalWrite(led_pin,HIGH); else digitalWrite(led_pin,LOW);

OCR1A = dutytime;
  
  if(!programMode){
   /*  buttonState = digitalRead(resetB);

  // compare the buttonState to its previous state
  if (buttonState != lastButtonState) {
    // if the state has changed, increment the counter
    if (buttonState == HIGH) {
      // if the current state is HIGH then the button
      // wend from off to on:
      buttonPushCounter++;
      Serial.println("on");
      Serial.print("number of button pushes:  ");
      Serial.println(buttonPushCounter);
    }
    else {
      // if the current state is LOW then the button
      // wend from on to off:
      Serial.println("off");
    }
  }
  // save the current state as the last state,
  //for next time through the loop
  lastButtonState = buttonState;

 
  // turns on the LED every four button pushes by
  // checking the modulo of the button push counter.
  // the modulo function gives you the remainder of
  // the division of two numbers:
  if (buttonPushCounter % 4 == 0) {
    digitalWrite(buzzer, HIGH);
  } else {
   digitalWrite(buzzer, LOW);
  }
    
    */
    
   if (digitalRead(resetB) == LOW) {     // when button pressed pin should get low, button connected to ground
     delay(3000);
     if (digitalRead(resetB) == LOW) {
        cycleLeds();  
        cycleLeds();
        currentMillis=millis();
        laikas=runtime+currentMillis/1000;
        secs =laikas- currentMillis/1000; 
     }  
   } 
  }

}


///////////////////////////////////////// Cycle Leds (Program Mode) ///////////////////////////////////
void cycleLeds() {
  digitalWrite(redLed, LED_OFF); // Make sure red LED is off
  digitalWrite(buzzer, LOW); 
  delay(200);
  digitalWrite(redLed, LED_ON); // Make sure red LED is off
  digitalWrite(buzzer, HIGH);   
  delay(200);
  digitalWrite(redLed, LED_OFF); // Make sure red LED is off
  digitalWrite(buzzer, LOW);
  delay(200);
}
/*
void beep(){
  
 //  unsigned long currentMillis = millis();
 
  if(currentMillis - previousMillis > interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;  

    // if the LED is off turn it on and vice-versa:
    if (ledState == HIGH)
      ledState = LOW;
    else
      ledState = HIGH;

    // set the LED with the ledState of the variable:
    digitalWrite(buzzer, ledState);
    digitalWrite(redLed, ledState);
  } //else digitalWrite(buzzer, LOW);
  
}
*/
void blinker(){
//  unsigned long currentMillis = millis();
if((ledState == HIGH) && (currentMillis - previousMillis >= OnTime))
{
ledState = LOW; // Turn it off
previousMillis = currentMillis; // Remember the time
digitalWrite(buzzer, ledState); // Update the actual LED
digitalWrite(redLed, ledState);
}
else if ((ledState == LOW) && (currentMillis - previousMillis >= OffTime))
{
ledState = HIGH; // turn it on
previousMillis = currentMillis; // Remember the time
digitalWrite(buzzer, ledState);	// Update the actual LED
digitalWrite(redLed, ledState);
} 
  
 }

//***************************************************************************
/*
void printDouble( double val, byte precision){
  // prints val with number of decimal places determine by precision
  // precision is a number from 0 to 6 indicating the desired decimial places
  // example: printDouble( 3.1415, 2); // prints 3.14 (two decimal places)

  Serial.print (int(val));  //prints the int part
  if( precision > 0) {
    Serial.print("."); // print the decimal point
    unsigned long frac;
    unsigned long mult = 1;
    byte padding = precision -1;
    while(precision--)
       mult *=10;
       
    if(val >= 0)
      frac = (val - int(val)) * mult;
    else
      frac = (int(val)- val ) * mult;
    unsigned long frac1 = frac;
    while( frac1 /= 10 )
      padding--;
    while(  padding--)
      Serial.print("0");
    Serial.print(frac,DEC) ;
  }
}
*/
