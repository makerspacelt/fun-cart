// fun-cart Makerspace.lt Arduino based controller v1.0
// last update 2015.03.21

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
  long currentmillis=0;
  long mins=0;
  long secs=0;
  long laikas=0;
  int runtime=0;
///////////////////////////////////////// Setup ///////////////////////////////////
void setup() {
  //Arduino Pin Configuration
  pinMode(redLed, OUTPUT);
  pinMode(buzzer, OUTPUT);
  digitalWrite(buzzer, LOW); // Make sure buzzer is off
  digitalWrite(redLed, LED_OFF); // Make sure led is off
   
//  Serial.begin(9600);	 // Initialize serial communications with PC

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
   for (int thisReading = 0; thisReading < numReadings; thisReading++)
    readings[thisReading] = 0;
   
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
// Serial.print("Max-ppm was: ");
// Serial.print(max_ppm);
//  Serial.print("  at EEPROM address ");
//   Serial.println(200+sk);
   if(runtime<=120) {runtime=300-runtime+40;} else runtime=300-runtime;
}


///////////////////////////////////////// Main Loop ///////////////////////////////////
void loop () {
        
    if (programMode) {
   
      if (digitalRead(resetB) == LOW) {     // when button pressed pin should get low, button connected to ground
        digitalWrite(redLed, LED_ON);
        digitalWrite(buzzer, HIGH);
      }
      delay(2000);
      if (digitalRead(resetB) == LOW) {     // when button pressed pin should get low, button connected to ground
          digitalWrite(redLed, LED_OFF); 
          digitalWrite(buzzer, LOW);
          int sk=EEPROM.read(199);
           if (sk==5){ 
             EEPROM.write(199, 0);
            } 
            else{
              sk+=1;
              EEPROM.write(199, sk); // if not write 0, it takes 3.3mS
              for ( int i = 0; i < sk; i++ ) {     // Read Master Card's UID from EEPROM
                   cycleLeds();
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
 
//  average=analogRead(pot_pin);
  // send it to the computer as ASCII digits
// Serial.print("gazo reiksme:---------    ");
//  Serial.println(average);   
/*
   //apply expo to throtle valuebegin
  byte expo_byt = 50; // 0=none, 25=medium, 50=strong 100=too much
		if (expo_byt != 0) {
			// apply full exponential curve to the throttle channel (contributed by jbjb)
			float expoval_flt = expo_byt / 10.0;
			float value_flt = average /860; //1023.0; // map to [0, +1] range
			value_flt = value_flt * exp(abs(expoval_flt * value_flt)) / exp(expoval_flt);
			average = (unsigned int)(860 * value_flt); // map to [0, 1023] range
                     Serial.print("VALFLOAT: ");
  Serial.print((unsigned int)(value_flt*1000));  
                    }
  */
  
  
  //expo end

  average=map(average, 0, 860, 1020, max_ppm);
  currentmillis=millis();
   
  if(secs==0){
    dutytime=(1040<<3);
  }
  else {
        secs =laikas- currentmillis/1000; //convect milliseconds to seconds
        dutytime=(average<<3);  
  }
  
  if((secs<20)&&(secs>0)){
      cycleLeds(); 
  }
/*
  Serial.println("Left Time");
  Serial.println(secs);
  Serial.print("ESC reiksme: ");
  Serial.print(average,DEC);
  Serial.print("  ESC dutytime: ");
  Serial.println(dutytime,DEC);
  */
  if(dutytime<1200) digitalWrite(led_pin,HIGH); else digitalWrite(led_pin,LOW);

OCR1A = dutytime;
  
  if(!programMode){
   if (digitalRead(resetB) == LOW) {     // when button pressed pin should get low, button connected to ground
     delay(3000);
     if (digitalRead(resetB) == LOW) {
        cycleLeds();  
        cycleLeds();
        currentmillis=millis();
        laikas=runtime+currentmillis/1000;
        secs =laikas- currentmillis/1000; 
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



