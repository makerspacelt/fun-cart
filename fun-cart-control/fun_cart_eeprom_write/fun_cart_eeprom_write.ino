/*
 * EEPROM Write
 *
 * Stores values read from analog input 0 into the EEPROM.
 * These values will stay in the EEPROM when the board is
 * turned off and may be retrieved later by another sketch.
 */

#include <EEPROM.h>

// the current address in the EEPROM (i.e. which byte
// we're going to write to next)
int addr = 0;
int sk=0;
void setup()
{
}

void loop()
{
 if (sk==0){
  // need to divide by 4 because analog inputs range from
  // 0 to 1023 and each byte of the EEPROM can only hold a
  // value from 0 to 255.
//  int val = analogRead(0) / 4;

  EEPROM.write(200, 238); //1904
    EEPROM.write(201, 225); //1800
      EEPROM.write(202, 213); //1704
        EEPROM.write(203, 200); //1600
          EEPROM.write(204, 188);//1504
            EEPROM.write(205, 175);//1400
sk=1;
 }

  
  delay(100);
}
