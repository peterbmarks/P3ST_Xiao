// Arduino "sketch" for use with the P3ST Xiao Digital VFO/BFO (v.2 -- 3.26.24)
// (C) T.F. Carney (K7TFC). For use under the terms of the
// Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International license.
// See https://creativecommons.org/licenses/by-nc-sa/4.0/legalcode.en
// This version modified by Peter Marks VK3TPM
/*
  The frequencies we wish to use are 14Mhz+
  The IF strip is at 4.915Mhz
  The VFO outputs 14 - 4.915 = 9.085Mhz
  The BFO outputs IF - a bit = 4.9175Mhz (to demod USB)
*/

#include "xiaoRP2040pinNums.h"  // Keep this library above the #defines.

#define NOP __asm__ __volatile__ ("nop\n\t")  // NOP needed to follow "skip" labels.
#define i2cSDA D4
#define i2cSCL D5
#define encoderButton D2
#define encoderA D0
#define encoderB D1
#define kLCDI2cAddress 0x27 // as supplied
//#define kLCDI2cAddress 0x3e // Waveshare LCD1602
//#define kLCDI2cAddress 0x3f // EONE LCD

#define kSi5351i2cAddress 0x60

//#define SERIAL_DEBUG  // uncomment this to send serial info

//========================================
//=============  LIBRARIES ===============
//========================================
#include <EEPROM.h>               // Author: Earle Philhower and Ivan Grokhotkov. Emulates EEPROM in program-flash 
                                  // memory. This library is part of the RP2040 board-manager package at:
                                  // ~/.arduino15/packages/rp2040/hardware/rp2040/3.1.0/libraries/EEPROM.
                                  // Do not confuse with libraries of the same name for other architectures.  
#include <Wire.h>
#include <LiquidCrystal_I2C.h>    // Author: Schwartz. But credited Frank de Brabander in Arduino Library
#include <Rotary.h>               // Author: Ben Buxton. From https://github.com/buxtronix/arduino/tree/master/libraries/Rotary 
                                  // Manually put this in your Arduino/library folder
#include "PinButton.h"            // Author: Martin Poelstra (part of MultiButton library with 
                                  // long press modified by K7TFC to 1000ms).
#include <si5351.h>               // Author: Jason NT7S. V.2.1.4.

//========================================
//======== CONSTANTS ===========
//======================================== 
// EEPROM addresses for settings
const int kInitedMagicAddress = 0;  // look for a magic number to determine if EEPROM values have been initialized
const int kCalFactorAddress = 5;
const int kDisplayOffsetAddress = 10;
const int kLastUsedBFOAddress = 15;
const int kLastUsedVFOAddress = 20;
const int kInitedMagicNumber = 1239; // magic number to look for to determine if initial values have been stored
const uint32_t kCalibrationOffset = 10000;  // used to avoid negative calibration factor storage
//========================================
//======== GLOBAL DECLARATIONS ===========
//======================================== 
int32_t gCalibrationFactor = 0;       //////*** DEFAULT VALUE. CHANGE FOR A DIFFERENT CALIBRATION FACTOR AS FOUND FROM
                                   //////*** THE ETHERKIT Si5351_calibration.ino SKETCH AS FOUND IN THE
                                   //////*** ARDUINO IDE: File/Examples/Etherkit Si5351. *OR* USE 

uint32_t gLastUsedVFO = 9187000;  // Default to 14.1Mhz for convenience
uint32_t gDisplayOffset = 4913000;    

uint32_t gLastUsedBFO = 4912380;    // Starting value. Later read from "EEPROM"
int kSteps[] = {10,100,1000,10000}; // Tuning kSteps to increment frequency (in Hz) each encoder detent.
int gStep = 1000;                   // Step on startup. THIS *MUST* REMAIN A REGULAR *SIGNED* INTEGER!

//========================================
//============ INSTANTIATIONS ============
//========================================
Si5351 si5351;
LiquidCrystal_I2C lcd(kLCDI2cAddress, 16, 2);
Rotary tuningEncoder = Rotary(D0, D1);
PinButton button(encoderButton);

////========================================
////******** FUNCTION: setup ***************
////========================================
void setup() {
  bool si5351_found = false;
  #ifdef SERIAL_DEBUG
  Serial.begin(115200);
  #endif

  Wire.begin();
  /*
  Todd wrote: 
  I don't know what the default is, but I think it's 400KHz. 
  The Philhower board package for the RP2040 (and its implementation of Wire) 
  may have the default set too high. I think it can be set as low as 10KHz. 
  I had to monkey with it once when dealing with a PCF8574 bus expander. 
  I think I had to take it down to 100KHz. By the way, the problem I was 
  having with the bus expander was also intermittent. Sometimes it worked, sometimes not. 
  Reducing the frequency of the I2C clock made it reliable.

  */
  Wire.setClock(100000);  // trying to avoid intermittent startup failure with si5351

  lcd.init();
  lcd.backlight();

  #ifdef SERIAL_DEBUG
  Serial.println("P3ST starting up...");
  i2cScan();
  #endif

  EEPROM.begin(256);
  delay(500); // give a bit of time before talking to the si5351

  si5351_found = si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
  if(!si5351_found)
  {
    #ifdef SERIAL_DEBUG
    Serial.println("Device not found on I2C bus!");
    #endif
  }
  //////////////////////////////////////
  setupInitialValues(); // read from EEPROM or set initial values if not already stored

  displayFreqLine(0, gLastUsedVFO + gDisplayOffset);  //Parameters: LCD line (0 or 1), frequency value.
  lcd.setCursor(0, 1);
  lcd.print("P3ST.");
  displayTuningStep(gStep, 1);      //Parameters: displayTuningStep(int Step, byte lineNum)

} // End of setup()

//========================================
//********* FUNCTION: (main)loop *********
//========================================
void loop() {
  unsigned char encoder;
  int counter = 0;

  // Read tuning encoder and set Si5351 accordingly
  /////////////////////////////////////////////////
  button.update();
  encoder = tuningEncoder.process();  

// Button activity on tuning encoder          
   if (button.isLongClick()) {                           
    bfoFreq();                // Long press-and-release will call BFO-setting function.
   } else if(button.isDoubleClick()) {
      saveVFO();  // don't do this too many times
      si5351CorrectionFactor();
   } else if(button.isSingleClick() && gStep == kSteps[3]) {    // These else-if statements respond to single (short click) button pushes to step-through   
      gStep = kSteps[0];                          // the tuning increments (10Hz, 100Hz, 1KHz, 10KHz) for each detent of the tuning encoder.
      displayTuningStep(gStep, 1);               // A short click on 10KHz loops back to 10Hz. The default step is 1KHz.
   } else if (button.isSingleClick() && gStep == kSteps[0]) {
      gStep = kSteps[1];
      displayTuningStep(gStep, 1);
  } else if (button.isSingleClick() && gStep == kSteps[1]) {
      gStep = kSteps[2];
      displayTuningStep(gStep, 1);
    } else if (button.isSingleClick() && gStep == kSteps[2]) {
      gStep = kSteps[3];
      displayTuningStep(gStep, 1);
   }

   uint32_t vfoValue = gLastUsedVFO;

// Skip to end of loop() unless there's change on either encoder or button
  // so LCD and Si5351 aren't constantly updating (and generating RFI).
  if (!encoder && !button.isClick()) {
    goto skip;
  } else if (encoder == DIR_CCW) {
    counter++;
  } else if (encoder == DIR_CW) {
    counter--;
  }
  vfoValue += (counter * gStep);
  #ifdef SERIAL_DEBUG
  Serial.print("set VFO freq CLK0: ");
  Serial.print(vfoValue);
  Serial.print(" (");
  Serial.print(vfoValue + gDisplayOffset);
  Serial.println("MHz)");
  #endif

  si5351.set_freq(vfoValue * SI5351_FREQ_MULT, SI5351_CLK0);  // Si5351 is set in 0.01 Hz increments. "vfoValue" is in integer Hz.
  gLastUsedVFO = vfoValue;

  // LCD display ///////////////////
  displayFreqLine(0,gLastUsedVFO + gDisplayOffset);
  displayTuningStep(gStep, 1);
 
  skip:   // This label is where the loop goes if there are no inputs.
  NOP;    // C/C++ rules say a label must be followed by something. This "something" does nothing.
}  // closes main loop() 

void setupInitialValues() {
  uint32_t initedMagicNumberValue = readUint32(kInitedMagicAddress);
  #ifdef SERIAL_DEBUG
  Serial.print("Read magic number = ");
  Serial.println(initedMagicNumberValue);
  #endif

  if(initedMagicNumberValue != kInitedMagicNumber) {
    #ifdef SERIAL_DEBUG
    Serial.println("##### Initializing EEPROM stored values");
    Serial.print("Initializing gCalibrationFactor = ");
    Serial.println(gCalibrationFactor);
    #endif

    saveUint32(kCalFactorAddress, gCalibrationFactor + kCalibrationOffset);  // 10000 padding added to prevent underflow for negative cal factors.
    #ifdef SERIAL_DEBUG
    Serial.print("Initializing gDisplayOffset = ");
    Serial.println(gDisplayOffset);
    #endif
    saveUint32(kDisplayOffsetAddress, gDisplayOffset);       // Saves default value in eeprom.

    Serial.print("Initializing gLastUsedBFO = ");
    Serial.println(gLastUsedBFO);
    saveUint32(kLastUsedBFOAddress, gLastUsedBFO);    // Saves default value in eeprom.
    #ifdef SERIAL_DEBUG
    Serial.print("Initializing gLastUsedVFO = ");
    Serial.println(gLastUsedVFO);
    #endif
    saveUint32(kLastUsedVFOAddress, gLastUsedVFO);    // Saves default value in eeprom.

    #ifdef SERIAL_DEBUG
    Serial.println("Writing magic number..");
    #endif

    saveUint32(kInitedMagicAddress, kInitedMagicNumber);  // write magic number so we know we've inited
  }
  // Read stored values from EEPROM
  gCalibrationFactor = readUint32(kCalFactorAddress) - kCalibrationOffset;
  #ifdef SERIAL_DEBUG
  Serial.print("Read calibration factor = ");
  Serial.println(gCalibrationFactor);
  #endif
  si5351.set_correction(((gCalibrationFactor) * SI5351_FREQ_MULT), SI5351_PLL_INPUT_XO); 
  gLastUsedBFO = readUint32(kLastUsedBFOAddress);
  /*
  Frequencies are indicated in units of 0.01 Hz. 
  Therefore, if you prefer to work in 1 Hz increments in your own code, 
  simply multiply each frequency passed to the library by 100ULL 
  (better yet, use the define called SI5351_FREQ_MULT in the header file for this multiplication).
  */
  #ifdef SERIAL_DEBUG
  Serial.print("set BFO freq CLK2: ");
  Serial.println(gLastUsedBFO);
  #endif
  si5351.set_freq(gLastUsedBFO * SI5351_FREQ_MULT, SI5351_CLK2);

  gLastUsedVFO = readUint32(kLastUsedVFOAddress);
  #ifdef SERIAL_DEBUG
  Serial.print("set VFO freq CLK0: ");
  Serial.print(gLastUsedVFO);
  Serial.print(" (");
  Serial.print(gLastUsedVFO + gDisplayOffset);
  Serial.println("MHz)");
  #endif
  si5351.set_freq(gLastUsedVFO * SI5351_FREQ_MULT, SI5351_CLK0);

  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);
  si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_8MA);
}

int counter = 0;

// experimental interrupt handler
void rotate() {
  unsigned char result = tuningEncoder.process();
  if (result == DIR_CW) {
    counter++;
    #ifdef SERIAL_DEBUG
    Serial.println(counter);
    #endif
  } else if (result == DIR_CCW) {
    counter--;
    #ifdef SERIAL_DEBUG
    Serial.println(counter);
    #endif
  }
}
////========================================
////***** FUNCTION: lcdClearLine ***********
////========================================
void lcdClearLine(byte lineNum) {
    lcd.setCursor(0,lineNum);
    lcd.print("                ");
    lcd.setCursor(0,lineNum);
}

////========================================
////***** FUNCTION: displayFreqLine ********  
////========================================
void displayFreqLine(byte lineNum, uint32_t freqValue) {
  String valueStr;
  String mhzOnly;
  String khzOnly;
  String hzOnly;
  
  if(lineNum == 0) {
    lcd.setCursor(13, 0);
    lcd.print("USB");
  }
    
  valueStr = String(freqValue);
  if (valueStr.length() == 8) {
    mhzOnly = valueStr.substring(0, 2);
    khzOnly = valueStr.substring(2,5);
    hzOnly = valueStr.substring(5,8);
  } else if (valueStr.length() == 7) {
    mhzOnly = valueStr.substring(0,1);
    khzOnly = valueStr.substring(1,4);
    hzOnly = valueStr.substring(4,7);
  }
                                                                            
   if (lineNum == 0) {
     lcd.setCursor(0, lineNum); 
   }

  if(valueStr.length() == 8) {
    lcd.print(mhzOnly + "." + khzOnly + "." + hzOnly);  // For frequencies >=10,000KHz (no leading blank space).
  } else {
    lcd.print(mhzOnly + "." + khzOnly + "." + hzOnly);  // For frequencies <=9,999KHz (adds leading blank space).
  }
} // End displayFreqLine()

void saveVFO() {
  saveUint32(kLastUsedVFOAddress, gLastUsedVFO); 
  #ifdef SERIAL_DEBUG
  Serial.println("Saved VFO to EEPROM");
  #endif
}

void setCursorForTuningStep(int step) {
  switch(step) {
    case 10:
      lcd.setCursor(8, 0);
      break;
    case 100:
      lcd.setCursor(7,0);
      break;
    case 1000:
      lcd.setCursor(5, 0);
      break;
    case 10000:
      lcd.setCursor(4, 0);
      break;
    case 100000:
      lcd.setCursor(3, 0);
      break;
  }
  lcd.cursor_on();
}
////========================================
////***** FUNCTION: displayTuningStep ******  
////========================================
void displayTuningStep(int step, byte lineNum) {
  switch (step) {
    case 10:
      lcd.setCursor(11, lineNum);
      lcd.print("     ");
      lcd.setCursor(12, lineNum);
      lcd.print("10Hz");
      lcd.setCursor(0, 1);
      lcd.print("P3ST");
      break;
    case 100:
      lcd.setCursor(11, lineNum);
      lcd.print("     ");
      lcdClearLine(lineNum);
      lcd.setCursor(11, lineNum);
      lcd.print("100Hz");
      lcd.setCursor(0, 1);
      lcd.print("P3ST");
      break;
    case 1000:
      lcd.setCursor(11, lineNum);
      lcd.print("     ");
      lcdClearLine(lineNum);
      lcd.setCursor(12, lineNum);
      lcd.print("1KHz");
      lcd.setCursor(0, 1);
      lcd.print("P3ST");
      break;
    case 10000:
      lcd.setCursor(11, lineNum);
      lcd.print("     ");
      lcdClearLine(lineNum);
      lcd.setCursor(11, lineNum);
      lcd.print("10KHz");
      lcd.setCursor(0, 1);
      lcd.print("P3ST");
      break;  
  } // End of switch-case
  setCursorForTuningStep(step);
} //End displayTuningStep()

////========================================
////***** FUNCTION: setDisplayOffset() *****
////========================================
void setDisplayOffset() {
  int encoder;
  int counter = 0;
  uint32_t offsetValue = readUint32(kDisplayOffsetAddress);

  lcdClearLine(1);

  lcd.setCursor(0,1);
  lcd.print("Offset ");
  displayFreqLine(1,offsetValue);
  //lcd.setCursor(9, 1);
  //lcd.print(offsetValue);
  button.update();
  
  while (!button.isSingleClick()) {
    //counter = 0;
    encoder = tuningEncoder.process();

    if (encoder == DIR_CCW) {
      counter++;
    }
    else if (encoder == DIR_CW) {
      counter--;
    }

    button.update();

    if (counter == 0) {   //Skip remainder of loop if there's no change in either encoder or button
      continue;
    } else {
      offsetValue += (counter * kSteps[0]);
      counter = 0;
      lcd.setCursor(7, 1);
      displayFreqLine(1,offsetValue);
    }
  }

  gDisplayOffset = offsetValue;
  saveUint32(kDisplayOffsetAddress, gDisplayOffset);
  
  // reset BFO display
    lcdClearLine(0);
    lcdClearLine(1);
    displayTuningStep(kSteps[0], 0);
    lcd.setCursor(0, 1);
    lcd.print("BFO:");
    lcd.setCursor(7, 1);
    displayFreqLine(1, gLastUsedBFO);  

    return;
}

////========================================
////******** FUNCTION: bfoFreq *********
////========================================
void bfoFreq() { 
    button.update();

    int bfoStep = kSteps[0];
    uint32_t bfoValue = gLastUsedBFO;
    int counter;

    lcdClearLine(0);
    lcdClearLine(1);
    displayTuningStep(bfoStep, 0);
    lcd.setCursor(0, 1);
    lcd.print("BFO:");
    lcd.setCursor(7, 1);
    displayFreqLine(1, bfoValue);  

  while (!button.isLongClick()) {       // Loop until a long press-and-release of encoder button
    unsigned char encoder;
    counter = 0;
    button.update();

  if (button.isSingleClick()) {
      setDisplayOffset();
  }

    // Read tuning encoder and set Si5351 accordingly
    /////////////////////////////////////////////////
    encoder = tuningEncoder.process();
    if (encoder == DIR_CCW) {
    counter++;
    }
    else if (encoder == DIR_CW) {
    counter--;
    }
 
    // Skip to end of loop() unless there's change on either encoder or button
    // so LCD and Si5351 aren't constantly updating (and generating RFI).
    if (counter == 0 && !button.isClick()) {
      goto skip;
    } else {
      bfoValue += (counter * bfoStep);
      gLastUsedBFO = bfoValue;

      // move the VFO the opposite direction to the BFO change
      gLastUsedVFO -= (counter * bfoStep);
      si5351.set_freq(gLastUsedVFO * SI5351_FREQ_MULT, SI5351_CLK0);
      #ifdef SERIAL_DEBUG
      Serial.print("Set VFO to: ");
      Serial.println(gLastUsedVFO);
      #endif
    }
    #ifdef SERIAL_DEBUG
    Serial.print("set BFO freq CLK2: ");
    Serial.println(gLastUsedBFO);
    #endif
    si5351.set_freq(gLastUsedBFO * SI5351_FREQ_MULT, SI5351_CLK2); //BFO frequency set within the loop for real-time adjustment.
    lcd.setCursor(7, 1);
    displayFreqLine(1,gLastUsedBFO);  //Parameters: LCD line (0 or 1), frequency value.

    skip: 
    NOP;
  } // End of while loop.
  
  // At this point (after long press-and-hold of encoder button),
  // save new BFO freq to "EEPROM" and restore the VFO display before returning.

  gLastUsedBFO = bfoValue;
  saveUint32(kLastUsedBFOAddress, gLastUsedBFO);  

  lcdClearLine(0);
  lcdClearLine(1);
  lcd.setCursor(0,1);
  displayFreqLine(0,gLastUsedVFO + gDisplayOffset);  
   lcd.setCursor(0, 1);
  lcd.print("P3ST");
  displayTuningStep(gStep, 1);

  return;
} // End of bfoFreq() 

//========================================
//*** FUNCTION: si5351CorrectionFactor ***
//========================================
void si5351CorrectionFactor() {

  int correctionStep = kSteps[0];
  int counter;
  float currentVFOnum = float(gLastUsedVFO);
  uint32_t correctionFactor = 0;
  float correctionFactorRaw = 0;
  float correctionFactorFloat;

  String sp10 = "          ";

  lcdClearLine(0);
  lcd.print("Set known Freq:");
  lcdClearLine(1);
  displayFreqLine(1, currentVFOnum + gDisplayOffset);    
  
  button.update();
  while (!button.isDoubleClick()) {
     unsigned char encoder;
    counter = 0;
    button.update();
      
    encoder = tuningEncoder.process();
    if (encoder == DIR_CCW) {
    counter++;
    }
    else if (encoder == DIR_CW) {
    counter--;
    }
 
    if (counter == 0 && !button.isClick()) {
      continue;
    }
    else {
      currentVFOnum += (counter * correctionStep);
    }

   lcd.setCursor(0, 1);
   displayFreqLine(1,currentVFOnum + gDisplayOffset);  
    
    //button.update();
  } // end while loop


  correctionFactorRaw = ((float(gLastUsedVFO) - currentVFOnum) * -1);
  correctionFactorFloat = correctionFactorRaw / currentVFOnum;
  correctionFactor = int(correctionFactorFloat * 10000000);   // Multiplies up toward parts-per-billion

  si5351.set_correction(correctionFactor * SI5351_FREQ_MULT, SI5351_PLL_INPUT_XO);   // x100 to parts-per-billion
  gLastUsedVFO = currentVFOnum;
  saveUint32(kCalFactorAddress, correctionFactor + 10000);  // Pad correctionFactor with 10000 to prevent integer underflow
                                          // of negative values.
  //Now fully restore normal VFO display
  lcdClearLine(0);
  displayFreqLine(0,gLastUsedVFO + gDisplayOffset);
  lcd.setCursor(13,0); lcd.print("USB");
  lcd.setCursor(0, 1); lcd.print(sp10); lcd.setCursor(0, 1); lcd.print("P3ST");
  displayTuningStep(gStep, 1);
}


////===================================== 
////******* FUNCTION: saveInt ********
////=====================================
void saveInt(int address, int number) {
  EEPROM.write(address, number >> 8);
  EEPROM.write(address + 1, number & 0xFF);
}

//////===================================== 
//////******* FUNCTION: readInt ********
//////=====================================
int readInt(int address) {
  byte byte1 = EEPROM.read(address);
  byte byte2 = EEPROM.read(address + 1);
  return (byte1 << 8) + byte2;
}

////===================================== 
////******* FUNCTION: saveUint32 ********
////=====================================
void saveUint32(int address, uint32_t number) {
  EEPROM.write(address, (number >> 24) & 0xFF);      // These lines encode the 32-bit variable into
  EEPROM.write(address + 1, (number >> 16) & 0xFF);  // 4 bytes (8-bits each) and writes them to
  EEPROM.write(address + 2, (number >> 8) & 0xFF);   // consecutive eeprom bytes starting with the
  EEPROM.write(address + 3, number & 0xFF);          // address byte. Using write() instead of update()
  EEPROM.commit();
}                                                    // to save time and because update() won't help
                                                     // save write cycles for *each* byte of the 4-byte blocks.
////===================================
//// ***** FUNCTION: readUint32 *******
///===================================
uint32_t readUint32(int address) {
  return ((uint32_t)EEPROM.read(address) << 24) +      // These lines decode 4 consecutive eeprom bytes (8-
         ((uint32_t)EEPROM.read(address + 1) << 16) +  // bits each) into a 32-bit variable (starting with the
         ((uint32_t)EEPROM.read(address + 2) << 8) +   // address byte) and returns to the calling statement.
         (uint32_t)EEPROM.read(address + 3);           // Example: uint32_t myNumber = readUint32(0);
}

// Scan the i2c bus (only if SERIAL_DEBUG is defined)
// From: https://playground.arduino.cc/Main/I2cScanner/
void i2cScan() {
  byte error, address;
  int nDevices;
 
  Serial.println("Scanning I2C...");
 
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("");
 
      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address, HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("i2c scan done\n");
}
