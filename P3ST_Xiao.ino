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
#define kLCDI2cAddress 0x27
#define kSi5351i2cAddress 0x60

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
const int kInitedMagicNumber = 1234; // magic number to look for to determine if initial values have been stored

//========================================
//======== GLOBAL DECLARATIONS ===========
//======================================== 
uint32_t gCalibrationFactor = 160100;       //////*** DEFAULT VALUE. CHANGE FOR A DIFFERENT CALIBRATION FACTOR AS FOUND FROM
                                   //////*** THE ETHERKIT Si5351_calibration.ino SKETCH AS FOUND IN THE
                                   //////*** ARDUINO IDE: File/Examples/Etherkit Si5351. *OR* USE 
                                   
uint32_t gLastUsedVFO = 9085000;
uint32_t gDisplayOffset = 4915000;    

uint32_t gLastUsedBFO = 4917500;    // Starting value. Later read from "EEPROM"
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
  Serial.begin(115200);

  lcd.init();
  lcd.backlight();
  Wire.begin();

  Serial.println("P3ST starting up...");
  i2cScan();

  EEPROM.begin(256);
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);
  si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_8MA);
  //////////////////////////////////////
  setupInitialValues(); // read from EEPROM or set initial values if not already stored

  displayFreqLine(0, gLastUsedVFO + gDisplayOffset);  //Parameters: LCD line (0 or 1), frequency value.
  displayTuningStep(gStep, 1);      //Parameters: displayTuningStep(int Step, byte lineNum)
  lcd.setCursor(0, 1);
  lcd.print("P3ST.");
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
  Serial.print("set VFO freq CLK0: ");
  Serial.println(vfoValue);
  si5351.set_freq(vfoValue * SI5351_FREQ_MULT, SI5351_CLK0);  // Si5351 is set in 0.01 Hz increments. "vfoValue" is in integer Hz.
  gLastUsedVFO = vfoValue;

  // LCD display ///////////////////
  displayFreqLine(0,gLastUsedVFO + gDisplayOffset);
  //displayTuningStep(gStep, 1);
 
  skip:   // This label is where the loop goes if there are no inputs.
  NOP;    // C/C++ rules say a label must be followed by something. This "something" does nothing.
}  // closes main loop() 

void setupInitialValues() {
  uint32_t initedMagicNumberValue = readUint32(kInitedMagicAddress);
  Serial.print("Read magic number = ");
  Serial.println(initedMagicNumberValue);
  if(initedMagicNumberValue != kInitedMagicNumber) {
    Serial.println("##### Initializing EEPROM stored values");
    Serial.print("Initializing gCalibrationFactor = ");
    Serial.println(gCalibrationFactor + 10000);
    saveUint32(kCalFactorAddress, gCalibrationFactor + 10000);  // 10000 padding added to prevent underflow for negative cal factors.

    Serial.print("Initializing gDisplayOffset = ");
    Serial.println(gDisplayOffset);
    saveUint32(kDisplayOffsetAddress, gDisplayOffset);       // Saves default value in eeprom.

    Serial.print("Initializing gLastUsedBFO = ");
    Serial.println(gLastUsedBFO);
    saveUint32(kLastUsedBFOAddress, gLastUsedBFO);    // Saves default value in eeprom.

    Serial.println("Writing magic number..");
    saveUint32(kInitedMagicAddress, kInitedMagicNumber);  // write magic number so we know we've inited
  }
  // Read stored values from EEPROM
  gCalibrationFactor = readUint32(kCalFactorAddress);
  si5351.set_correction(((gCalibrationFactor - 10000) * SI5351_FREQ_MULT), SI5351_PLL_INPUT_XO); 
  gLastUsedBFO = readUint32(kLastUsedBFOAddress);
  /*
  Frequencies are indicated in units of 0.01 Hz. 
  Therefore, if you prefer to work in 1 Hz increments in your own code, 
  simply multiply each frequency passed to the library by 100ULL 
  (better yet, use the define called SI5351_FREQ_MULT in the header file for this multiplication).
  */
  Serial.print("set BFO freq CLK2: ");
  Serial.println(gLastUsedBFO);
  si5351.set_freq(gLastUsedBFO * SI5351_FREQ_MULT, SI5351_CLK2);

  Serial.print("set VFO freq CLK0: ");
  Serial.println(gLastUsedVFO);
  si5351.set_freq(gLastUsedVFO * SI5351_FREQ_MULT, SI5351_CLK0);
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

////========================================
////***** FUNCTION: displayTuningStep ******  
////========================================
void displayTuningStep(int Step, byte lineNum) {
  switch (Step) {
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
    }
    Serial.print("set BFO freq CLK2: ");
    Serial.println(gLastUsedBFO);
    si5351.set_freq(gLastUsedBFO * SI5351_FREQ_MULT, SI5351_CLK2); //BFO frequency set within the loop for real-time adjustment.
    lcd.setCursor(7, 1);
    displayFreqLine(1,bfoValue);  //Parameters: LCD line (0 or 1), frequency value.

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
  displayTuningStep(gStep, 1);
  lcd.setCursor(0, 1);
  lcd.print("P3ST");
 
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

// Scan the i2c bus
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
