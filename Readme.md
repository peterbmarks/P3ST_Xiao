# P3ST Firmware

The original is from Todd Carney and can be found [here](https://github.com/mostlydiyrf/p3st).

The kit is available from [Mostly DIY RF](https://mostlydiyrf.com/p3st/).

This version is currently under development by Peter Marks, VK3TPM and is stored [here](https://github.com/peterbmarks/P3ST_Xiao). 

Discussion about the project is on [groups.io](https://groups.io/g/pssst-20/messages).

## Changes by Peter

* Print debugging to Serial
* Better method for deciding to initialise non volatile stored values
* Use constants rather than values in the code
* Minor formatting changes to my taste
* Took out the fancy RGB led display during startup. I think the LCD display is enough.
* Underline with a cursor the digit which will change when you tune
* When adjusting the BFO adjust the VFO the same amount in the other direction

## Notes

* Arduino IDE 2 is needed I think. I'm currently on 2.3.2
* Set board to Raspberry Pi Pico -> Seeed XIAO RP2040
* Plug the board in to USB while holding the boot button the first time.
* Choose "UF2 Port" for the serial port
* I've added a built binary in the Binary folder. If you plug the board in with the boot button down it will mount as a disk drive. Drag the .uf2 file to it to program.

## Serial debug message example

### First run
    P3ST starting up...
    Scanning...
    I2C device found at address 0x27  !
    I2C device found at address 0x60  !
    i2c scan done
    
    Read magic number = 4294967295
    ##### Initializing EEPROM stored values
    Initializing gCalibrationFactor = 170100
    Initializing gDisplayOffset = 4915000
    Initializing gLastUsedBFO = 4917500
    set BFO freq CLK2: 491750000
    set VFO freq CLK0: 908500000

### Subsequent run

    P3ST starting up...
    Scanning...
    I2C device found at address 0x27  !
    I2C device found at address 0x60  !
    i2c scan done
    
    Read magic number = 1234
    set BFO freq CLK2: 491750000
    set VFO freq CLK0: 908500000

## Future enhancement ideas

* Interrupt driven rotary handling - I find it misses steps if I tune too fast
* Calibration - I see it’s there but don’t think it works?
* Store last used VFO (without wearing out storage) [I’ve put in a line that stores it if you double click but not obvious] Perhaps a menu item to do this?
* Support for other popular LCD modules. I scan the I2C bus on startup and could use this to recognise what’s connected.
* I’d love to do an S meter but that would need a hardware patch. There are GPIO pins available that could do the A/D
* Sometimes the Si5351 doesn't initialise correctly on power on. Perhaps a delay is needed?

## Original Readme below

The firmware included with the P3ST kit (as loaded on the **Xiao RP2040**) was written in C/C++ for use with the Arduino IDE and with the **Raspberry Pi RP2040 Boards (3.1.0)** "package" installed through the Boards Manager (Menu: **Tools/Board/Board Manager**). 

The libraries for use with the program (a.k.a., "sketch") are available using the **Tools/Manage Libraries** utility in the IDE except for the **MultiButton** and **xiaoRP2040pinNums** libraries included in this repository. Download and install those manually (see <https://learn.adafruit.com/adafruit-all-about-arduino-libraries-install-use/how-to-install-a-library>).

The Xiao can also be programmed in MicroPython or CircuitPython. More information on doing so can be found at <https://wiki.seeedstudio.com/XIAO-RP2040-with-MicroPython/>. MDRF is not able to provide any support regarding the use of python with the Xiao.

## Firmware Version 2
A number of bug fixes and improvements have been incorporated in V.2. You are strongly urged to download and install it if you haven't already edited P3ST_Xiao to your liking. You will need to use the Arduino IDE (provisioned as indicated above) to do so.

The new version includes the following features.

* **VFO starting frequency** set to 9085MHz with a +4.915MHz display offset. This start-up default will display as "14.000.000"
  
   ![LCD normal](https://mostlydiyrf.com/wp-content/uploads/2024/03/LCD_normal.jpg)
* **Changable tuning steps**: 10Hz, 100Hz, 1KHz, and 10KHz. A single (short) click on the rotary-encoder "button" will cycle through the steps. The step increment is shown in the lower-right-hand corner of the LCD display.
* **Ability to change BFO frequency** in real time. Access BFO tuning by pressing and holding the encoder button for one second. The display will switch to BFO tuning mode.The tuning increment is fixed at 10Hz. You will hear the change in the receiver as you tune the BFO up and down. Press and hold the button a second time to save the new BFO frequency in EEPROM and return to the regular display.

   ![LCD BFO](https://mostlydiyrf.com/wp-content/uploads/2024/03/LCD_bfo.jpg)

  For more on the proper placement of the BFO frequency in SSB transceivers, see videos by **VK3YE** (<https://youtu.be/sXJmAhpAjeI?si=G-n8YnkCASo_OiY>, **Farhan** (<https://youtu.be/t6LGXhS4_O8?si=mpzQzo4UJICHqtBc>), **ZL2CTM** (<https://youtu.be/2FbMPRpvZh8?si=mC00r52cYEJh7q_Y>), and **Level UP EE Lab** (<https://youtu.be/6aMZeNEG9s8?si=9wfS2dIyOJ_XF6zv>).
* **Ability to change the display offset** to account for the difference between the VFO frequency and the intended frequency of operation (a.k.a., IF offset). For instance, to receive or transmit a signal at 14.015MHz, the VFO has to be set to 9.10MHz to be mixed with the 4.915MHz BFO/carrier frequency. No one wants to see the actual VFO frequency displayed--they want to see the frequency of operation.

   For this reason, an offset equal to the IF frequency is used for display purposes, and it needs to be adjustable because the precise IF frequency is determined by the center of the crystal filter passband--each one is a little different owing to component tolerances.

  To change the display offset, press and hold the encoder button to bring up the BFO tuning. From there, a single click on the button will bring up the current offset. Change it with the tuning knob. A single click will then save the new offset to EEPROM and return to the BFO display. From there, a long click-and-hold will take you back to normal operation.

  ![LCD offset](https://mostlydiyrf.com/wp-content/uploads/2024/03/LCD_offset.jpg)

* **Ability to change the Si5351 correction factor**. As implemented here, the Si5351 time base is an external crystal built into the module. In this case its nominal frequency is 25MHz, but of course its actual frequency will be a little off from that. A correction factor can be applied in firmware (ultimately in parts-per-billion) to compensate, but that factor will be different from one Si5351 module to another. 

   By default, the factor is set to 160000 (in 0.01Hz resolution) which seems to be average for the modules the P3ST uses. Without the correction, a desired signal of 10MHz, for instance, would actually be outputted as 10.001600MHz, or 1.6KHz off. The correction factor fixes that. But the 160000 factor is only an average. You milage **will** vary. As part of the process of aligning the frequency display, you'll have to change the factor saved in firmware.

   To do so, using the regular tuning display, first tune in to a known frequency. This could be a beacon, a net known to operate on a specific frequency, another transceiver operating at a specific frequency, or even a signal generator fed into a dummy load (as long as you can hear and zero-beat it on the P3ST).
   
   The display on the P3ST will almost certainly **not** show the exact frequency you're tuned to. It could show higher or lower than the known frequency. At this point, double-click on the tuning-knob button. The display will change to allow you to dial in the frequency you know you're actually tuned to.

  ![LCD BFO](https://mostlydiyrf.com/wp-content/uploads/2024/03/LCD_correction.jpg)
  
   **Example**: You've tuned to a signal you know is at 14.175 but your regular tuning display shows 14.176.100. Now double-click to enter the correction display and then dial down to 14.175.000. This is the frequency you know you're actually tuned to. Now double-click again. You'll be returned to the regular tuning display, and it will show the correct frequency. The correction factor has been calculated and saved to EEPROM. Subsequent tuning should agree with the display (but see below for caveat).
* **Caveat**: It's clear that there is an interplay between the settings for the BFO frequency, the display offset, and the correction factor. As one changes, the others need to be changed as well. This is typical of the radio-alignment process: one thing affects the other thing which then affects the first thing. Inevitably, there's an **interative** process--going from one adjustment to the next and then cycling back through them until everything is right, each iteration getting closer and closer.

   Since everything seems to depend on the actual center frequency of the crystal filter (which for all practical purposes can't be changed), finding that and setting the BFO frequency accordingly would be the first step. See the videos mentioned above for ideas on how to do that. Once the BFO frequency is known, then the dance is only between the display offset and the Si5351 correction factor.
   
   Since all these settings are now accessible from the "front panel," going back and forth until everything is (close enough to) dead-on should take only a little time, and once set everything is saved to EEPROM for use in susequent operating sessions. If you find after some use that one or more of the settings are off, you can change them on the fly.
