# Change Log for Arduino Nano
Note there MAY be some information missing, see actual code comments for more info

## SD2_Nano_V0.1
- Combined Test Code:
  - Level_With_Serial
    - Changed the Actuation location logic from sin/cos axis length to vector addition
  - CM_Calculation
    - Angle calculation works but weightmass is a bit messed up still
  - Chatgpt6_WorseSpeedCtrl
    - Changed by giving each actuator proper proportioning for pitch and roll values
    - Angle acceleration compensation (alpha) actual works now
    - Magnitude of gravity calculation added for debug
- Added Serial Singular Commands:
  - start / stop (Begins / Stops Leveling)
  - LDon / LDoff (Shows / Hides Leveling debug)
  - BDon / BDoff (Shows / Hides Balancing debug)
  - calibrate (Sets current WA, WB, WC values to 0)
- Added Serial Variable Commands:
  - alpha"#" (sets alpha value)
  - minspeed"#" (sets min actuator speed value)
  - maxspeed"#" (sets max actuator speed value)
  - LDcycle"#" (shows Leveling debug for "#" of cycles)
  - BDcycle"#" (shows Balancing debug for "#" of cycles)

## SD2_Nano_V0.2
- Added Test Code:
  - ClocksPerCycle
  - Temp_SD_Laser (Added and Removed)
    - (Replaced With) No_Library_Servo_Laser
      - Basically this is code that inputs 0 to 360 degrees and turns a 180 degree servo and switches the laser to on the correct side
      - This recreates a pwm signal on a non-pwm pin and does not mess with the internal timers
      - This builds off No_Library_Servo
      - DO NOT USE Servo.h
        - The nano has 2 builtin timers (1&2) and for some reason Servo.h deactivates PWM on pins 9 and 10, aka actuator 1
- Added Mass Compensation
  - Takes the weight calculated on the load cells and adds voltage to increase the power to lift a tire while not over shooting velocity
- Added Serial Singular Commands:
  - Laseron / Laseroff (Enables / disables the laser and servo)
  - CPSon / CPSoff (Shows / Hides ClocksPerCycle)
  - MCon / MCoff (Enables / disables compensation for the actuators to get more voltage with more weight added)
- Added Serial Variable Commands:
  - RimD"#" (sets Rim Diameter value)
  - TreadW"#" (sets Tread Width value)
  - SideWallRatio"#" (sets the side wall percentage value)
  - TestMass"#" (Adds mass directly to WA,WB, and WC of the amount "#" in kg)
  - rollOffset"#" (Offsets the roll axis by "#" degrees)
  - pitchOffset"#" (Offsets the pitch axis by "#" degrees)
- Note:
  - Angle calculation works but weightmass is a bit messed up still
  - Since the servo code was rewritten, pulsewidth is not a linear corelation to degree angle
    - needs compensation still

## SD2_Nano_V0.3
- Added calibration for mpuBottom
- Added Base Angle calculation
  - under leveling code
  - error printed to serial if over MaxBaseAngle (13 degrees currently)
  - There is no reason to attempt to level if the actuators cannot reach their final destination
- Added BalanceState that controls when the average of WeightAngle and WeightMass is taken
  - Error message printed to serial and cancels Balancing if currently Leveling
- Added Serial Singular Commands:
  - Balance (Takes the average of WeightAngle and WeightMass)

## SD2_Nano_V0.4
- Added Laser Position Offset "LaserOffset" becasue the laser is about 90 degrees off from 0
  - Also fixed the accidental inversion of the laser angle
  - Added library Math.h to use "mod()"
- Also some serial bug fixes to get it to work with screen release "CYD_SD2_V0.1"
- Not going to lie this was worked on over a dual all nighter to get the screen code working before the final presentation
  - aka there is probably more that I added (Very Professional)
 
## SD2_Nano_V0.5
- Serial
  - Added "outputExponent" (Controls speed curve of the actuators) variable: "outputExponent"
  - Added TestMasson/off for debug
  - Added REon/REoff for ReportErrors on or off.
    - Currently just for the hx711 sensor errors, mainly for boot. OFF by default, turn ON for initial setup. Uses ReportErrorsState bool flag.
  - Commented out the delay(100); from loop
    - If REon is set then it will spam serial with error messages if the delay is not there
    - I think the hx711's are just slow? Anyway, the data read I've validated to still be accurate. So go figure if they say they aren't connected
    - Hopefully this might fix some esp and nano connection issues
  - Changed most rarly used serial writes to write from flash and not ram
    - SIGNIFICATLY reduces ram usage
    - Also fixed bugs where some serial writes would disappeear from hitting the ram cap
- Fixed TestMass Logic so it won't delete last sent TestMass amount using TestMassTemp
- Removed moment logic from ReadMass()
    - Apperently after testing and calibrating the load cells at a different location the the calibration value was very different from the previous calibration value. So after double checking, these load cells are not affected by the distance of force applied. They only count the mass applied in the vertical direction regardless of the force away from the sensor. I'd imagine that at some point this would become an issue at a further distance due to flex, but we are well within small angle approximation to not worry at 6 and under inches from the plate to the furthest measuring point.
- Added "Multi_HX711_Calibrate" to the test code
  - This gives you the values for Scale1Calibration, Scale2Calibration, and Scale3Calibration
- Fixed the Moment Balance Logic
- Fixed the RimD and TheadW so it actually updates with serial commands
- Removed XWeight,YWeight as they were bad duplicates of the centroid calculation
- Added outRangeWeight, inRangeWeight, %inRangeWeight, and inOverout
  - Basically this says how much weight is on the same side as the CMAngle
  - This is important because the center of mass calculation for how much weight to be needs to be added needs to be based on the mass difference of the side with the most weight and its opposite side
    - This cancels out the mass that would be balanced without the extra unbalancing weight, leaving only the mass that is needed to counter
  - ~This means that all past versions works for testing point masses but not practical tire masses~
  - ~TLDR old code not work, new code "should" be more acurate for all scenarios, needs testing but theory works~
  - Ok so after some testing with this new setup I've relized how I originally coded this (Sorry 4 month break). So these new values are cool to have but its just the same data we already had just in a different more understandable fasion. It does take more processing time to solve these new values but its really a debate on how accesable I want to make this for modifications. Likely, I will leave it in but comment it out when I get it working how I like.
  - By fixing the moment logic from ReadMass(), it also fixed a bug in how these values are calculated since that is the step before this calculation
  - inRangeWeight is basically CMweight
