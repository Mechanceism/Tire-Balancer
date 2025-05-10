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
  - Added "outputExponent" (Controls speed curve of the actuators)
  - Added TestMasson/off for debug
- Fixed TestMass Logic so it won't delete last sent TestMass amount using TestMassTemp
- Added "Multi_HX711_Calibrate" to the test code
  - This gives you the values for Scale1Calibration, Scale2Calibration, and Scale3Calibration
- Fixed the Moment Balance Logic
- Fixed the RimD and TheadW so it actually updates with serial commands
- Removed XWeight,YWeight as they were bad duplicates of the centroid calculation
- Added percentinRangeWeight and inRangeWeight
  - Basically this says how much weight is on the same side as the CMAngle
  - This is important because the moment calculation for how much weight to be added needs to be based on the mass difference of each side and not of the total tire weight
  - This means that all past versions works for testing point masses but not practical tire masses
  - TLDR old code not work, new code "should" be more acurate for all scenarios
