Please review the Howto.md from the Nano_Releases folder for Nano version compatibility

https://github.com/Mechanceism/Tire-Balancer/blob/main/Nano_Releases/HowTo.md


# How to upload
Not going to lie this CYD board is very picky

You must do the following:
- Download the zip of the version of the screen code you want to use (Newest version is best)
- Extract the zip file
- Open the folder and see the folders "ui" and "libraries"
  - Do not copy and paste this to your libraries folder
  - Included is the folder "V{Version Number}" which contains the squareline files incase you want to edit the UI
- Open the "ui" folder
- Open the "ui.ino" file
  - I am personally using 2.3.4 of the arduino ide in case this is important for bug fixing
  - Most likely your version is fine to be on but if not then try version 2.3.4
- Goto file, preferences, sketchbook location, and click browes and select the folder that contains the folders "ui" and "libraries" that you downloaded
  - I made this very simple so you didn't have to download libraries seperatly
- Make sure you have the board manager open to insure you have the esp32 boards installed
  - Here is the github link to insure you do or not: https://github.com/espressif/arduino-esp32
  - Owner: Espressif Systems
- Select the following board:
  - Esp32 Dev Module
  - Change flash mode to DIO
- Plug in the CYD over usb
  - Confirm the nano is not connected to the serial connection on the CYD
  - USE USB 2, DO NOT USE USB 3
    - Idk why but usb 3 breaks how the serial connect from the nano to esp works from programming
- Press upload
  - Wait till the compile and upload is complete
- Reconnect the nano to the serial lines on the CYD
- Connect Power and Test
