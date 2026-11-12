# Firmware read back from the boards (2026-11-12)

Read from the machine before V0.6.0 was flashed. These are compiled programs,
not source code, but they can be flashed back to get the old behaviour.

| File | What it is |
|---|---|
| `Nano_V0.5_readback.hex` | The Nano's flash: the "lost" V0.5 (has `outputExp`, `TestMasson`, `REoff`, `ManualLaseron`, `inRangeWeight`). |
| `Nano_V0.5_readback_eeprom.hex` | The Nano's EEPROM at the same time. |
| `Screen_V0.2_readback_4MB.bin` | Full 4 MB flash of the screen (CYD_SD2_V0.2). Not committed (large, and the V0.2 source is in git history); kept locally only. |

## Putting V0.5 back on the Nano

The Nano has the old bootloader (57600 baud). From the Arduino15 folder:

```
avrdude -C avrdude.conf -p atmega328p -c arduino -P COM6 -b 57600 -U flash:w:Nano_V0.5_readback.hex:i
```

(`avrdude.exe` and `avrdude.conf` are under
`%LOCALAPPDATA%\Arduino15\packages\arduino\tools\avrdude\<version>\`.)

## Putting the old screen back

```
esptool --port COM7 --baud 460800 write-flash 0 Screen_V0.2_readback_4MB.bin
```
