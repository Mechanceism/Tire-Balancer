# Wiring

## Arduino Nano

| Nano pin | Goes to | Notes |
|---|---|---|
| D0 (RX) | Screen TX | Shared with the Nano's USB. |
| D1 (TX) | Screen RX | Shared with the Nano's USB. 5 V signal into a 3.3 V board - see below. |
| D2 / D3 | HX711 **A** DOUT / SCK | Load cell at 210° |
| D7 / D8 | HX711 **B** DOUT / SCK | Load cell at 90° |
| D11 / D12 | HX711 **C** DOUT / SCK | Load cell at 330° |
| D4 | Laser servo signal | Pulse made by Timer2. Do not use `Servo.h`. |
| D9 / D10 | Actuator 1 UP / DOWN (PWM) | Actuator at 150° |
| D5 / D6 | Actuator 2 UP / DOWN (PWM) | Actuator at 30° |
| A2 / A3 | Laser 1 / Laser 2 | Laser 1 covers 0-180°, laser 2 covers 180-360° |
| A4 / A5 | I2C SDA / SCL | Platform MPU6050 at 0x68, frame MPU6050 at 0x69 (AD0 high) |
| GND | Screen GND | The two boards must share ground. |

Free pins: D13, A0, A1 (A6 and A7 are analog input only).

**H-bridge safety.** UP and DOWN of the same actuator must never be on
together. Only `actuatorDrive()` in `Level.ino` writes those pins; it switches
both off and waits 150 ms (`DEAD_TIME_MS`) before any start or reversal.

## Platform layout (top view)

```
                 90°  load cell B
                  |
   actuator 1     |     actuator 2
      150°  \     |     /  30°
             \    |    /
   180° ------ centre (laser servo) ------ 0°   (+X)
             /    |    \
      210°  /     |     \  330°
  load cell A     |     load cell C
                  |
                270°  fixed pivot, screen faces the user here
```

Angles are counter-clockwise from the right, like a normal graph.

## Screen (JC2432W328C) through a logic level converter

Use a bidirectional level converter (the common BSS138 MOSFET type) between
the boards. The Nano's 5 V TX straight into the ESP32's 3.3 V RX pin is over
its rated voltage.

| Screen (low side) | Converter | Nano (high side) |
|---|---|---|
| 3.3 V | LV | |
| | HV | 5 V |
| GND | GND | GND |
| TX (GPIO 1) | LV1 ⇄ HV1 | D0 (RX) |
| RX (GPIO 3) | LV2 ⇄ HV2 | D1 (TX) |

- **LV must be the ESP32's 3.3 V.** The VCC pin on the CYD's 4-pin serial plug
  is most likely 5 V: measure it before using it for anything. If the plug has
  no 3.3 V, take it from another CYD connector that does.
- Connect only **one** 5 V source to each board. If the screen is powered from
  the machine through that VCC pin, don't also tie it to the Nano's 5 V.
- Ground must be shared.

### Leaving it connected permanently

That is fine, with these conditions:

- **Uploading to the Nano (USB):** works while the screen is quiet. The screen
  normally sends `Ping` every 2 s, which would corrupt an upload. It now detects
  the Nano's bootloader traffic and stops sending for 10 s (the corner shows
  `Paused: noise on RX`). The first sync attempt can still collide, but avrdude
  retries. If an upload still fails, switch the screen's power off for the upload.
- **Uploading to the screen (USB):** works while the Nano is quiet. The Nano
  only transmits when spoken to (debug streams are off by default) and silently
  ignores the screen's upload traffic. If you have turned a debug stream on,
  send `Stop`, `LevelDebugOff` and `ScaleDebugOff` first, or power the Nano off.
- **The computer:** the converter does not connect either board's power to the
  other's USB, and with one board unpowered it does not back-feed the other.
  The usual rules still apply: don't feed two 5 V sources into one board, and
  return the actuator current to its supply on its own heavy wires, not through
  the logic ground or a USB cable.

## If the screen cannot hear the Nano

The bottom-left corner of the screen shows the link state:

| Corner text | Meaning |
|---|---|
| `Nano OK` (green) | The screen is hearing valid lines from the Nano. |
| `No link rx:0 bad:0` | Not a single byte is arriving on the screen's RX pin. |
| `No link rx:<counting> bad:<counting>` | Bytes arrive but are garbled. |

What the firmware already does about it: the Nano no longer uses `String`
(V0.4 had 467 bytes of RAM left and printed text could go missing), never
blocks, and no longer floods the line with error text. Important messages carry
a checksum and are repeated until acknowledged.

If it still shows `No link` with the Nano running:

1. Open Serial Monitor on the Nano (wires to the screen disconnected), type
   `Ping`. `Pong` proves the Nano side is fine.
2. Reconnect. If `rx` stays at 0 or `bad` climbs, the signal is not arriving
   cleanly, and the likely reason is electrical: the screen's RX pin (GPIO 3) is
   also driven by the screen's own USB-serial chip, so the Nano's TX has to
   overpower it. That fight depends on supply voltage, which would explain why
   it worked from one USB port and not others, and why a resistor divider
   (which weakens the Nano's signal) did not help.
3. The test for that: move only the **screen's RX wire** (on the converter's
   low side) to a GPIO the USB chip is not connected to. GPIO 35 is input-only
   and a good choice if your board brings it out (check the silkscreen). Then
   change one line in `Screen/ui/ui.ino`:

   ```cpp
   #define NANO_RX_PIN  35
   ```

   Nothing changes on the Nano, and the screen's TX wire stays where it is.

This is a hypothesis until tested on the machine - the on-screen counters are
there to confirm or rule it out.
