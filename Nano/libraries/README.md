# Libraries for the Nano

The exact versions the Nano firmware was built and tested with, copied here
unchanged so the repo builds without installing anything.

| Folder | Library | Version | Licence |
|---|---|---|---|
| `MPU6050` | MPU6050 by Electronic Cats (includes `I2Cdev`) | 1.4.3 | MIT (see its `LICENSE`) |
| `HX711` | HX711 by Rob Tillaart | 0.6.0 | MIT (see its `LICENSE`) |

`Wire` and `EEPROM` come with the Arduino AVR board package.

To use them, set the Arduino IDE's **Sketchbook location** to the `Nano`
folder (the one holding `TireBalancer_Nano` and `libraries`). Full steps are in
[docs/Setup.md](../../docs/Setup.md#nano).

Do not update these from the Library Manager: a newer version may behave
differently. To change a version on purpose, replace the folder and re-test
the leveling and the scale.
