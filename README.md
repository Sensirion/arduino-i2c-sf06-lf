# Sensirion I²C SF06-LF Arduino Library

This is the Sensirion SF06-LF library for Arduino allowing you to 
communicate with a sensor of the SF06-LF family over I²C.

<img src="images/sensor_SLF3C_1300F.png" width="300px">

Click [here](https://sensirion.com/products/product-categories/liquid-flow/) to learn more about the Sensirion SF06-LF sensor family.


Not all sensors of this driver family support all measurements.
In case a measurement is not supported by all sensors, the products that
support it are listed in the API description.



## Supported sensor types

| Sensor name   | I²C Addresses  |
| ------------- | -------------- |
|[SLF3C-1300F](https://sensirion.com/products/catalog/SLF3C-1300F/)| **0x08**|
|[SLF3S-1300F](https://sensirion.com/products/catalog/SLF3S-1300F/)| **0x08**|
|[SLF3S-0600F](https://sensirion.com/products/catalog/SLF3S-0600F/)| **0x08**|
|[SLF3S-4000B](https://sensirion.com/products/catalog/SLF3S-4000B/)| **0x08**|
|[LD20-0600L](https://sensirion.com/products/catalog/LD20-0600L/)| **0x08**|
|[LD20-2600B](https://sensirion.com/products/catalog/LD20-2600B/)| **0x08**|

For instructions how to change the I2C address of your SLF3x sensor please refer to `examples/exampleI2cAddressChange/README.md`.

The following instructions and examples use a *SLF3C-1300F*.

## Installation of the library

This library can be installed using the Arduino Library manager:
Start the [Arduino IDE](http://www.arduino.cc/en/main/software) and open
the Library Manager via

`Sketch` ➔ `Include Library` ➔ `Manage Libraries...`

Search for the `Sensirion I2C SF06-LF` library in the `Filter
your search...` field and install it by clicking the `install` button.

If you cannot find it in the library manager, download the latest release as .zip file 
and add it to your [Arduino IDE](http://www.arduino.cc/en/main/software) via

`Sketch` ➔ `Include Library` ➔ `Add .ZIP Library...`

Don't forget to **install the dependencies** listed below the same way via library 
manager or `Add .ZIP Library`

#### Dependencies
* [Sensirion Core](https://github.com/Sensirion/arduino-core)

## Sensor wiring

Use the following pin description to connect your SF06-LF to the standard I²C bus of your Arduino board:

<img src="images/SLF3x_Pinout.png" width="300px">

| *Pin* | *Cable Color* | *Name* | *Description*  | *Comments* |
|-------|---------------|:------:|----------------|------------|
| 1 |  | NC | Do not connect | 
| 2 | green | SDA | I2C: Serial data input / output | 
| 3 | red | VDD | Supply Voltage | 3.2V to 3.8V
| 4 | black | GND | Ground | 
| 5 | yellow | SCL | I2C: Serial clock input | 
| 6 |  | NC | Do not connect | 


The recommended voltage is 3.3V.

### Board specific wiring
You will find pinout schematics for recommended board models below:



<details><summary>Arduino Uno</summary>
<p>

| *SF06-LF* | *SF06-LF Pin* | *Cable Color* | *Board Pin* |
| :---: | --- | --- | --- |
| SDA | 2 | green | D18/SDA |
| VDD | 3 | red | 3.3V |
| GND | 4 | black | GND |
| SCL | 5 | yellow | D19/SCL |



<img src="images/Arduino-Uno-Rev3-i2c-pinout-3.3V.png" width="600px">
</p>
</details>



<details><summary>Arduino Nano</summary>
<p>

| *SF06-LF* | *SF06-LF Pin* | *Cable Color* | *Board Pin* |
| :---: | --- | --- | --- |
| SDA | 2 | green | A4 |
| VDD | 3 | red | 3.3V |
| GND | 4 | black | GND |
| SCL | 5 | yellow | A5 |



<img src="images/Arduino-Nano-i2c-pinout-3.3V.png" width="600px">
</p>
</details>



<details><summary>Arduino Micro</summary>
<p>

| *SF06-LF* | *SF06-LF Pin* | *Cable Color* | *Board Pin* |
| :---: | --- | --- | --- |
| SDA | 2 | green | D2/SDA |
| VDD | 3 | red | 3.3V |
| GND | 4 | black | GND |
| SCL | 5 | yellow | ~D3/SCL |



<img src="images/Arduino-Micro-i2c-pinout-3.3V.png" width="600px">
</p>
</details>



<details><summary>Arduino Mega 2560</summary>
<p>

| *SF06-LF* | *SF06-LF Pin* | *Cable Color* | *Board Pin* |
| :---: | --- | --- | --- |
| SDA | 2 | green | D20/SDA |
| VDD | 3 | red | 3.3V |
| GND | 4 | black | GND |
| SCL | 5 | yellow | D21/SCL |



<img src="images/Arduino-Mega-2560-Rev3-i2c-pinout-3.3V.png" width="600px">
</p>
</details>



<details><summary>ESP32 DevKitC</summary>
<p>

| *SF06-LF* | *SF06-LF Pin* | *Cable Color* | *Board Pin* |
| :---: | --- | --- | --- |
| SDA | 2 | green | GPIO 21 |
| VDD | 3 | red | 3V3 |
| GND | 4 | black | GND |
| SCL | 5 | yellow | GPIO 22 |



<img src="images/esp32-devkitc-i2c-pinout-3.3V.png" width="600px">
</p>
</details>


## Quick Start

1. Install the libraries and dependencies according to [Installation of the library](#installation-of-the-library)

2. Connect the SF06-LF sensor to your Arduino as explained in [Sensor wiring](#sensor-wiring)

3. Open the `exampleUsage` sample project within the Arduino IDE:

   `File` ➔ `Examples` ➔ `Sensirion I2C SF06-LF` ➔ `exampleUsage`

  
   The provided example is working with a SLF3C-1300F, I²C address 0x08.
   In order to use the code with another product or I²C address you need to change it in the code of `exampleUsage`. 
   You find the list with pre-defined addresses in `src/SensirionI2CSf06Lf.h`.


5. Click the `Upload` button in the Arduino IDE or `Sketch` ➔ `Upload`

4. When the upload process has finished, open the `Serial Monitor` or `Serial
   Plotter` via the `Tools` menu to observe the measurement values. Note that
   the `Baud Rate` in the used tool has to be set to `115200 baud`.

## Contributing

**Contributions are welcome!**

We develop and test this driver using our company internal tools (version
control, continuous integration, code review etc.) and automatically
synchronize the master branch with GitHub. But this doesn't mean that we don't
respond to issues or don't accept pull requests on GitHub. In fact, you're very
welcome to open issues or create pull requests :)

This Sensirion library uses
[`clang-format`](https://releases.llvm.org/download.html) to standardize the
formatting of all our `.cpp` and `.h` files. Make sure your contributions are
formatted accordingly:

The `-i` flag will apply the format changes to the files listed.

```bash
clang-format -i src/*.cpp src/*.h
```

Note that differences from this formatting will result in a failed build until
they are fixed.


## License

See [LICENSE](LICENSE).
