Adafruit-Blinka-Raspberry-Pi5-rp1pio
====================================

|      CI              | status |
|----------------------|--------|
| pip builds           | [![Pip Actions Status][actions-pip-badge]][actions-pip-link] |
| [`cibuildwheel`][]   | [![Wheels Actions Status][actions-wheels-badge]][actions-wheels-link] |

[actions-badge]:           https://github.com/adafruit/Adafruit_Blinka_Raspberry_Pi5_rp1pio/workflows/Tests/badge.svg
[actions-pip-link]:        https://github.com/adafruit/Adafruit_Blinka_Raspberry_Pi5_rp1pio/actions?query=workflow%3A%22Pip
[actions-pip-badge]:       https://github.com/adafruit/Adafruit_Blinka_Raspberry_Pi5_rp1pio/workflows/Pip/badge.svg
[actions-wheels-link]:     https://github.com/adafruit/Adafruit_Blinka_Raspberry_Pi5_rp1pio/actions?query=workflow%3AWheels
[actions-wheels-badge]:    https://github.com/adafruit/Adafruit_Blinka_Raspberry_Pi5_rp1pio/workflows/Wheels/badge.svg

Installation
------------

Installing from source:

 - clone this repository
 - `pip install ./Adafruit_Blinka_Raspberry_Pi5_rp1pio`

Installing from pip (not yet available):

 - `pip install Adafruit-Blinka-Raspberry-Pi5-rp1pio`

Building the documentation
--------------------------

Documentation for the example project is generated using Sphinx. Sphinx has the
ability to automatically inspect the signatures and documentation strings in
the extension module to generate beautiful documentation in a variety formats.
The following command generates HTML-based reference documentation; for other
formats please refer to the Sphinx manual:

 - `cd Adafruit_Blinka_Raspberry_Pi5_rp1pio`
 - `pip install '.[docs]'`
 - `make -C docs html`

License
-------

Adafruit\_Blinka\_rp1pio is provided under the GPL-2-only license that can be found in the LICENSE
file. By using, distributing, or contributing to this project, you agree to the
terms and conditions of this license.

[`cibuildwheel`]:          https://cibuildwheel.readthedocs.io
