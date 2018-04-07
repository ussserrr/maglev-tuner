# maglev-tuner
Tuner plugin for PID controlling based on the Texas Instruments TM4C1294NCPDT MCU (EK-TM4C1294XL board). Name has been defined by the first application of the device - magnetic levitation.

## Description
Program implementing PID controller with the feature of tuning PID coefficients (Kp, Ki, Kd) by analog inputs (potentiometers). There is no any remote control protocol such as in [maglev-ti-rtos](https://github.com/ussserrr/maglev-ti-rtos) or [maglev](https://github.com/ussserrr/maglev) projects, but device is able to send current PID coefficients over UART. So main purpose is to properly tune controller (i.e. determine correct coefficients) using some algorithm (for example [Ziegler–Nichols method](https://en.wikipedia.org/wiki/Ziegler–Nichols_method)) and then set them as default ones in the developing program.

## Usage
4 analog inputs of the MCU are used: PE0 for PID feedback and PE3-PE1 for Kp, Ki and Kd respectively (connect 3 potentiometers to them). Use on-board button (PJ0) to send current coefficients over USB UART at 115200 speed (8N1 configuration). PID PWM output is PG0 pin (`PWM_FREQUENCY`=8kHz frequency).

## Requirements
  - Texas Instruments TivaWare library
