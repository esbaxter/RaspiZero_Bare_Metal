# RaspberryPi-Zero-BareMetal

This example of bare metal programming is for the Raspberry Pi Zero.  The ultimate goal is to develop a set of utilities that could be used in a drone system or for model rocketry or whatever you find interesting.  I started out by perusing David Welch's bare metal examples (https://github.com/dwelch67/raspberrypi-zero).  It currently has support for serial communications, I2C, SPI, interrupts and timers.  In addition, there is support for up to two Bosch BME 280s, a SensorTech MPU6050 and a PCA 9685 servo controller.

Note:  I know I need to rework the I2C support to handle both interrupt driven and "user level" interleaved transactions.