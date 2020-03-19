# bmi088 

A rust embedded-hal driver for the 
[Bosch Sensortec BMI088](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf)
6DOF integrated accelerometer and gyroscope (IMU).

The BMI088 combines the functionality of two inertial sensors
into one device: a triaxial 16-bit gyroscope 
and a triaxial 16-bit accelerometer.


## Status

- [x] Basic SPI support
- [x] Support for probe (check product identifier)
- [x] split into separate gyro and accel interfaces (separate CSN lines)
- [ ] blocking read of gyro data
- [ ] blocking read of accel data
- [ ] Support for DRDY pins
- [ ] Support for DMA with SPI
- [ ] Basic I2C support
- [ ] Tests with mock embedded hal
- [ ] Usage example with `cortex-m` hal
- [ ] Doc comments
- [ ] CI
- [ ] Support for user recalibration






