# bmi088 

A rust embedded-hal driver for the 
[Bosch Sensortec BMI088](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf)
6DOF integrated accelerometer and gyroscope (IMU).

The BMI088 combines the functionality of two inertial sensors
into one device: a triaxial 16-bit gyroscope 
and a triaxial 16-bit accelerometer.


## Status

**I am not actively maintaining this crate. If you would like to take on ownership of the crate, please open a PR and provide a note to that effect.**

- [x] Basic SPI support
- [x] Support for probe (check product identifier)
- [x] Split into separate gyro and accel interfaces (separate eg CSN lines)
- [x] blocking read of gyro data
- [x] blocking read of accel data
- [ ] configure FIFO
- [ ] Support for data ready (DRDY) pins
- [ ] Support for interrupt pins
- [ ] Support for DMA with SPI
- [ ] Basic I2C support
- [ ] Tests with mock embedded hal
- [ ] Usage example with `cortex-m` hal
- [ ] Doc comments
- [ ] CI
- [ ] Support for user recalibration




## Possible Example
Likely to change:

```
    let mut bmi088_a = bmi088::Builder::new_accel_spi(spi_bus1.acquire(), spi1_cs_bmi088_accel);
    bmi088_a.setup(&mut delay_source).unwrap();

    let mut bmi088_g = bmi088::Builder::new_gyro_spi(spi_bus1.acquire(), spi1_cs_bmi088_gyro);
    if bmi088_g.setup(&mut delay_source);

    if let Ok(gyro_sample) = bmi088_g.get_gyro() {
        hprintln!("bmi088_g: {:?}", gyro_sample));
    }

    if let Ok(accel_sample) = bmi088_a.get_accel() {
        hprintln!("bmi088_a: {:?}", accel_sample));
    }
```

