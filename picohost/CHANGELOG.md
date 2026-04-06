# Changelog

## [0.1.0](https://github.com/EIGSEP/pico-firmware/compare/v0.0.3...v0.1.0) (2026-04-06)


### Features

* add APP_IMU2 dispatch for second IMU (antenna) pico ([8c26b32](https://github.com/EIGSEP/pico-firmware/commit/8c26b32676c5cddde932ae7b3570c4391e9ccad9))
* add potentiometer app, host, emulator, test ([7d7c531](https://github.com/EIGSEP/pico-firmware/commit/7d7c5318df4f70257eedbd0245f1075d453af706))
* replace I2C IMU with UART RVC protocol ([cea6305](https://github.com/EIGSEP/pico-firmware/commit/cea630569abe21a618586e06df26360095468372))


### Bug Fixes

* use in_waiting as property to match pyserial-mock 0.1.0 ([34e2f70](https://github.com/EIGSEP/pico-firmware/commit/34e2f706d9ecf7d7032466f3668d6cf64c1b373e))

## [0.0.3](https://github.com/EIGSEP/pico-firmware/compare/v0.0.2...v0.0.3) (2026-03-23)


### Bug Fixes

* add serial communication watchdog to tempctrl app ([#12](https://github.com/EIGSEP/pico-firmware/issues/12)) ([fbfdc5d](https://github.com/EIGSEP/pico-firmware/commit/fbfdc5dd6a7b7ece08e8e2ac9dd1e84b4dd37a1a))
* fix bugs and dead code in pico apss ([#15](https://github.com/EIGSEP/pico-firmware/issues/15)) ([2d6913a](https://github.com/EIGSEP/pico-firmware/commit/2d6913ad96a1181c1476a7759fa77b646af37240))
