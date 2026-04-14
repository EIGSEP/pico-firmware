# Changelog

## [2.2.0](https://github.com/EIGSEP/pico-firmware/compare/v2.1.0...v2.2.0) (2026-04-14)


### ⚠ BREAKING CHANGES

* **picohost:** `picohost.RFSwitchProxy` removed. Replace `RFSwitchProxy("rfswitch", r).switch(state)` with `PicoProxy("rfswitch", r).send_command("switch", state=state)`.

### Code Refactoring

* **picohost:** drop RFSwitchProxy, use generic PicoProxy only ([9e13d6e](https://github.com/EIGSEP/pico-firmware/commit/9e13d6e1b70f6aad0f351465f823959294bd3ed3))

## [2.1.0](https://github.com/EIGSEP/pico-firmware/compare/v2.0.0...v2.1.0) (2026-04-13)


### Features

* **picohost:** add PicoProxy for Redis-mediated device control ([7a873da](https://github.com/EIGSEP/pico-firmware/commit/7a873da519eeca7c84e936a3a0aac7dcb89f7e27))


### Bug Fixes

* **picohost:** fix proxy response race, timeout drift, and manager request_id echo ([d380d7e](https://github.com/EIGSEP/pico-firmware/commit/d380d7eaa33ca2e0c062c4d55880877f49d13e8f))

## [2.0.0](https://github.com/EIGSEP/pico-firmware/compare/v1.0.0...v2.0.0) (2026-04-13)


### ⚠ BREAKING CHANGES

* **picohost:** PicoDevice.start()/stop() removed from public API. PicoMotor.stop() renamed to PicoMotor.halt().

### Features

* **picohost:** add cascading config discovery, flash integration, and Redis config store ([2f5c873](https://github.com/EIGSEP/pico-firmware/commit/2f5c873d0de483115644629d7a9229e4414f9b29))
* **picohost:** add PicoLidar class, raise on duplicate device name ([0d4eafd](https://github.com/EIGSEP/pico-firmware/commit/0d4eafddd2e1cc7f7954c552f9e38eeb2c173d9f))
* **picohost:** add PicoManager service for redis-based pico orchestration ([4aceac9](https://github.com/EIGSEP/pico-firmware/commit/4aceac94fefc5f26cd55aaa36b32a4f7242fdcba))
* **picohost:** add reconnect hook and last_status_time tracking ([a7c2801](https://github.com/EIGSEP/pico-firmware/commit/a7c28015b421894fe335fe53c1fedd50590a4f03))
* **picohost:** scan() homes before and after, rework motor_manual as zeroing script ([fbcca2a](https://github.com/EIGSEP/pico-firmware/commit/fbcca2a72fe3218cd18c9136f044766bcc848cb6))


### Bug Fixes

* **picohost:** fix PicoMotor halt no-op params, broken reset_deg_position, and missing scan homing ([e6c115d](https://github.com/EIGSEP/pico-firmware/commit/e6c115d4cf1382f5f15522f59d190d7100a9ec9b))
* **picohost:** make connect() idempotent and restart keepalive on reconnect ([5f898d8](https://github.com/EIGSEP/pico-firmware/commit/5f898d8c00a2127003914d59e0f6101fb96ccfd8))
* **picohost:** prevent Redis failures from killing background threads ([1ffe4d4](https://github.com/EIGSEP/pico-firmware/commit/1ffe4d419b4bbde92c35be9a435cd0d85331f4dc))


### Code Refactoring

* **picohost:** simplify PicoDevice lifecycle API ([cb03742](https://github.com/EIGSEP/pico-firmware/commit/cb037425f9337a8c2cf85096305490bd797dbe90))

## [1.0.0](https://github.com/EIGSEP/pico-firmware/compare/v0.1.0...v1.0.0) (2026-04-07)


### ⚠ BREAKING CHANGES

* consumers of the `potmon` Redis stream that read `pot_el_cal` / `pot_az_cal` (as a 2-element list) must switch to `pot_el_cal_slope` + `pot_el_cal_intercept` and the `pot_az_*` counterparts. The eigsep_observing `SENSOR_SCHEMAS` entry for `potmon` must be updated in lockstep when this lands.

### Code Refactoring

* flatten potentiometer cal field to scalar slope/intercept ([8866db6](https://github.com/EIGSEP/pico-firmware/commit/8866db6f4ca06bc405d222f2840297c3ce574fbc))

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
