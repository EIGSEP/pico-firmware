# Changelog

## [4.4.0](https://github.com/EIGSEP/pico-firmware/compare/v4.3.0...v4.4.0) (2026-07-09)


### Features

* **picohost:** el-only calibrate-imu with derived (most-down) home; retire az blend ([#152](https://github.com/EIGSEP/pico-firmware/issues/152)) ([f86a226](https://github.com/EIGSEP/pico-firmware/commit/f86a22697c92bde4f34ae4ed14c5092142488cbe))

## [4.3.0](https://github.com/EIGSEP/pico-firmware/compare/v4.2.0...v4.3.0) (2026-07-09)


### Features

* **tempctrl:** report every plausible sample; rate guard is control-only ([a983473](https://github.com/EIGSEP/pico-firmware/commit/a983473eddc22062dd6fe44aa9d72d2557715488))


### Bug Fixes

* **picohost:** publish whole-valued firmware floats as floats ([#148](https://github.com/EIGSEP/pico-firmware/issues/148)) ([74fc79e](https://github.com/EIGSEP/pico-firmware/commit/74fc79eb4b589323c2a24c404e10fbc24b31d158))
* **tempctrl:** drop rate anchor on sensor-trip latch so reporting recovers ([0b41a1d](https://github.com/EIGSEP/pico-firmware/commit/0b41a1d80231e0f0fb3fc48a131f1fe3c25f7d83))

## [4.2.0](https://github.com/EIGSEP/pico-firmware/compare/v4.1.0...v4.2.0) (2026-07-08)


### Features

* **tempctrl:** per-channel installed flag descopes absent hardware ([2d3e707](https://github.com/EIGSEP/pico-firmware/commit/2d3e7071ab92d3541a2ef6e46acf32a1a2d99dec))
* **tempctrl:** swap thermistor curve to Vishay NTCLE100E3 10k NTC ([ba30131](https://github.com/EIGSEP/pico-firmware/commit/ba30131f7420aa2467840f0a5db3fe6ff0bc1762))


### Bug Fixes

* **tempctrl:** sync emulator stall constants to firmware ([2979311](https://github.com/EIGSEP/pico-firmware/commit/29793112e070ace1d7a00c7e0be5e1a5fbef0104))

## [4.1.0](https://github.com/EIGSEP/pico-firmware/compare/v4.0.0...v4.1.0) (2026-07-03)


### Features

* **tempctrl:** split data-validity status from control latches ([7b0bb3a](https://github.com/EIGSEP/pico-firmware/commit/7b0bb3a2d58932401c059f2e00687ebdb66677be))

## [4.0.0](https://github.com/EIGSEP/pico-firmware/compare/v3.11.0...v4.0.0) (2026-07-03)


### ⚠ BREAKING CHANGES

* **rfswitch:** sw_state is now an EEPROM path address (0-15) for the new RF switch PCB, not an 8-bit GPIO bitmask; this firmware cannot drive the old direct-GPIO switches, and raw integer commands from old callers are incompatible (named paths via PicoRFSwitch.switch() carry over unchanged).

### Features

* **calibrate-pot:** default to azimuth mode ([6ba5603](https://github.com/EIGSEP/pico-firmware/commit/6ba5603add264b1f3931954ca5b4bd124da229b2))
* **calibrate-pot:** motor-driven auto mode ([#141](https://github.com/EIGSEP/pico-firmware/issues/141)) ([cfede3a](https://github.com/EIGSEP/pico-firmware/commit/cfede3a5355fb034e22705866d4d6ba941c90762))
* **rfswitch:** drive EEPROM path addresses instead of raw GPIO bitmask ([db973ec](https://github.com/EIGSEP/pico-firmware/commit/db973ec00460c36eef4af62421f42a235bfb84e3))
* **rfswitch:** fan PCB thermistors into rfswitch_therm stream with host-side degC ([e4119c0](https://github.com/EIGSEP/pico-firmware/commit/e4119c001a3a956b6dfe5cfadbd1353cfa13d7a2))


### Bug Fixes

* **rfswitch:** report None on ADC-saturated thermistor readings ([a934b76](https://github.com/EIGSEP/pico-firmware/commit/a934b76ad7dde58d6f640a113d6651e34c5dee02))

## [3.11.0](https://github.com/EIGSEP/pico-firmware/compare/v3.10.0...v3.11.0) (2026-07-01)


### Features

* **calibrate-imu:** gate faulted IMU at startup with operator prompt, guard fit ([65add47](https://github.com/EIGSEP/pico-firmware/commit/65add472b3d8d228282ff8c0f482a26964a4ed39))
* **calibrate-imu:** three-state stream_status classifier (healthy/faulted/dead) ([49d7308](https://github.com/EIGSEP/pico-firmware/commit/49d73081979d3247b911a426f36aa91d2f3ebf44))
* **current:** record system_current cal in metadata (symmetric) + manual recovery ([#137](https://github.com/EIGSEP/pico-firmware/issues/137)) ([d09866d](https://github.com/EIGSEP/pico-firmware/commit/d09866d1a0503e0d862015a3c9885b0dc3471281))
* **motor:** smooth constant-acceleration step ramp ([8b71de2](https://github.com/EIGSEP/pico-firmware/commit/8b71de2d039edc67cf59b0dfdf953b18f952f111))


### Bug Fixes

* **calibrate-imu:** clean abort on mid-sweep fault; harden faulted-prompt test ([b91ac34](https://github.com/EIGSEP/pico-firmware/commit/b91ac341e00b13231a878168f15f7120d36636b1))
* **calibrate-imu:** drop status=error frames in collect_vector, abort on sustained fault ([f73fca6](https://github.com/EIGSEP/pico-firmware/commit/f73fca67f3b9c62e3ab633947d2d6bfb3cce03cb))
* **imu_geometry:** reject zero-norm/degenerate accel with clear ValueError ([f0f5bba](https://github.com/EIGSEP/pico-firmware/commit/f0f5bbacea3d578d10f92dcda55cc479fb993d84))
* **imu:** pole-symmetric sin² azimuth blend with deadband ([#133](https://github.com/EIGSEP/pico-firmware/issues/133)) ([f6d798b](https://github.com/EIGSEP/pico-firmware/commit/f6d798bd33f40819e037b449aa3bd6a267394422))

## [3.10.0](https://github.com/EIGSEP/pico-firmware/compare/v3.9.0...v3.10.0) (2026-06-29)


### Features

* **calibrate-pot:** manual mode + slope sanity check ([190a2c9](https://github.com/EIGSEP/pico-firmware/commit/190a2c9a75eedabd5bcd4ea9e15281ed4aaecde0))
* IMU az/el calibration + live conversion ([#129](https://github.com/EIGSEP/pico-firmware/issues/129)) ([5f08d79](https://github.com/EIGSEP/pico-firmware/commit/5f08d79436e6fd0b487598e6b14b1419f8a8e9b5))

## [3.9.0](https://github.com/EIGSEP/pico-firmware/compare/v3.8.0...v3.9.0) (2026-06-28)


### Features

* **calibrate-pot:** add headroom computation to pot electrical ends ([6437756](https://github.com/EIGSEP/pico-firmware/commit/6437756b02c7b2d25379b335483b0a37d22ea557))
* **calibrate-pot:** add in-box azimuth sweep collection ([cb82291](https://github.com/EIGSEP/pico-firmware/commit/cb82291800ebd9f2db80bb4a1564a5bd50941ede))
* **calibrate-pot:** add predicted_angle_divergence helper ([73fcfbb](https://github.com/EIGSEP/pico-firmware/commit/73fcfbb2ce446141596519e1c39f434f48f7ce05))
* **calibrate-pot:** add prompt_save confirm/discard helper ([82b38af](https://github.com/EIGSEP/pico-firmware/commit/82b38afec781165d1ef83ec47bf3f353d6297c1e))
* **calibrate-pot:** add read-only stream:motor az reader ([3022060](https://github.com/EIGSEP/pico-firmware/commit/3022060e699c499ea903aa3d0651221ca0ae84ec))
* **calibrate-pot:** add rezero mode reusing stored slope ([c78f6f2](https://github.com/EIGSEP/pico-firmware/commit/c78f6f24661ff445aa7894b893737d884117e297))
* **calibrate-pot:** add slope fit with zero pinned to motor-home ([2f7687a](https://github.com/EIGSEP/pico-firmware/commit/2f7687af53b7e5b39ac53c10538de0828b9950aa))
* **calibrate-pot:** default --turns to the installed 3.75-turn pot ([7ba94ec](https://github.com/EIGSEP/pico-firmware/commit/7ba94ec88fbedc2fcd1097bd1f66b4ced663768c))
* **calibrate-pot:** report azimuth fit linearity; fix span guard and headroom labels ([9d0a9bd](https://github.com/EIGSEP/pico-firmware/commit/9d0a9bdf0a6451dd5bb235da7184ca5d1309ffca))
* **calibrate-pot:** wire azimuth and rezero modes into the CLI ([b1529f3](https://github.com/EIGSEP/pico-firmware/commit/b1529f3b4940a1fda549206e8b84b55205785357))
* **currentmon:** multi-point least-squares fit with quality metric ([f8973b6](https://github.com/EIGSEP/pico-firmware/commit/f8973b6e53c09b9d5bf009b9843ea240b3eb41a6))
* **currentmon:** two-point calibrate-current tool ([5982542](https://github.com/EIGSEP/pico-firmware/commit/5982542553d5a74d0e7eb9496940ccb5d1805f1b))
* **flash-picos:** confirm flashed boards via manager-owned pico_config ([e1e61cf](https://github.com/EIGSEP/pico-firmware/commit/e1e61cf9ceed11f334c3adf864ccde3d907f0c1e))
* **lidar:** emulate current_voltage for emulator/firmware parity ([9e5162b](https://github.com/EIGSEP/pico-firmware/commit/9e5162b2586c21318d20954bec346f16df81e678))
* **lidar:** fan current out to metadata['system_current'] ([424d848](https://github.com/EIGSEP/pico-firmware/commit/424d848715cb007906b8e9440c23f5f83d89de75))
* **manager:** continuous self-discovery of unbound CDC ports ([78b3ee0](https://github.com/EIGSEP/pico-firmware/commit/78b3ee093215f9f34936d9f5a7fbc4368020e0e5))


### Bug Fixes

* **currentmon:** guard preset-override input; cast residual; cover multi happy path ([25f60a7](https://github.com/EIGSEP/pico-firmware/commit/25f60a7292484b07bc908462878f168e142234b3))
* **currentmon:** harden co-located current monitor against shared-status coupling ([16d0f6e](https://github.com/EIGSEP/pico-firmware/commit/16d0f6e4adeaf344d3bd60cbadaa04f31d8a2feb))
* **currentmon:** use DMM-measured divider resistor values (3.32k/4.64k) ([e428e84](https://github.com/EIGSEP/pico-firmware/commit/e428e84f0c4829d27fbc0c346e9bfcd39a84d47f))
* **flash-picos:** require heartbeat liveness for manager confirmation ([e847ba9](https://github.com/EIGSEP/pico-firmware/commit/e847ba918b655e8342ea866977f396ed67ce7095))
* **flash-picos:** split GPIO readback into fast discovery + quiet-bus sweep ([f6d5008](https://github.com/EIGSEP/pico-firmware/commit/f6d5008a0cf8dcff4d90881b57dd50a55ed5eaf3))
* **flash-picos:** stop the GPIO readback from dropping flashed boards ([6654e6e](https://github.com/EIGSEP/pico-firmware/commit/6654e6ecb2fc14f818ff5b3547a32898c87e1181))
* **manager:** drop removed --uf2 flag from picomanager.service ([b4af160](https://github.com/EIGSEP/pico-firmware/commit/b4af1600e407d106c530c76f21c4ee88e9115c30))
* **manager:** snapshot device list under lock in _discover_new ([2372ff5](https://github.com/EIGSEP/pico-firmware/commit/2372ff5fd43203c610fbdfe81e0f5a5884908ae5))
* use the pico gpio26 for currentmon ([77f38f0](https://github.com/EIGSEP/pico-firmware/commit/77f38f0de895958892316415b10129c94d75a3f9))


### Documentation

* update flash-picos docs for manager-owned discovery ([07cd444](https://github.com/EIGSEP/pico-firmware/commit/07cd444695c216b4d3db4d7bb5f6d360f7b4db8a))

## [3.8.0](https://github.com/EIGSEP/pico-firmware/compare/v3.7.0...v3.8.0) (2026-06-20)


### Features

* **potmon:** drop el channel, read az only ([400e002](https://github.com/EIGSEP/pico-firmware/commit/400e002d8cda44e3a036f46b2e0ba853f11f90cd))


### Bug Fixes

* **flash_picos:** run serial readback in a child process so a wedged port can't pin process exit ([d122ceb](https://github.com/EIGSEP/pico-firmware/commit/d122cebc6931af1203bc664c56b476bad65a1a04))
* raise runtime error instead of json error ([5ab32cc](https://github.com/EIGSEP/pico-firmware/commit/5ab32ccb5a760feae4e53ca380ceccd1609840c8))

## [3.7.0](https://github.com/EIGSEP/pico-firmware/compare/v3.6.0...v3.7.0) (2026-06-19)


### Features

* **flash_picos:** auto-stop picomanager around the flash window ([2b95c4f](https://github.com/EIGSEP/pico-firmware/commit/2b95c4f185b4587c0ae0d2d3b7d4885d22c0e82d))
* **flash_picos:** default to GPIO mass-BOOTSEL flash flow with --no-gpio fallback ([032d11b](https://github.com/EIGSEP/pico-firmware/commit/032d11b7d62cd248b0965355694aa2d06fc3cb5a))
* **motor:** report random per-boot boot_id in status ([39a8fc2](https://github.com/EIGSEP/pico-firmware/commit/39a8fc2de92c768cc29caaa3935e9ce0d1b845b2))
* **picohost:** add gpio module for mass BOOTSEL entry and reset ([866b6e3](https://github.com/EIGSEP/pico-firmware/commit/866b6e3e182feb89bea4d00f6c327b87bc470350))
* **picohost:** add manager_service systemctl wrapper for picomanager ([b7af7b6](https://github.com/EIGSEP/pico-firmware/commit/b7af7b6bb29b605cac8607bdec27dfe861f66e2f))
* **picohost:** add pico-gpio CLI (bootsel/reset subcommands) ([426ec84](https://github.com/EIGSEP/pico-firmware/commit/426ec84b3c531f1a0173480dea6ac4f449fccc5e))
* **picohost:** declare gpiozero runtime dependency ([#101](https://github.com/EIGSEP/pico-firmware/issues/101)) ([cdfe2df](https://github.com/EIGSEP/pico-firmware/commit/cdfe2df4a5bc10731ffd62053b48baba55bab42b))
* **picohost:** persist motor position and re-seed after pico reboot ([85a6c60](https://github.com/EIGSEP/pico-firmware/commit/85a6c60210d5a8c35528ca8478a17464c7f3687f))
* **picohost:** rename systemd unit to picomanager.service ([74241c6](https://github.com/EIGSEP/pico-firmware/commit/74241c6c585900cc444a304c1d0bb6a4cb01d2f1))
* **tempctrl:** add runaway trip + sensor rate-sanity guards ([ae8095b](https://github.com/EIGSEP/pico-firmware/commit/ae8095bee6bfff82365733f83d5d36f7a4b54f92))
* **tempctrl:** add scratchpad CRC guard and two-to-anchor sensor seed ([af25994](https://github.com/EIGSEP/pico-firmware/commit/af2599403b8a40e2c217f6d6b82544d514e39c76))
* **tempctrl:** lower default drive clamp from 0.6 to 0.2 ([#107](https://github.com/EIGSEP/pico-firmware/issues/107)) ([57728b3](https://github.com/EIGSEP/pico-firmware/commit/57728b3c36b8b6650af035405324acad36b3d3e2))
* **tempctrl:** read thermistors over ADC ([#109](https://github.com/EIGSEP/pico-firmware/issues/109)) ([949a40c](https://github.com/EIGSEP/pico-firmware/commit/949a40c534b2e62a51a8f0537f82b71e7120da42))


### Bug Fixes

* **flash_picos:** bound serial readback so a wedged CDC port can't hang the fleet flash ([66c5f8f](https://github.com/EIGSEP/pico-firmware/commit/66c5f8fe52dc56c9aa40552c7e2ac8aa670ef296))
* **flash_picos:** read back device-info in flash-picos ([3d32557](https://github.com/EIGSEP/pico-firmware/commit/3d32557c0778eaf31d511294b264bd8c81133220))
* **flash_picos:** reboot into BOOTSEL, then load ([d138141](https://github.com/EIGSEP/pico-firmware/commit/d1381415e023aba5b02dc60c0a4c7b302340dfec))
* **flash_test:** prefer --bus/--address over --ser in auto-discovery ([d00aa7e](https://github.com/EIGSEP/pico-firmware/commit/d00aa7ea2704361577355c2c520ba4df6e68f254))
* **flash_test:** recognize RP2350 BOOTSEL PID 000f (was RP2040 0003) ([0199563](https://github.com/EIGSEP/pico-firmware/commit/0199563426100d70d1d352a45710034fc6a95c59))
* **picohost:** make pyserial-mock an optional lazy import ([#112](https://github.com/EIGSEP/pico-firmware/issues/112)) ([c98ab96](https://github.com/EIGSEP/pico-firmware/commit/c98ab963864e56e5dd8cb2f7e8f6afdf405497d0))
* **picohost:** retry flash step and settle between Picos for BOOTSEL re-enumeration races ([1d9bff0](https://github.com/EIGSEP/pico-firmware/commit/1d9bff090fda51c03a122176ae538f8c379f20bc))
* **tempctrl:** make sensor-sanity latch sticky until host ack ([f783cef](https://github.com/EIGSEP/pico-firmware/commit/f783cef933096fb3b7b1c2c8e10e21bd3d97e516))

## [3.6.0](https://github.com/EIGSEP/pico-firmware/compare/v3.5.0...v3.6.0) (2026-05-22)


### Miscellaneous Chores

* release 3.6.0 ([cf1809b](https://github.com/EIGSEP/pico-firmware/commit/cf1809ba72b1a467bf94630da9c2d287fec9dc7e))

## [3.5.0](https://github.com/EIGSEP/pico-firmware/compare/v3.4.0...v3.5.0) (2026-05-21)


### Features

* **tempctrl:** add cooling-mode guard via asymmetric clamp ([fc4c13f](https://github.com/EIGSEP/pico-firmware/commit/fc4c13ff75e3d95a5de4897d5da29dc40af5b33d))


### Bug Fixes

* **tempctrl:** gate stall guard on drive!=0, not just active ([9960ece](https://github.com/EIGSEP/pico-firmware/commit/9960ece24fb7ddc65a1315ef60adfdd0b5d8b1ae))

## [3.4.0](https://github.com/EIGSEP/pico-firmware/compare/v3.3.0...v3.4.0) (2026-05-21)


### Features

* **flash_test:** auto-discover all BOOTSEL Picos by default ([#85](https://github.com/EIGSEP/pico-firmware/issues/85)) ([41d1df8](https://github.com/EIGSEP/pico-firmware/commit/41d1df817a3a755439af6e22c5c0b6114cf4fc74))
* **picohost:** identify Picos by USB serial in flash CLIs ([#81](https://github.com/EIGSEP/pico-firmware/issues/81)) ([2dae27d](https://github.com/EIGSEP/pico-firmware/commit/2dae27da900f1a6556dfc390091e337cccd7bdc8))
* **picohost:** run calibrate_pot through PicoManager, support fractional turns ([2347450](https://github.com/EIGSEP/pico-firmware/commit/2347450afccc6cd95622397daf57ed79fb3f9b87))
* **tempctrl:** per-channel stall guard for stuck sensors ([d953ba3](https://github.com/EIGSEP/pico-firmware/commit/d953ba34ab823155e2b85228c52ee26444338905))
* **tempctrl:** replace bang-bang with PI controller ([b2e6299](https://github.com/EIGSEP/pico-firmware/commit/b2e62991c1eb897cbd6a05a6dd401fb49c6d2cc7))


### Bug Fixes

* **flash_picos:** resolve port from usb_serial after re-enumeration ([#80](https://github.com/EIGSEP/pico-firmware/issues/80)) ([8391445](https://github.com/EIGSEP/pico-firmware/commit/83914457b770a4eb5be300e70a2bdf502bb7bd81))
* **imu-emulator:** clear sensor data on (re-)init to match firmware ([cec4137](https://github.com/EIGSEP/pico-firmware/commit/cec4137bd52e8f3c3fcee0a5e0ecad54cd98a074))
* **lidar/potmon-emulator:** align defaults with firmware contract ([e2d2211](https://github.com/EIGSEP/pico-firmware/commit/e2d22112e1b378cccf243c360383363e153d7ef2))
* **picohost:** settle udev before opening post-flash serial port ([#83](https://github.com/EIGSEP/pico-firmware/issues/83)) ([2e0b729](https://github.com/EIGSEP/pico-firmware/commit/2e0b72955f2dbdac5f9c069ad63418ff814f1c36))
* **picohost:** tolerate non-UTF-8 bytes in picotool output ([b9acd25](https://github.com/EIGSEP/pico-firmware/commit/b9acd25afa26596347a6d625f3787a332e3d45b7))
* **tempctrl-emulator:** close remaining divergences from tempctrl.c ([534661b](https://github.com/EIGSEP/pico-firmware/commit/534661b718d6cfcd1f8f7158dbfd94ac48f56184))
* **tempctrl-emulator:** match firmware cJSON parse semantics ([d0469e0](https://github.com/EIGSEP/pico-firmware/commit/d0469e0b0d76a926c042fc3fdf0c177cb27b056a))
* **tempctrl:** clear active flag when channel is disabled ([f48cf18](https://github.com/EIGSEP/pico-firmware/commit/f48cf18f660703b4d896fff8f442299c52f98056))
* **tempctrl:** reset PI integrator on Ki change; freeze it when Ki=0 ([5527bd8](https://github.com/EIGSEP/pico-firmware/commit/5527bd8f868e01249c46694b19eb907bbefa5521))

## [3.3.0](https://github.com/EIGSEP/pico-firmware/compare/v3.2.1...v3.3.0) (2026-05-16)


### Features

* **tempctrl:** split status into per-channel Redis streams ([6c6a29d](https://github.com/EIGSEP/pico-firmware/commit/6c6a29d50a2f72f3bebe2604c004fe9a362575db))

## [3.2.1](https://github.com/EIGSEP/pico-firmware/compare/v3.2.0...v3.2.1) (2026-05-14)


### Bug Fixes

* emit status="error" per-cycle on firmware data-refresh failure ([3acdeff](https://github.com/EIGSEP/pico-firmware/commit/3acdeffe60616aa6a0aad13e19ae736082043350))

## [3.2.0](https://github.com/EIGSEP/pico-firmware/compare/v3.1.1...v3.2.0) (2026-05-13)


### Features

* add flash-test CLI and BOOTSEL test firmware ([#71](https://github.com/EIGSEP/pico-firmware/issues/71)) ([fad169f](https://github.com/EIGSEP/pico-firmware/commit/fad169f6271488ca8592a255600f73d7027f0f55))


### Bug Fixes

* **picohost:** drop --config flag from pico-manager.service ExecStart ([cfe4a98](https://github.com/EIGSEP/pico-firmware/commit/cfe4a98cd1b6af54dfc0110255990abb751cc978))

## [3.1.1](https://github.com/EIGSEP/pico-firmware/compare/v3.1.0...v3.1.1) (2026-05-05)


### Bug Fixes

* **flash-picos:** use PICO_CONFIG_KEY constant in confirmation message ([c414935](https://github.com/EIGSEP/pico-firmware/commit/c41493595cea85943160c05e711d3eb7afa2f900))

## [3.1.0](https://github.com/EIGSEP/pico-firmware/compare/v3.0.0...v3.1.0) (2026-04-30)


### Features

* **peltier:** replay last-applied config on reconnect ([fbb77cc](https://github.com/EIGSEP/pico-firmware/commit/fbb77cc01641db30cf54afeabb8d6bd6cbfd853f))
* **rfswitch:** report UNKNOWN sentinel while switch is settling ([15c4a0f](https://github.com/EIGSEP/pico-firmware/commit/15c4a0f5076fd4867e542b995f0cc03998f16d51))


### Bug Fixes

* make keepalive idempotent, add serial write lock, only set last properties after succesfull command send ([6507163](https://github.com/EIGSEP/pico-firmware/commit/65071634793c7277951b81a45c6ce653978d1374))
* **motor:** publish position fields as float, not int ([#63](https://github.com/EIGSEP/pico-firmware/issues/63)) ([31ffc73](https://github.com/EIGSEP/pico-firmware/commit/31ffc7316c0ff653fd5d53041a4e5c8cdeb977e3))
* **picohost:** fire on_reconnect from reader-thread self-heal ([6ac65fb](https://github.com/EIGSEP/pico-firmware/commit/6ac65fb3ed046922695fea0b47359203a46ff135))


### Documentation

* add imu sim analysis notebook ([64f2efa](https://github.com/EIGSEP/pico-firmware/commit/64f2efa42046d0e8192fe104d72372829536c334))

## [3.0.0](https://github.com/EIGSEP/pico-firmware/compare/v2.2.1...v3.0.0) (2026-04-21)


### ⚠ BREAKING CHANGES

* **picohost:** Redis schema rework. ``pico_config`` / ``pico_health`` / ``picos`` keys are replaced by the new config-store blob and per-device ``heartbeat:pico:{name}`` keys. ``flash-picos`` now publishes to Redis by default (``--output-file`` is an opt-in for offline provisioning). ``PicoDevice`` / ``PicoMotor`` / ``PicoPeltier`` / ``PicoPotentiometer`` constructors take ``metadata_writer=`` instead of ``eig_redis=``. ``PicoProxy`` takes a ``Transport`` instead of a raw redis client.
* **picohost:** picohost now requires `eigsep_redis` in addition to (or instead of) `eigsep_observing`.

### Features

* **picohost:** depend on eigsep_redis directly, not eigsep_observing ([bc6a913](https://github.com/EIGSEP/pico-firmware/commit/bc6a913d27126e765c9317c8555673c8a8165622))
* **picohost:** make Redis the canonical pot calibration store ([d07674d](https://github.com/EIGSEP/pico-firmware/commit/d07674d405d870d65c422e476493db66d5d897e4))
* **picohost:** publish human-readable RfSwitch state to Redis ([c40dc2b](https://github.com/EIGSEP/pico-firmware/commit/c40dc2b626d03b43f370f3c127fa9afea2487c4f))
* **picohost:** rebuild Redis surface on eigsep_redis writer/reader classes ([8e4b8ff](https://github.com/EIGSEP/pico-firmware/commit/8e4b8ffec6dfaf93af178a37dc4534a1a378f013))


### Performance Improvements

* **picohost:** interrupt manager shutdown instead of waiting for blocking I/O ([54f19ca](https://github.com/EIGSEP/pico-firmware/commit/54f19ca64fc776c5648509ef51c67d6771fe81ba))

## [2.2.1](https://github.com/EIGSEP/pico-firmware/compare/v2.2.0...v2.2.1) (2026-04-17)


### Bug Fixes

* initialize redis hander before starting Pico reader thread([#48](https://github.com/EIGSEP/pico-firmware/issues/48)) ([40cbb5a](https://github.com/EIGSEP/pico-firmware/commit/40cbb5a5556a1995124c6264a2ab7991abf70d21))

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
