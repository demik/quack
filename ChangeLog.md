## v1.4.8
- added support for MacAlly Joystick
- added support for Wiimotes
	- D-pad movement working, click on B, half-click on A
	- selectable movement speed on the Weemote + LED feedback
	- this lay the framework to add drivers for some unsupported devices

## v1.4.7
- new feature: half click on right BT click (similar to some ADB devices)
- workarounds for v1.4.6 green led regression

## v1.4.6
- added support for ADB composite devices (Kensignton and some Joysticks)
- fixed ADB start and stop bit times (65 µs vs 70 µs, error in AN591)
- update to ESP-IDF v5.2.2
	- https://github.com/espressif/esp-idf/releases/tag/v5.2.2
	- fixed flash corruption after pairing too much devices (20+)
	- fixed ADB host mode where the ESP boot was too fast (v5.2 regression)

## v1.4.5
- update to ESP-IDF v5.1.2
	- https://github.com/espressif/esp-idf/releases/tag/v5.1.2
	- big background update, lot of cleaning and rewrites
	- as most patches are now mainstream, sdk patching is now minimal
	- removed direct ABI bluedroid call

## v1.4.4
- update to ESP-IDF v4.2.3:
	- https://github.com/espressif/esp-idf/releases/tag/v4.2.3
	- mostly bluetooth related security and bug fixes

## v1.4.3
- provides some slight acceleration and desceleration on bluetooth mouses

## v1.4.2
- added support for MicroSpeed MacTRAC 2.0
- fixed a BT led state when a device reconnect while scanning

## v1.4.1
- update to ESP-IDF v4.2.2:
	- https://github.com/espressif/esp-idf/releases/tag/v4.2.2
	- fixed controller not reporting disconnect event
- fixed BT LED state when using multiple mouses

## v1.4.0
- Initial firmare release
- 1.4.x series is compatible with 1.3 and 1.4 PCBs (EDA subfolder)
