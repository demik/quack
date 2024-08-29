# Quack X esp_8_bit
This is a crossover between two projects, that allow you to mod a Quack board to use with esp_8_bit firmware.
The result is a two players small box that can contains a few NES games and fit near your TV.

![quack+esp_8_bit](https://github.com/demik/quack/blob/master/Misc/esp_8_bit/quack%2Besp_8_bit.jpeg)

this is unsupported (outside of the quack project) and the parent project (esp_8_bit) is likely dead so this will likely not receive any updates or fixes

- parent project: [rossumur/esp_8_bit](https://github.com/rossumur/esp_8_bit)
- used patches from: [CornN64/esp_8_bit](https://github.com/CornN64/esp_8_bit)

it supports two players (one hardwired, either NES or SNES) and one Wiimote based. Unlike other builds, the hardwired connector runs at +5V (uses both level converters of the quack board), which allow the use of retro wireless adapters

schematic:

```
    -----------
    |         |
    |      25 |----------------------------------> video out
    |         |
    |         |   ------------
    |       2 |---| LVC 4245 |----/\/\/\/----|---> audio out
    |         |   ------------      1kΩ      |
    |         |                             ---
    |         |                             --- 10nF
    |  ESP32  |                              |
    |         |                              v gnd
    |         |
    |         |   ------------
    |       4 |---| LVC 2T45 |---< NES (or SNES) controller DATA
    |         |   ------------
    |         |   ------------
    |      12 |---| LVC 4245 |---> NES (or SNES) controller CLOCK
    |      14 |---| LVC 4245 |---> NES (or SNES) controller LATCH
    |         |   ------------
    |         |            5v <--> NES (or SNES) controller VCC
    |         |           gnd <--> NES (or SNES) controller GND
    |         |
    -----------


NES        ___
    DATA  |o o| NC
    LATCH |o o| NC
    CLOCK |o o/ 5V
    GND   |o_/

SNES       _
    5V    |o|
    CLOCK |o|
    LATCH |o|
    DATA  |o|
          |-|
    NC    |o|
    NC    |o|
    GND   |o|
           -

```

## Software

Since the esp_8_bit source code is old, you will need an old IDE (Arduino 1.8.14 with esp board 1.0.4)

there is a couple of patches in this directory that you will need to patch on top of [rossumur/esp_8_bit](https://github.com/rossumur/esp_8_bit)

- base.diff integrates patches from [CornN64/esp_8_bit](https://github.com/CornN64/esp_8_bit) and enable quack specific features (pinout, LEDs, DIP switches)
- controller.diff removes IR support, clean code and adds (S)NES controller support

```
> git clone https://github.com/rossumur/esp_8_bit.git
Cloning into 'esp_8_bit'...
remote: Enumerating objects: 345, done.
remote: Counting objects: 100% (15/15), done.
remote: Compressing objects: 100% (10/10), done.
remote: Total 345 (delta 5), reused 5 (delta 5), pack-reused 330 (from 1)
Receiving objects: 100% (345/345), 8.01 MiB | 8.04 MiB/s, done.
Resolving deltas: 100% (108/108), done.
> cd esp_8_bit
> patch -p1 < ../base.diff
patching file README.md
patching file esp_8_bit.ino
patching file 'src/emu.cpp'
patching file 'src/emu.h'
patching file 'src/emu_nofrendo.cpp'
patching file 'src/gui.cpp'
patching file 'src/hid_server/hid_server.cpp'
patching file 'src/video_out.h'
> patch -p1 < ../controller.diff
patching file 'src/ir_input.h'
patching file 'src/nintendo.h'
```

To avoid [this issue](https://github.com/rossumur/esp_8_bit/issues/19) you may want to change the partition layout as well. Check the solution into that issue, and also the boards.txt and quack_8_bit.csv if needed

###

## Hardware

This mod is compatible with board revision 1.3 and 1.4

### PCB mods

hardware modifications needed

- remove Q1
- remove FB9
- remove D1
- bridge R1 or replace by 0Ω
- remove the Mini DIN-4 connector

Under the board:

- connect a wire from DE-9 pin 6 to D1 pin (+) on top of the board
- connect a wire from DE-9 pin 8 to Mini DIN-4 pin 1

![bodge](https://github.com/demik/quack/blob/master/Misc/esp_8_bit/bodge.png)

### Adapter cable

You will need a NES or SNES extension cable, one triple RCA video cable, one USB cable (charge only is OK) and one DE-9 solderable connector
RC filtering circuit will be integrated into the connector. pinout is a follow (board side)

```
   _____________________
  /                     \
  \  1   2   3   4   5  /
   \   6   7   8   9   /
    \_________________/
```

| Pin | Connection | Notes |
|---|---|---|
| 1 | Ground | power ground & RCA shields here |
| 2 | +5V | USB +5V and (S)NES +5V connector go here |
| 3 | Ground | (S)NES ground + 10nF capacitor ground |
| 4 | NC | unused |
| 5 | Clock | (S)NES controller clock |
| 6 | Video | composite video out, yellow RCA |
| 7 | Sound | one pin of the 1kΩ resistor |
| 8 | Data | (S)NES controller data |
| 9 | Latch | (S)NES controller data |
| R | Resistor | solder capacitor on the other resistor pin & white + red RCA (+) wires |

isolate everything with hot glue or better

## Usage

Nothing different than the parent project as far as emulation goes

### Switches

Quack set of dip switches allows for the following settings

- ADB Host: (S)NES controller is player A, Wiimote player B or the other way around
- Bluetooth Disable: Kill BT Operation if not needed. More stable / less noise. Use with ADB Host to ON
- Flash: flash mode (GPIO0 to GND)

### LED

PCB LEDs are usable and do have basic functionality

- green: init done, board powered up
- yellow: valid (S)NES controller input received
- red: embedded flash operation in progress