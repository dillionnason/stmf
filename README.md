# STM Framework
A library to support development/prototyping on the STM32L476 Discovery board in C. This library was developed on Linux with GCC and has not been tested on any other platforms.

**NOTE: This library is primarily for personal projects and is not actively developed.**

## Building
This will generate a static library under the include directory (stmf/include/stmf.a)
```sh
git clone https://github.com/dillionnason/stmf
cd stmf
make
```

## Currently Targetted Interfaces
- [  ] NVIC
- [  ] GPIO 

## Important Sources
* [RM0351 Reference Manual: STM32L4x5 and STM32L4x6 advanced Arm-based 32-bit MCUs](https://github.com/dillionnason/stmf/blob/master/docs/RM0351%20Reference%20Manual.pdf)
* [UM1879 User Manual: Discovery kit with STM32L476VG MCU](https://github.com/dillionnason/stmf/blob/master/docs/UM1879%20User%20Manual.pdf)
* _Embedded Systems with Arm Cortex-M Microcontrollers in Assembly Language and C (Third Edition)_
  * Dr. Yifeng Zhu
  * ISBN: 978-0-9826926-6-0
