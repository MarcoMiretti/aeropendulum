<img src="https://user-images.githubusercontent.com/26353057/69723422-3f68fb80-10f8-11ea-8771-58c19bcd802d.png" alt="drawing" align="left" width="80" height="80"/>

# Aeropendulum [![Build Status](https://travis-ci.org/MarcoMiretti/aeropendulum.svg?branch=master)](https://travis-ci.org/MarcoMiretti/aeropendulum)

Final project of the subject control systems. It is composed of a drone propeller driven pendulum. An angle is measured through a rotary encoder, (taken out of a printer).

The unstable system can be controlled using numerous control techniques. For the subject PID was used.


## :hammer: Compilation:
```
cd firmware
make
```
#### Prequisites:
* arm-none-eabi-*
* GNU Make

## :arrow_down_small: Target Download:
```
cd firmware
make download
```

#### Prequisites:
* arm-none-eabi-*
* GNU Make
* OpenOCD

## :bug: Debug:
```
cd firmware
make server_open
```

open new terminal

```
cd firmware
make server_connect 
```

#### Prequisites:
* arm-none-eabi-*
* GNU Make
* OpenOCD

## :books: Generate documentation:
```
cd firmware
make documentation
```

#### Prequisites:
* GNU Make
* doxygen
* pdflatex
