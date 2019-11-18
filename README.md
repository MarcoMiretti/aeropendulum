# Aeropendulum

Repositorio para el proyecto final de la materia sistemas de control

## :hammer: Compilación:
### :penguin: Linux:
```
cd firmware
make
```
#### Prequisitos:
* arm-none-eabi-*
* GNU Make

## :arrow_down_small: Target Download:
### :penguin: Linux:

```
cd firmware
make download
```

#### Prequisitos:
* arm-none-eabi-*
* GNU Make
* OpenOCD

## :bug: Debug:
### :penguin: Linux:

```
cd firmware
make server_open
```

open new terminal

```
cd firmware
make server_connect 
```

#### Prequisitos:
* arm-none-eabi-*
* GNU Make
* OpenOCD

## :books: Generate documentation:
### :penguin: Linux:
```
cd firmware
make documentation
```

#### Prequisitos:
* GNU Make
* doxygen
* pdflatex
