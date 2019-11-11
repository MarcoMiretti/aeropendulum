# Aeropendulum

Repositorio para el proyecto final de la materia sistemas de control

## Arbol de proyecto
Se espera que el proyecto siga la siguente estructura:
```
+-- README.md
+-- .gitignore
|
+-- firmware
|	+-- Makefile
|	+-- libs (not yet commited)
|	|	+-- (not yet commited)
|	+-- scripts (not yet commited)
|	|	+-- (not yet commited)
|	+-- aero
|	|	+-- inc
|	|	|	+-- main.h
|	|	|	+-- FreeRTOSConfig.h
|	|	|	+-- lpc4337_HAL.h
|	|       |	+-- (tasks and/or files headers)
|	|	+-- src
|	|       |	+-- main.c (aquí la descripción del proyecto)
|	|	|	+-- FreeRTOSConfig.c
|	|	|	+-- lpc4337_HAL.c
|	|       |	+-- (tasks and/or files headers)
+-- mechanical
|	+-- 3DPrints
|	|	+-- freecad
|	|	|	+-- *.fcstd
|	|	+-- stl
|	|	|	+-- *.stl
| 	+-- blueprints
| 	| 	+-- *.*
```
## Lineamientos de código

* Se trabaja con tabs equivalentes a 8 espacios.
* Variables y parámetros tienen tipos de datos completos.
* Expresiones para constantes #define están encerradas en paréntesis.
Si es posible, se busca utilizar:
* Nombres en _MAYÚSCULAS_ para identificar registros o instrucciones del procesador.
* Nombres en _camelCase_ para identificar funciónes o rutinas de interrupciones.
* Prefijos Namespace_ para agrupar funciónes relacionadas (ej. cuando creamos una librería para periféricos).

## Compilación
### Linux:
```
cd firmware
TODO: UPDATE
```
#### Prequisitos
* arm-none-eabi-gcc
* st-util
* GNU Make
* OpenOCD

## Flash
### Linux

```
cd firmware
TODO: UPDATE
```

#### Prequisitos
* arm-none-eabi-gcc
* st-util
* GNU Make
* OpenOCD
