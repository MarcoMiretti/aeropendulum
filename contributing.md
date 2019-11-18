# Lineamientos de contribuciónes

## Arbol de proyecto
Se espera que el proyecto siga la siguente estructura:
```
+-- readme.md
+-- contributing.md
+-- .gitignore
|
+-- firmware
|	+-- Makefile
|	+-- libs 
|	|	+-- cmsis
|	|	|	+-- inc
|	|	|	|	+-- (cmsis headers)
|	|	|	+-- src
|	|	|	|	+-- (cmsis sources)
|	|	+-- hal
|	|	|	+-- inc
|	|	|	|	+-- (hal headers)
|	|	|	+-- src
|	|	|	|	+-- (hal sources)
|	+-- middlewares
|	|	+-- FreeRTOS
|	|	|	+-- Source
|	|	|	|	+-- (FreeRTOS code)
|	+-- scripts
|	|	+-- openocd
|	|	|	+-- stm32f4discovery.cfg
|	|	|	+-- gdbinit
|	+-- aero
|	|	+-- inc
|	|	|	+-- main.h
|	|	|	+-- FreeRTOSConfig.h
|	|       |	+-- (tasks and/or files headers)
|	|	+-- src
|	|       |	+-- main.c
|	|	|	+-- FreeRTOSConfig.c
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
