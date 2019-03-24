# ciaa-z3r0-blinky
Blinky con la ciaa-z3r0 compilando el proyecto con Meson

## Dependencias:

 - Meson
   - En arch-linux: `sudo pacman -S meson`
   - En Window$: ho cares???

## Compilacion:

En el root del archivo (osea `ciaa-z3r0-blinky`) hacemos:

`meson gccbuild --cross-file=cross_file_gcc.build`

Donde: `gccbuild` puede ser cualquier nombre que le queramos asignar a la carpeta de salida
