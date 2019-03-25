project('blinky', 'c',
          default_options : ['b_lto=false',
                             'b_asneeded=false'])

# variables globales
srcs       =  []
inc_dir    =  []
c_args     =  []
link_args  =  []
link_deps  =  []
#==============================================================================#
# uController / HAL Driver dependend options
c_args     += '-DEFM32HG322F64' # HAL Driver define
linkfiles   = files(['external_dependencies/sapi/libs/Device/SiliconLabs/EFM32HG/Source/GCC/efm32hg.ld'])
startupfile = files(['external_dependencies/sapi/libs/Device/SiliconLabs/EFM32HG/Source/GCC/startup_efm32hg.c'])

# NOTE(elsuizo:2019-03-24): esto no se si va...
#==============================================================================#
# # # parse linkerfiles to a linker commando
foreach linkfile : linkfiles
 link_args += ['-Wl,-T,@0@/@1@'.format(meson.current_source_dir(), linkfile)]
endforeach
#==============================================================================#



#==============================================================================#
subdir('external_dependencies') # con esto creo que agregamos un subdirectorio que debe tener un meson.build

main = executable(
            'main',
            [srcs, sapi_srcs, 'program.c'] ,
            name_suffix         : 'elf',
            c_args              : [c_args ],
            link_args           : [link_args, '-Wl,--gc-sections','--specs=nano.specs', '--specs=rdimon.specs'],
            dependencies        : link_deps,
            include_directories : [inc_dir, sapi_include_dirs] )

#==============================================================================#
# import binary objects
objcopy  = '@0@'.format(find_program('objcopy').path())
objdump  = '@0@'.format(find_program('objdump').path())
size     = '@0@'.format(find_program('size').path())
gdb      = '@0@'.format(find_program('gdb').path())
terminal = '@0@'.format(find_program('terminal',required:false).path())
openocd  = '@0@'.format(find_program('openocd', required:false).path())

#==============================================================================#
# create custom target : create binary, hex dump, size and disassemble dump
mainbin = custom_target(
                        'main.bin',
    output           : ['main.bin'],
    # name_suffix      : 'bin',
    build_by_default : true,
    command          : [objcopy, '-O', 'binary', 'main.elf', 'main.bin'],
    depends          : [main])

mainhex = custom_target(
                            'main.hex',
        output           : ['main.hex'],
        build_by_default : true,
        command          : [objcopy, '-O', 'ihex', 'main.elf', 'main.hex'],
        depends          : [main])

mainsize = custom_target(
                          'size',
        capture          : true,
        output           : ['main.size'],
        build_by_default : true,
        command          : [size, '--format=berkeley', 'main.elf'],
        depends          : [main])

if meson.get_compiler('c').get_id() == 'clang'
    maindump = custom_target(
                          'dump',
        capture          : true,
        output           : 'main.dump',
        build_by_default : false,
        command          : [objdump, '-triple=@0@-none-eabi'.format(arch), '-disassemble-all', '-S', '-t', 'main.elf'],
        depends          : [main])

elif meson.get_compiler('c').get_id() == 'gcc'
  maindump = custom_target(
                        'dump',
      capture          : true,
      output           : 'main.dump',
      build_by_default : false,
      command          : [objdump, '-D', '-S', '-t', 'main.elf'],
      depends          : [main])
endif


#==============================================================================#
#support target commands
run_target('gdb',
         command : [terminal, '--working-directory=@0@'.format(meson.source_root()), '-e', gdb, '-q', '@0@/main.elf'.format(meson.current_build_dir())])

run_target('openocd',
         command : [terminal, '-e', openocd, '-f', 'interface/stlink-v2.cfg', '-f' , 'target/stm32f3x.cfg'])

#==============================================================================#
