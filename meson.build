project('blinky', 'c',
          default_options : ['c_std=c99', 'cpp_std=c++11'])

# variables globales
srcs       =  []
inc_dir    =  []
c_args     =  []
link_args  =  []
link_deps  =  []
#==============================================================================#
# uController / HAL Driver dependend options
c_args     += '-DEFM32HG322F64=1' # HAL Driver define
linkfiles   = files(['external_dependencies/sapi/libs/Device/SiliconLabs/EFM32HG/Source/GCC/efm32hg.ld'])
startupfile = files(['external_dependencies/sapi/libs/Device/SiliconLabs/EFM32HG/Source/GCC/startup_efm32hg.c'])

#==============================================================================#
# convenience function : link correct C-runtime for each controller
# TODO: use llvm compiler-rt
arch       = (host_machine.cpu() == 'cortex-m0') or (host_machine.cpu() == 'cortex-m0+') or (host_machine.cpu() == 'cortex-m1') ? 'armv6-m'  : ''
arch      += (host_machine.cpu() == 'cortex-m3') ?                                                                                'armv7-m'  : ''
arch      += (host_machine.cpu() == 'cortex-m4') or (host_machine.cpu() == 'cortex-m7') ?                                         'armv7e-m' : ''

link_deps +=  meson.get_compiler('c').find_library('/usr/lib/gcc/arm-none-eabi/8.3.0/@0@/libgcc'.format(arch))
# we have to define the std headers : for stdint.h
if meson.get_compiler('c').get_id() == 'gcc'
  inc_dir += include_directories('/usr/arm-none-eabi/include') # bare-metal : std header includes
endif

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
            [srcs, sapi_srcs, 'program.c', startupfile] ,
            name_suffix         : 'elf',
            c_args              : [c_args, '-std=c99', '-O2'],
            link_args           : [link_args, '-Wl,--gc-sections'],
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
