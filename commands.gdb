# run 'arm-none-eabi-gdb target/thumbv7em-none-eabihf/debug/rc-car --command=commands.gdb'
# after 'cargo run'

# Connect to gdb remote server
target remote :3333

# Enable demangling asm names on disassembly
set print asm-demangle on

# Enable pretty printing
set print pretty on

# Disable style sources as the default colors can be hard to read
set style sources off

break main

# Set a breakpoint at DefaultHandler
break DefaultHandler

# Set a breakpoint at HardFault
break HardFault

monitor reset halt

continue
