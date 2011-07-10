avrdude -p attiny48 -P /dev/ttyUSB0 -c stk500 -b 115200 -Uflash:w:LedSuit.elf.hex:i LedSuit.elf -F
