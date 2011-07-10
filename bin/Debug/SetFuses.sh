avrdude -c stk500 -p attiny48 -P /dev/ttyUSB0 -U lfuse:w:0xE2:m hfuse:w:0xDF:m efuse:w:0xFF:m
