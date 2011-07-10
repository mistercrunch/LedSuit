avrdude -v -c stk500 -p attiny48 -P /dev/ttyUSB0 -U hfuse:r:high.txt:i -U lfuse:r:low.txt:i
