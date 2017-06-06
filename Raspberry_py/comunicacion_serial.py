#! /usr/bin/env python

import serial
import time

arduino = serial.Serial('/dev/ttyUSB0', 115200)

time.sleep(2)

print("preparado...")
print('enviando comandos....')
arduino.write(b'SV0001400')
time.sleep(1)
arduino.write(b'SH1400000')
time.sleep(1)
arduino.write(b'SV0700700')
time.sleep(1)
arduino.write(b'SH1200000')
time.sleep(1)
print('mirando al frente...OK')
time.sleep(1)
arduino.write(b'FR0000000')
time.sleep(1)
arduino.write(b'VE1601600')
time.sleep(1)
arduino.write(b'VE0200200')
time.sleep(1)
arduino.write(b'VE0800800')
time.sleep(1)
arduino.write(b'VE1001000')
time.sleep(1)
arduino.write(b'FR0000000')
time.sleep(1) 
print('comandos enviados...OK')
arduino.close()      
    
