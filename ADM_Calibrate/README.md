## Stat Cache -- A black box datalogger for On Screen Display ##

To calibrate your hardware upload the ADM_Calibrate.ino sketch to your datalogger with an sd card installed. If you get a blinking red led there is a problem with the sd card, try reformatting it FAT 16 or try a different card. After the led lights begin to flash shades of yellow, rotate the device 360 degrees several times in at least 3 axis. When you are finished you can shut the device off and a CLB00.csv file will be on the card with your accel, gyro, and magnetometer maximums and minimums. You can open it in excel or any text app to copy the values into the ADM_Stat_Cache.ino sketch starting on line 166. 

This is a rough calibration process, it will not provide a soft iron magnetic calibration, but the gyro and accel should be fairly accurate using this method. The magnetometer still needs work.
