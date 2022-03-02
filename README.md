# vario
paragliding vario with arduino nano, piezo sounder, bmp180 altimeter, 4x Nimh batteries  
  
old video:  
https://youtu.be/r1CVEgGCK1Q  
  
I have been having problems with this vario, but I have now moved to using the MS5611 module, which is more expensive, but I read that it is less affected by RF noise.  
I need to flight test this next version and then maybe reduce from 4 cells to 1 cell.  
  
new video:  
https://youtu.be/hXAc20TRS4Y  
  
latest news; I made one with BMP180 with ideas from here  
https://www.youtube.com/watch?v=dkzky-Mxypo  
putting it in a metal tin seems to cut out the RF interference.  
  
I also made one with BMP280. This works so well that I can leave the sensor to do the averaging and let the arduino sleep in between.  
The reponse time is quicker than the one with the BMP180  
  
![Schematic Picture](./varios.jpg)
![Schematic Picture](./"varios open.jpg")

