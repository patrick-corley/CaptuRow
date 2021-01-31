# CaptuRow
Hardware and software platform for capturing and analyzing rowing performance data.

<b>Project Goal</b>: 
- Develop open-source hardware, firmware and data analysis systems for the niche market of rowing performance monitors.
- Make useful, reliable performance monitoring systems that can be produced at a low cost and thus, made more accessible for a larger group of individuals in the rowing community.

Folder Structure:  

<b>Arduino</b>:
  - Firmware for data capture hardware (~~Atmega 328P~~ Atmega 2560 + MMA8452 Accelerometer + Neo 6M GPS module + micro-SD card module)

<b>Python:</b>
  - Scripts for post-processing captured rowing data (acceleration on all axes, distance, speed, etc.)
  
<b>Octave:</b>
  - More scripts for post-processing data (filtering, FFT, etc.) - May migrate to python in the future...
  
<b>Reliable Datasets:</b>
  - Contains good data from rowing sessions. Used as reference for development.
  
 <b>Third-party Libraries Used:</b>
 
    Arduino:
     - https://github.com/kosme/arduinoFFT
     - https://github.com/mikalhart/TinyGPSPlus
     - https://github.com/greiman/SdFat
     - https://github.com/PaulStoffregen/TimerOne
     - https://github.com/sparkfun/SparkFun_MMA8452Q_Arduino_Library
     - https://github.com/Seeed-Studio/Grove_LCD_RGB_Backlight
