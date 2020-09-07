# CaptuRow
Hardware and software platform for capturing and analyzing rowing performance data.

Folder Structure:  

<b>Arduino</b>:
  - Firmware for data capture hardware (~~Atmega 328P~~ Atmega 2560 + MMA8452 Accelerometer + Neo 6M GPS module + micro-SD card module)

<b>Python:</b>
  - Scripts for post-processing captured rowing data (acceleration on all axes, distance, speed, etc.)
  
<b>Octave:</b>
  - More scripts for post-processing data (filtering, FFT, etc.) - May migrate to python in the future...
  
<b>Reliable Datasets:</b>
  - Contains good data from rowing sessions. Used as reference for development.
