# CaptuRow
Hardware and software platform for capturing and analyzing rowing performance data.

Folder Structure:
Arduino:
  - Firmware for data capture hardware (Atmega 328P + MMA8452 Accelerometer + Neo 6M GPS module + micro-SD card module)

Python:
  - Scripts for post-processing captured rowing data (acceleration on all axes, distance, speed, etc.)
  
Octave:
  - More scripts for post-processing data (filtering, FFT, etc.) - May migrate to python in the future...
  
Reliable Datasets:
  - Contains good data from rowing sessions. Used as reference for development.
