# ECE4180_FinalProject
![image](https://github.gatech.edu/storage/user/36924/files/f3635698-ef15-46a8-bdad-d2ec784cee50)

Project Description [here](https://github.gatech.edu/pages/nkoh8/ECE4180_FinalProject/)

# Getting Started

### Hardware Setup 
**After gathering all the parts listed in the appendix section**,
1. Create the schematic detailed in the Power Management section to setup the power management system for Herome.
2. Create the schematic detailed in the Audio section to setup the bluetooth, speaker, and emic chip connection to mbed
3. Create the schematic detailed in the Motor Control section to setup the 4 wheel chassis driven by the Pi and the hardware acceleration delegation
4. If desired, 3D print your own decorative pieces and assemble

### Software Setup
**The application currently uses the MobileDet V2 but any quantized model compiled using the edgetpu compiler and trained for the MS COCO dataset labels will suffice
Setup a version of the Debian Bullseye compatible with the user's selected Raspberry Pi**

1. Enable the CSI interface on the Raspberry Pi
2. Clone this repo to the desired directory on the Pi
```
git clone https://github.gatech.edu/nkoh8/ECE4180_FinalProject.git
```

3. Follow these steps [here](https://coral.ai/docs/edgetpu/tflite-python/) at the following link to setup the TfLite and the edgetpu runtime library

4. Navigate to the /VideoProcessing directory and run
```
pip3 install -r requirements.txt
```

5. After all the dependencies are installed, run any of the following to have Herome start following you. The difference is in the execution throughput but there shouldn't be any visual noticable difference in the tracking trajectory.
```
python3 SingleThread.py
python3 MultiThread.py
python3 MultiProc.py
```

