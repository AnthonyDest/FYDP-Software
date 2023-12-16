# FYDP-Software

## SnoMo

Code is split into 4 modules:  
image_processing.py   - used for machine vision and localization  
path_planning.py      - used to determine path to take  
robot_control.py      - used to control robot hardware  
main.py               - used to merge all modules together  

## Setup
Install Python: https://code.visualstudio.com/docs/python/python-tutorial

Install requirements: pip install -r requirements.txt

### SSH into Raspberry Pi

1. Ensure Raspberry Pi is on and connected to same wifi network
1. Test connection: ```ping raspberrypi.local```

1. Following https://code.visualstudio.com/docs/remote/ssh
    1. user@hostname: ```fydp@raspberrypi.local```  
    1. Platform: Linux

