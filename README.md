# SnoMo Capstone

Welcome to SnoMo! SnoMo is an innovative autonomous ice-resurfacing robot developed as the centerpiece of our 2024 Capstone Project.

## About SnoMo
**What is SnoMo?** SnoMo is an autonomous ice-resurfacing robot designed to tackle the challenges traditional ice-resurfacers face. It offers an affordable, effective, and easy solution for maintaining high-quality ice rinks, both indoors and outdoors.

For more information, please check out the [SnoMo Project Website](https://24capstone.wixsite.com/my-site-1)

# Technical Details

## SnoMo Software

### Code is split into 4 modules:  
**image_processing.py**   - used for machine vision and localization  
**path_planning.py**     - used to determine path to take  
**robot_control.py**      - used to control robot hardware  
**main.py**               - used to merge all modules together  

## Initial Setup

### Installations:

**Step 1:** Install Visual Studio Code

Download and install Visual Studio Code from the [official website](https://code.visualstudio.com/download).

**Step 2:** Install Python

If you don't have Python installed, you can download and install it from the [official Python website](https://www.python.org/downloads/).

**Step 3:** Install Miniconda

If you don't have Miniconda installed, you can download and install it from the [official website](https://docs.conda.io/en/latest/miniconda.html).

### Clone the Repository

```bash
git clone https://github.com/AnthonyDest/FYDP-Software.git
```

### Create and Activate Venv
```
python -m venv fydp_venv
.\fydp_venv\Scripts\activate
pip install -r requirements.txt
```

## Pi Setup

### SSH into Raspberry Pi

1. Ensure Raspberry Pi is on and connected to same wifi network
1. Test connection: ```ping raspberrypi.local```

1. Following https://code.visualstudio.com/docs/remote/ssh
    1. user@hostname: ```fydp@raspberrypi.local```  
    1. Platform: Linux

