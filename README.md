# SnoMo Capstone

Welcome to SnoMo! SnoMo is an innovative autonomous ice-resurfacing robot developed as the centerpiece of our 2024 Capstone Project.

## About SnoMo
**What is SnoMo?** SnoMo is an autonomous ice-resurfacing robot designed to tackle the challenges traditional ice-resurfacers face. It offers an affordable, effective, and easy solution for maintaining high-quality ice rinks, both indoors and outdoors. For more information, please check out the [SnoMo Project Website](https://24capstone.wixsite.com/my-site-1

## See SnoMo In Action
[![Watch the Video](https://img.youtube.com/vi/F2YSc3F6f1k/maxresdefault.jpg)](https://youtu.be/F2YSc3F6f1k)

# Initial Setup

## Base Installs

1. [Visual Studio Code](https://code.visualstudio.com/download).
2. Add [Python](https://www.python.org/downloads/) to Path [windows](https://docs.python.org/3.9/using/windows.html)
3. [Git](https://git-scm.com/downloads)
4. [RealVNC](https://www.realvnc.com/en/connect/download/viewer/)

## Clone the Repository

```bash
git clone https://github.com/AnthonyDest/FYDP-Software.git
```

## Configure Visual Studio Code
Import fydp_base profile:
https://vscode.dev/profile/github/ee790e3fa4f2fad9bd25d6a99199eab6

## Create and Activate Venv
```
python -m venv fydp_venv
.\fydp_venv\Scripts\activate
pip install -r requirements.txt
```

# Pi Setup

## Verify Connection to Pi:
1. Ensure Raspberry Pi is on and connected to same wifi network
1. Test connection: ```ping raspberrypi.local```

### If No Network Connection Available:
1. Connect via ethernet to Pi
2. Set a [Static IP](https://www.trendnet.com/press/resource-library/how-to-set-static-ip-address) on connected port

## Setup Remote SSH Connection VS Code
Following https://code.visualstudio.com/docs/remote/ssh  
Import ssh_config to remote explorer settings  
   1. user@hostname: `fydp@raspberrypi.local`  
   2. Platform: `Linux`

## Setup Remote VNC Viewer Connection
Local Connection:
   1. VNC Server: `raspberrypi.local`
   2. Name: `Pi_Local`
   
