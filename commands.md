# Commands For Quick Reference

### Run Teleop:  
`sudo /home/fydp/Documents/FYDP-Software/fydp_venv/bin/python /home/fydp/Documents/FYDP-Software/main.py --teleop`

### Scan I2C
`i2cdetect -y 1`

### SSH Copy
`scp -r fydp@raspberrypi:/home/fydp/Documents/FYDP-Software/videos "C:/Users/Anthony/Documents/School/4B/FYDP/Feb_19"`

## Pip Environment Management

### Create a new environment:
`python -m venv <env-name>`  
`python -m venv fydp_venv`

### Activate an environment:
`.\<env-name>\Scripts\activate`  
`.\fydp_venv\Scripts\activate`

### Deactivate the current environment:
`deactivate`

### Install packages
`pip install <package-name>`

### Export required packages:
`pip freeze > requirements.txt`

### Import required packages:
`pip install -r requirements.txt`

## Git

### Clean up branches:
`git branch | %{ $_.Trim() } | ?{ $_ -ne 'master' } | ?{ $_ -ne 'main'} | ?{ $_ -ne 'develop'} | %{ git branch -D $_ }`


---
Not in use:

## Conda Environment Management

### Create a new environment:
`conda create --name <env-name>`

### List all environments:
`conda env list`

### Activate an environment:
`conda activate <env-name>`  
If terminal has: `(<env-name>)` in the cmd, the env was activated successfully.  
Else, try `conda init`

### Deactivate the current environment:
`conda deactivate`

### Install packages
`conda install <package-name>`

### Export an environment:
`conda env export > environment.yml`

### Import an environment:
`conda env create -f environment.yml`
