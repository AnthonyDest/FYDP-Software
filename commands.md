# Commands For Quick Reference

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
