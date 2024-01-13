# Commands For Quick Reference

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
