# 16.413 Project

## Requirements
To make sure you have the right dependencies, run the following command in the home directory of this repository:
```
pip install -r python/requirements.txt
```
You also need to install the pddl-parser. You can follow the instructions on the [offical repo](https://github.com/pucrs-automated-planning/pddl-parser/tree/master). We have configured this as a submodule. To set this up, run the following commands in the terminal:
```
git submodule update --init --recursive
```
You then need to update compile the python package. To do this:
```
cd lib/pddl-parser
python3 setup.py install
```
For the last command, you may need to use sudo.

## Activity Planning
### PDDL Domain
TODO

### Approach, Files, Functions
TODO