# Final Project - Principles of Autonomy and Decision Making, Fall 2023

This repo contains instructions for setting up the simulator for your projects. Cloning this repo is optional - see the installation instructions below.

## Installation Instructions

We've prepared a few ways for you to get the simulator setup. Feel free to use any of them. If you aren't running Ubuntu, you may want to consider working in an [Ubuntu 20.04 VM](https://ist.mit.edu/vmware-fusion
). For M1 MacOS users, please follow the following [blog](https://medium.com/@paulrobu/how-to-run-ubuntu-22-04-vms-on-apple-m1-arm-based-systems-for-free-c8283fb38309) to get Ubuntu 20.04 VM setup on your machine.

1. A set of instructions to copy/paste for Ubuntu (tested on 20.04)
1. An install script for Ubuntu (tested on 20.04)

#### Dependencies

* Python 3.8 `sudo apt-get install python3-dev`
* g++ `sudo apt-get install g++`


### Option 1: Install on Ubuntu 20.04 Manually

We're assuming you already have Python 3.8 installed. The steps below will setup the rest of the Python environment for the project.

#### Dependencies

* Python >=3.8

#### Installation

We'll assume `python` is Python 3.8

1. Install non-Python dependencies:
  ```sh
  curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash
  sudo apt-get install git-lfs clang cmake python3-pip python3-venv  && git lfs install --skip-repo
  ```
2. Make your own repository in Github and then clone it. While inside the directory for that repository, clone this repository and then remove the `.git` subfolders.
  ```sh
  git clone https://gitlab.com/mit-mers/teaching/padm-project-2023f.git
  cd padm-project-2023f
  rm -rf .git
  ```
3. Install Python packages: `pip install -r requirements.txt`
4. Build the project:
  ```sh
  cd ss-pybullet/pybullet_tools/ikfast/franka_panda/ && \
  python setup.py
  cd - && cd pddl-parser && \
  python setup.py install
  ```

#### Test

Run this from the root of *your* repo

```sh
source ./.venv/bin/activate && cd padm-project-2023f/ && python minimal_example.py
```

### Option 2: Run on Ubuntu 20.04

This convenience script automates the same steps enumerated above. It doesn't assume you have Python 3.8 installed. Make sure that you have first created your own repository and are inside that repository after cloning it.

```sh
curl -s https://gitlab.com/mit-mers/teaching/padm-project-2023f/-/raw/main/install.sh | bash
```

#### Test

```sh
source ./.venv/bin/activate && cd padm-project-2023f/ && python minimal_example.py
```

### Option 3: Run with Docker [NOT WORKING]

*We're leaving this here in case there are any enterprising Docker experts in the class who can figure out the `DISPLAY` variable. Let us know if you do! We've been unsuccessful so far.*

We can create a Docker container from this repo. This should work on any OS, though you'll need to configure the Docker `DISPLAY` variable for your host.

#### Dependencies

* [Docker](https://www.docker.com/)
* [git lfs](https://git-lfs.github.com/)

#### Instructions

1. Make your own repository and then clone it. Go to the repository directory using `cd`
2. Clone this repository within your personal repository.
4. Build the image: `docker build -t padm-project-2023f .`

### Trajectory Optimization Installation Instructions

For the final piece you can either use SNOPT/IPOPT solvers. SNOPT is a paid software but you can get access to via pyDrake which provides its own implementation of the software. It also provides IPOPT solver in addition to bunch of other solvers as well but since you will be working with non-linear constrained optimization problems, these solvers are pretty much state-of-the-art for them. To install pydrake follow these instructions inside your virtual environment.

```sh
pip install --upgrade pip
pip install drake pydrake
```

More information on how to use the library can be found [here](https://deepnote.com/workspace/Drake-0b3b2c53-a7ad-441b-80f8-bf8350752305/project/Tutorials-2b4fc509-aef2-417d-a40d-6071dfed9199/%2Fnonlinear_program.ipynb) in addition to the chapters that are posted on the project guidelines document.
