#!/bin/bash

set -e

sudo apt-get update -y
curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash
sudo apt-get install -y git-lfs clang cmake python3-pip python3-venv
git lfs install --skip-repo

python3 -m venv .venv && source ./.venv/bin/activate

git clone https://gitlab.com/mit-mers/teaching/padm-project-2023f.git
pushd padm-project-2023f &> /dev/null

pip3 install -r requirements.txt

pushd ss-pybullet/pybullet_tools/ikfast/franka_panda
python3 setup.py

popd &> /dev/null

pushd pddl-parser
python3 setup.py install

popd &> /dev/null
popd &> /dev/null
