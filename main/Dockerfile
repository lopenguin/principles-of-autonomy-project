FROM python:3.9

# install dependencies
RUN curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | bash && \
  apt-get update -y && \
  apt-get install -y clang cmake git-lfs && \
  git lfs install && \
  pip install numpy scipy pybullet scikit-learn

# build project
WORKDIR /app
COPY . /app

RUN cd ss-pybullet/pybullet_tools/ikfast/franka_panda && \
  python setup.py

WORKDIR /app

RUN cd pddl-parser && \
  python setup.py install
