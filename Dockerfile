FROM nvidia/cuda:9.0-runtime-ubuntu16.04

# openrave dependencies
RUN apt-get update
RUN apt-get -y install cmake g++ git ipython minizip python-dev python-h5py python-numpy python-scipy python-sympy qt4-dev-tools
RUN apt-get -y install libassimp-dev libavcodec-dev libavformat-dev libavformat-dev libboost-all-dev libboost-date-time-dev libbullet-dev libfaac-dev libglew-dev libgsm1-dev liblapack-dev liblog4cxx-dev libmpfr-dev libode-dev libogg-dev libpcrecpp0v5 libpcre3-dev libqhull-dev libqt4-dev libsoqt-dev-common libsoqt4-dev libswscale-dev libswscale-dev libvorbis-dev libx264-dev libxml2-dev libxvidcore-dev

# openscenegraph
RUN apt-get -y install libcairo2-dev libjasper-dev libpoppler-glib-dev libsdl2-dev libtiff5-dev libxrandr-dev
RUN git clone git://github.com/openscenegraph/OpenSceneGraph.git
RUN cd OpenSceneGraph && git checkout 1f89e6eb1087add6cd9c743ab07a5bce53b2f480 && mkdir build && cd build && cmake .. -DDESIRED_QT_VERSION=4 && make && make install

# fcl
RUN apt-get -y install libccd-dev
RUN git clone git://github.com/flexible-collision-library/fcl.git
RUN ln -sf /usr/include/eigen3/Eigen /usr/include/Eigen && cd fcl && git checkout 7075caf32ddcd5825ff67303902e3db7664a407a && mkdir build && cd build && cmake .. && make && make install

# collada-dom
RUN git clone git://github.com/rdiankov/collada-dom.git
RUN cd collada-dom && git checkout d37ae7532e350b87c88712e9f6ab4b1f440d20cd && mkdir build && cd build && cmake .. && make && make install

# openrave
RUN git clone git://github.com/rdiankov/openrave.git
RUN cd openrave && git checkout 9c79ea260e1c009b0a6f7c03ec34f59629ccbe2c && mkdir build && cd build && cmake .. -DOSG_DIR=/usr/local/lib64/ && make && make install

# other qqq dependencies
RUN apt-get update && apt-get -y install --no-install-recommends cmake g++ g++-multilib make python python-tk python-pip libcudnn7=7.0.5.15-1+cuda9.0
RUN pip install --upgrade pip==19.0.3 setuptools==39.1.0
RUN pip install --upgrade --ignore-installed numpy==1.14.5 scipy==1.1.0 pybullet==2.4.1 sympy==0.7.1 tensorflow-gpu==1.10.0

# pddlstream + fastdownward
RUN git clone git://github.com/caelan/pddlstream.git
RUN cd pddlstream && git checkout d7645b96906e9c6167af631fb9dc16e4b784d61d && git submodule update --init --recursive && ./FastDownward/build.py

# copy qqq
COPY . /qqq

