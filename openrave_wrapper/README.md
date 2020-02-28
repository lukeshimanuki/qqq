# lis-openrave

This module contains manipulation primatives useful for manipulation that use OpenRAVE. The only dependecy is OpenRAVE. 

## OpenRAVE Installation

Documentation - http://www.openrave.org/docs/latest_stable/

Examples - http://openrave.org/docs/0.8.2/examples/, http://openrave.org/docs/latest_stable/tutorials/openravepy_examples/

Users List - http://openrave-users-list.185357.n3.nabble.com/

Instructions - http://openrave.org/docs/latest_stable/coreapihtml/installation.html

OpenRAVE only reliably installs on Linux systems. However, I was able to install OpenRAVE on OS X using MacPorts. 

Clone the OpenRAVE repository from the Github https://github.com/rdiankov/openrave.
Choose between the "latest_stable" and "master" branches. The "master" branch is more experimental but may have fixes for recent bugs.

```
git clone --branch latest_stable https://github.com/rdiankov/openrave.git
git clone --branch master https://github.com/rdiankov/openrave.git
```

Install the following dependencies - cmake, boost, libxml2, glew, py27-numpy, py27-pyplusplus, Qt4, Coin3D, SoQt, ffmpeg 

Ubuntu 14.04 - https://scaron.info/teaching/installing-openrave-on-ubuntu-14.04.html, http://fsuarez6.github.io/blog/openrave-trusty/
Ubuntu 16.04 - https://scaron.info/teaching/installing-openrave-on-ubuntu-16.04.html

Then, apply the following commands in your OpenRAVE installation.

```
mkdir build
cd build
cmake ..
make
sudo make install
```

Finally, add OpenRAVE to your ```PYTHONPATH``` by adding the following command to ```~/.bash_rc``` (Linux) or ```~/.bash_profile``` (OS X):

```
export PYTHONPATH=$PYTHONPATH:`openrave-config --python-dir`
```

### Mac OS X Notes

The Mac OS X installation instructions use MacPorts to install OpenRAVE's dependencies. I've had success installing using MacPorts. MacPorts and Homebrew do not interact well. If you use Homebrew instead of MacPorts, I would installing the dependecies using ```brew install ...``` instead of ```sudo port install ...```.

MacPorts - https://www.macports.org/

Homebrew - https://brew.sh/

## Help

Contact Caelan Garrett at caelan@csail.mit.edu for help with installation.
