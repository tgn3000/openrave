OpenRAVE3
---------
This repository forks the original OpenRAVE, the Open Robotics Automation Virtual Environment, from

<https://github.com/rdiankov/openrave>

The repository allows the user to build this C++ package and its Python3 bindings on macOS and Linux.

Install Python3 and Boost on macOS
----------------------------------
* Install Homebrew
```
$ /usr/bin/ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
```
See <https://brew.sh/> for more details.

* Install third party libraries with Homebrew
```
$ brew install python3 libxml2 boost boost-python3
```

The packages are normally installed under `/usr/local/Cellar`, the header files are linked to `/usr/local/include`, and the libraries are linked to `/usr/local/lib`.

A more preferrable way to install Python3 on macOS is using its 64-bit installer (`python-3.x.y-macosx10.9.pkg`), which one can download from https://www.python.org/downloads/mac-osx/. It installs Python3 under 

```
/Library/Frameworks/Python.framework/Versions/3.x
```

Install Python3 and Boost on Linux
----------------------------------
* Specify the installation path of `OpenRAVE3` by
```
echo 'export OPENRAVE3_INSTALL_DIR='$HOME'/Projects/openrave3' >> ~/.bashrc
```
Add its subdirectory `lib` into the library search paths by
```
echo 'export LD_LIBRARY_PATH='$OPENRAVE3_INSTALL_DIR/lib':$LD_LIBRARY_PATH' >> ~/.bashrc
```

* Install Python3.x and Boost 1.7x

We show how to build Python 3.8 and Boost 1.71 from source files. Other similar versions can be installed analogously.

```
# Install Python 3.8
wget https://www.python.org/ftp/python/3.8.0/Python-3.8.0.tgz
tar xvfz Python-3.8.0.tgz
cd Python-3.8.0
./configure --prefix=$OPENRAVE3_INSTALL_DIR --enable-shared
make && make install
cd $OPENRAVE3_INSTALL_DIR/bin
./pip3 install numpy ipython
```

```
# Install Boost 1.71 
wget https://dl.bintray.com/boostorg/release/1.71.0/source/boost_1_71_0.tar.gz
tar xvfz boost_1_71_0.tar.gz
cd boost_1_71_0
bash bootstrap.sh --prefix=$OPENRAVE3_INSTALL_DIR --with-python=$OPENRAVE3_INSTALL_DIR/bin/python3
./b2 --prefix=$OPENRAVE3_INSTALL_DIR install
```

Install OpenRAVE3
-----------------
- Clone this repo
```
git clone git@github.com:tgn3000/openrave3.git
```
Edit accordingly Python3's version and Boost's version in `openrave3/src/CMakeLists.txt`.

(To be written ...)

- Final installation
```
cd openrave3/src
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=$OPENRAVE3_INSTALL_DIR ..
make -j4
make install
```

License
-------
* LGPL <https://www.gnu.org/licenses/license-list.html#LGPL>
