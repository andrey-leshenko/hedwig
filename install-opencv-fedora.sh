# OpenCV Fedora Installation Script
# Based on http://milq.github.io/install-opencv-ubuntu-debian

# FAQ:

# Q: No rule to make target `/usr/lib/x86_64-linux-gnu/libGL.so'
# A: http://techtidings.blogspot.co.il/2012/01/problem-with-libglso-on-64-bit-ubuntu.html

# Keep Fedora up to date

sudo dnf -y check-update upgrade

# Install needed packages

PKGS=''

# Build tools:
PKGS+=' gcc gcc-c++ make cmake'
# GUI:
PKGS+=' qt-devel vtk-devel'
# Media I/O:
PKGS+=' zlib-devel libjpeg-turbo-devel libwebp-devel libpng-devel libtiff-devel jasper-devel OpenEXR-devel gdal-devel'
# Video I/O:
PKGS+=' libdc1394-devel libv4l-devel gstreamer-plugins-base-devel'
# Parallelism and linear algebra libraries:
PKGS+=' tbb-devel eigen3-devel'
# Documentation:
PKGS+=' doxygen'
# Downloading and extracting the source code:
PKGS+=' unzip wget'

sudo dnf install -y $PKGS

# Download, compile and install. (You can change 3.1.0 for the last stable version)

wget -nc https://github.com/opencv/opencv/archive/3.1.0.zip

unzip 3.1.0.zip
#rm 3.1.0.zip
mv opencv-3.1.0 OpenCV
cd OpenCV
mkdir build
cd build

FLAGS=''

FLAGS+=' -DCMAKE_BUILD_TYPE=RELEASE'
FLAGS+=' -DBUILD_DOCS=ON -DBUILD_EXAMPLES=OFF -DUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF'
FLAGS+=' -DWITH_QT=ON -DWITH_OPENGL=ON -DFORCE_VTK=ON -DWITH_TBB=ON -DWITH_EIGEN=ON -DWITH_GDAL=ON -DWITH_XINE=ON'
# Workaround. See: https://github.com/opencv/opencv/pull/6541
FLAGS+=' -DENABLE_PRECOMPILED_HEADERS=OFF '

cmake $FLAGS ..

make -j4
sudo make install
sudo ldconfig
