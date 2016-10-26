# OpenCV Ubuntu Installation Script
# Based on http://milq.github.io/install-opencv-ubuntu-debian

# Keep Ubuntu up to date

sudo apt-get -y update
sudo apt-get -y upgrade
sudo apt-get -y dist-upgrade
sudo apt-get -y autoremove

# Install needed packages

PKGS=''

# Build tools:
PKGS+=' build-essential cmake'

# GUI (if you want to use GTK instead of Qt, replace 'qt5-default' with 'libgtkglext1-dev' and remove '-DWITH_QT=ON' option in CMake):
sudo apt-get install -y qt5-default libvtk6-dev
PKGS+=' qt5-default libvtk6-dev'

# Media I/O:
PKGS+=' zlib1g-dev libjpeg-dev libwebp-dev libpng-dev libtiff5-dev libjasper-dev libopenexr-dev libgdal-dev'

# Video I/O:
PKGS+=' libdc1394-22-dev libavcodec-dev libavformat-dev libswscale-dev libtheora-dev libvorbis-dev libxvidcore-dev libx264-dev yasm libopencore-amrnb-dev libopencore-amrwb-dev libv4l-dev libxine2-dev'

# Parallelism and linear algebra libraries:
PKGS+=' libtbb-dev libeigen3-dev'

# Python:
##PKGS+=' python-dev python-tk python-numpy python3-dev python3-tk python3-numpy'

# Java:
##PKGS+=' ant default-jdk'

# Documentation:
PKGS+=' doxygen'

sudo apt-get install -y $PKGS

# Download, compile and install. (You can change 3.1.0 for the last stable version)

sudo apt-get install -y unzip wget
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

#cmake -DCMAKE_BUILD_TYPE=RELEASE -DWITH_QT=ON -DWITH_OPENGL=ON -DFORCE_VTK=ON -DWITH_TBB=ON -DWITH_GDAL=ON -DWITH_XINE=ON -DBUILD_EXAMPLES=ON ..

make -j4
sudo make install
sudo ldconfig
