# Get temperature at start
file='/sys/class/thermal/thermal_zone0/temp'
tempbegin=$(cat $file)

python3 -m pip install --upgrade pip
sudo apt-get update
sudo apt-get -y upgrade
pip install --upgrade pip setuptools wheel
sudo apt -y install libhdf5-dev libatlas-base-dev
sudo apt -y install build-essential cmake pkg-config libjpeg-dev libtiff5-dev libjasper-dev libpng-dev libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libxvidcore-dev libx264-dev libfontconfig1-dev libcairo2-dev libgdk-pixbuf2.0-dev libpango1.0-dev libgtk2.0-dev libgtk-3-dev libatlas-base-dev gfortran libhdf5-dev libhdf5-serial-dev libhdf5-103 python3-pyqt5 python3-dev -y
# sudo apt-get install -y libhdf5-dev libhdf5-serial-dev python3-pyqt5 libatlas-base-dev libjasper-dev -y
pip install opencv-contrib-python==4.5.3.56
pip install -U numpy
pip install apriltag
pip install imutils



# Install java for running photonvision
sudo apt install default-jdk

# install our latest software for apriltag
if [! -d "apriltag" ]; then
  echo "Make directory for apriltag test"
  mkdir apriltag
fi

git config --global user.email "kglasnapp@gmail.com"
git config user.name 'Keith Glasnapp'

echo "Clone latest directory for apriltag"
git config --global credential.helper 'cache --timeout=3600'
git clone https://thedirtymechanics.com/bitbucket/scm/rob22/apriltag.git

echo ***** compile apriltag ******
cd ~/apriltag
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
sudo make install
cd ~
python -c '
import cv2
print("cv2 version:", cv2.__version__)
import numpy
print("numpy version", numpy.__version__)
'
java -version

tempEnd=$(cat $file)
echo Temp at start - temp at end
echo "$tempBegin / 1000" | bc
echo "$tempEnd / 1000" | bc