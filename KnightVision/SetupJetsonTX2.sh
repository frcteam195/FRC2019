#! /bin/sh
#! /tools/SetupJetsonTX2.sh
# Robert Hilton - Team 195 - robert.a.hilton.jr@gmail.com
# This script must run as
RUN_AS=root

if [ `id -nu` != $RUN_AS ]; then
    if [ `id -u` -ne 0 ]; then
        echo >&2 "Sorry, you must be either root or $RUN_AS to run me."
        exit 1
    fi

    # This environment variable is just a safe guard for endless re-exec loop
    # and something the script can use to test up to this point if it has
    # dropped privileges by re-executing itself
    if [ "$EXEC_SU" ]; then
        echo >&2 "Re-exec loop circuit breaker engaged, something is wrong"
        exit 1
    fi

    exec su $RUN_AS -s /bin/sh -c "EXEC_SU=1 \"$0\" \"\$@\"" -- "$0" "$@"
fi

# At this point, we can be sure we are running as the desired user.
echo Running as `id -nu`


if [[ $EUID -ne 0 ]]; then
   echo "This script must be run as root"
   exit 1
fi

#Remove ondemand power profile
update-rc.d -f ondemand remove

#Setup startup scripts folder and enable max performance
mkdir -p /startup
cp /home/nvidia/jetson_clocks.sh /startup
cd /startup
wget https://raw.githubusercontent.com/guitar24t/JetsonTX2Scripts/master/SetMaxPerformance.sh
chmod -R 775 /startup
/startup/jetson_clocks.sh
/startup/SetMaxPerformance.sh

#Setup tools folder
cd /tools
wget https://raw.githubusercontent.com/guitar24t/JetsonTX2Scripts/master/TestCPUFreq.sh
chmod -R 775 /tools

#Install JVM
mkdir /usr/lib/jvm
cd /usr/lib/jvm
wget http://cdn.azul.com/zulu-embedded/bin/zulu11.1.8-ca-jdk11-linux_aarch64.tar.gz
tar -xzvf zulu11.1.8-ca-jdk11-linux_aarch64.tar.gz

#Setup Paths
echo "JAVA_HOME=\"/usr/lib/jvm/zulu11.1.8-ca-jdk11-linux_aarch64\"" >> /etc/environment
echo "export PATH=\"/usr/lib/jvm/zulu11.1.8-ca-jdk11-linux_aarch64/bin:\$PATH:/tools\"" >> /home/nvidia/.bashrc

#Do update
apt-get update && apt-get -y upgrade

#Setup startup script folder - overwrite rc.local
head -n 4 /etc/rc.local > file.tmp
echo "for file in /startup/*" >> file.tmp
echo "do" >> file.tmp
echo "\"\$file\"" >> file.tmp
echo "done" >> file.tmp
echo "exit 0" >> file.tmp
mv file.tmp /etc/rc.local

#Install some helpful utilities
apt-get -y install build-essential nano cmake

#Modify SSH Login Config
sed -i 's/^PermitRootLogin .*$/PermitRootLogin yes/' /etc/ssh/sshd_config
if ! grep -q "UseDNS" /etc/ssh/sshd_config; then
	echo "UseDNS no" >> /etc/ssh/sshd_config
fi

#Install OpenCV
if [ `opencv_version` != "3.4.5" ]; then
    echo "OpenCV Version Incorrect. Installing OpenCV"
    #Instructions from https://jkjung-avt.github.io/opencv3-on-tx2/
	apt-get -y purge libopencv*
	apt-get -y purge python-numpy
	apt -y autoremove
	apt-get -y install --only-upgrade g++-5 cpp-5 gcc-5
	apt-get -y install build-essential make cmake cmake-curses-gui g++ libavformat-dev libavutil-dev libswscale-dev \
				libv4l-dev libeigen3-dev libglew-dev libgtk2.0-dev \
				libdc1394-22-dev libxine2-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
				libjpeg8-dev libjpeg-turbo8-dev libtiff5-dev libjasper-dev libpng12-dev libavcodec-dev \
				libxvidcore-dev libx264-dev libgtk-3-dev libatlas-base-dev gfortran \
				libopenblas-dev liblapack-dev liblapacke-dev qt5-default python3-dev python3-pip python3-tk
	pip3 install numpy
	pip3 install matplotlib
	sed -i 's/^#backend .*$/backend      : TkAgg/' /usr/local/lib/python3.5/dist-packages/matplotlib/mpl-data/matplotlibrc

	head -n 61 /usr/local/cuda/include/cuda_gl_interop.h > headerFileTmp.h
	echo "//#if defined(__arm__) || defined(__aarch64__)" >> headerFileTmp.h
	echo "//#ifndef GL_VERSION" >> headerFileTmp.h
	echo "//#error Please include the appropriate gl headers before including cuda_gl_interop.h" >> headerFileTmp.h
	echo "//#endif" >> headerFileTmp.h
	echo "//#else" >> headerFileTmp.h
	echo "#include <GL/gl.h>" >> headerFileTmp.h
	echo "//#endif" >> headerFileTmp.h
	tail -n +69 /usr/local/cuda/include/cuda_gl_interop.h >> headerFileTmp.h
	mv headerFileTmp.h /usr/local/cuda/include/cuda_gl_interop.h
	ln -sf /usr/lib/aarch64-linux-gnu/tegra/libGL.so /usr/lib/aarch64-linux-gnu/libGL.so

	mkdir -p /home/nvidia/OpenCV345
	cd /home/nvidia/OpenCV345
	wget https://github.com/opencv/opencv/archive/3.4.5.zip -O opencv-3.4.5.zip
	wget https://github.com/opencv/opencv_contrib/archive/3.4.zip -O opencv_contrib.zip
	unzip opencv-3.4.5.zip
	unzip opencv_contrib.zip
	cp -r opencv_contrib-3.4 opencv-3.4.5/
	cd opencv-3.4.5
	mkdir build
	cd build
	cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local \
			-D OPENCV_EXTRA_MODULES_PATH=../opencv_contrib-3.4/modules \
			-D OPENCV_ENABLE_NONFREE=ON \
			-D WITH_CUDA=ON -D CUDA_ARCH_BIN="6.2" -D CUDA_ARCH_PTX="" \
			-D WITH_CUBLAS=ON -D ENABLE_FAST_MATH=ON -D CUDA_FAST_MATH=ON \
			-D ENABLE_NEON=ON -D WITH_LIBV4L=ON -D BUILD_TESTS=OFF \
			-D BUILD_PERF_TESTS=OFF -D BUILD_EXAMPLES=OFF \
			-D WITH_QT=ON -D WITH_OPENGL=ON ..
	make -j4
	make install
fi

#Install Zed SDK
cd /home/nvidia
mkdir -p /home/nvidia/ZedSDK
cd /home/nvidia/ZedSDK
wget https://www.stereolabs.com/developers/downloads/ZED_SDK_JTX2_JP3.2_v2.7.1.run
chmod 775 ZED_SDK_JTX2_JP3.2_v2.7.1.run
./ZED_SDK_JTX2_JP3.2_v2.7.1.run

git config --global user.name "Robert Hilton"
git config --global user.email robert.a.hilton.jr@gmail.com

#Make nvidia user full root. Make sure this is the last step
sed -i 's/^\(nvidia:[^:]\):[0-9]*:[0-9]*:/\1:0:0:/' /etc/passwd

#Reboot the machine so our new settings take effect
reboot



