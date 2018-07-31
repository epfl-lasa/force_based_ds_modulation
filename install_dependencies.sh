tput setaf 6
echo "########################################################"
echo "# Install lib armadillo needed for the SVMGrad package #"
echo "########################################################"
tput sgr0
sudo apt-get install libarmadillo-dev
tput setaf 6
echo "################################"
echo "# Get all package dependencies #"
echo "################################"
tput sgr0
wstool init
wstool merge force_based_ds_modulation/dependencies.rosinstall
wstool merge kuka-lwr-ros/dependencies.rosinstall
wstool up
tput setaf 6
echo "##########################################################"
echo "# Compile libsvm and add executables to environment path #"
echo "##########################################################"
tput sgr0
cd libsvm
make
export PATH=$PATH:$(pwd)
cd ../..
tput setaf 6
echo "#####################"
echo "# Make all packages #"
echo "#####################"
tput sgr0
catkin_make
tput setaf 6
echo "#################################"
echo "# Everything done hopefully !!! #"
echo "#################################"
tput sgr0