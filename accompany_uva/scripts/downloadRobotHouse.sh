
cd ../
mkdir -p res
cd res
mkdir -p testRobotHouse
cd testRobotHouse

wget -nc http://basterwijn.nl/bterwijn/Accompany/RobotHouse/map.pgm
wget -nc http://basterwijn.nl/bterwijn/Accompany/RobotHouse/map.yaml

mkdir -p camera1
cd camera1
wget -nc http://basterwijn.nl/bterwijn/Accompany/RobotHouse/camera1/2012-09-25-11-59-11.bag
wget -nc http://basterwijn.nl/bterwijn/Accompany/RobotHouse/camera1/background_images.zip
unzip -u background_images.zip
#wget -nc http://basterwijn.nl/bterwijn/Accompany/RobotHouse/camera1/res/bgmodel.xml
#wget -nc http://basterwijn.nl/bterwijn/Accompany/RobotHouse/camera1/res/camera_extrinsic.xml
#wget -nc http://basterwijn.nl/bterwijn/Accompany/RobotHouse/camera1/res/camera_intrinsic.xml
#wget -nc http://basterwijn.nl/bterwijn/Accompany/RobotHouse/camera1/res/frame.dat
#wget -nc http://basterwijn.nl/bterwijn/Accompany/RobotHouse/camera1/res/params.xml
#wget -nc http://basterwijn.nl/bterwijn/Accompany/RobotHouse/camera1/res/prior.txt
ln -sf ./2012-09-25-11-59-11.bag video.bag
rosrun accompany_uva retopicBag.py video.bag retopic.bag /camera1
cd ../

#mkdir -p camera2
#cd camera2
#wget -nc http://basterwijn.nl/bterwijn/Accompany/RobotHouse/camera2/2012-09-25-11-59-17.bag
#wget -nc http://basterwijn.nl/bterwijn/Accompany/RobotHouse/camera2/res/bgmodel.xml
#wget -nc http://basterwijn.nl/bterwijn/Accompany/RobotHouse/camera2/res/camera_extrinsic.xml
#wget -nc http://basterwijn.nl/bterwijn/Accompany/RobotHouse/camera2/res/camera_intrinsic.xml
#wget -nc http://basterwijn.nl/bterwijn/Accompany/RobotHouse/camera2/res/frame.dat
#wget -nc http://basterwijn.nl/bterwijn/Accompany/RobotHouse/camera2/res/params.xml
#wget -nc http://basterwijn.nl/bterwijn/Accompany/RobotHouse/camera2/res/prior.txt
#ln -sf ./2012-09-25-11-59-17.bag video.bag
#cd ../

mkdir -p camera3
cd camera3
wget -nc http://basterwijn.nl/bterwijn/Accompany/RobotHouse/camera3/2012-09-25-11-59-13.bag
wget -nc http://basterwijn.nl/bterwijn/Accompany/RobotHouse/camera3/background_images.zip
unzip -u background_images.zip
#wget -nc http://basterwijn.nl/bterwijn/Accompany/RobotHouse/camera3/res/bgmodel.xml
#wget -nc http://basterwijn.nl/bterwijn/Accompany/RobotHouse/camera3/res/camera_extrinsic.xml
#wget -nc http://basterwijn.nl/bterwijn/Accompany/RobotHouse/camera3/res/camera_intrinsic.xml
#wget -nc http://basterwijn.nl/bterwijn/Accompany/RobotHouse/camera3/res/frame.dat
#wget -nc http://basterwijn.nl/bterwijn/Accompany/RobotHouse/camera3/res/params.xml
#wget -nc http://basterwijn.nl/bterwijn/Accompany/RobotHouse/camera3/res/prior.txt
ln -sf ./2012-09-25-11-59-13.bag video.bag
rosrun accompany_uva retopicBag.py video.bag retopic.bag /camera3
cd ../

mkdir -p cameras
cd cameras
wget -nc  http://basterwijn.nl/bterwijn/Accompany/RobotHouse/cameras/params.xml
wget -nc  http://basterwijn.nl/bterwijn/Accompany/RobotHouse/cameras/prior.txt
wget -nc  http://basterwijn.nl/bterwijn/Accompany/RobotHouse/cameras/camera_intrinsic1.xml
wget -nc  http://basterwijn.nl/bterwijn/Accompany/RobotHouse/cameras/camera_extrinsic1.xml
wget -nc  http://basterwijn.nl/bterwijn/Accompany/RobotHouse/cameras/camera_intrinsic3.xml
wget -nc  http://basterwijn.nl/bterwijn/Accompany/RobotHouse/cameras/camera_extrinsic3.xml
wget -nc  http://basterwijn.nl/bterwijn/Accompany/RobotHouse/cameras/frame.dat
wget -nc  http://basterwijn.nl/bterwijn/Accompany/RobotHouse/cameras/entryExit.txt
wget -nc  http://basterwijn.nl/bterwijn/Accompany/RobotHouse/cameras/makeListOfImages.sh
chmod +x ./makeListOfImages.sh
./makeListOfImages.sh
cd ../

