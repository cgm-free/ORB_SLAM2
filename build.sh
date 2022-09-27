echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../../

echo "Uncompress vocabulary ..."

cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

echo "Configuring and building ORB_SLAM2 ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j


cd ..
#./Examples/Monocular/mono_tum Vocabulary/ORBvoc.txt Examples/Monocular/TUM1.yaml ../DataSet/TUM_Dataset/rgbd_dataset_freiburg1_xyz
./Examples/Stereo/stereo_euroc Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml /home/cgm/DataSet/EuRoC_Dataset/MH_01_easy/mav0/cam0/data /home/cgm/DataSet/EuRoC_Dataset/MH_01_easy/mav0/cam1/data Examples/Stereo/EuRoC_TimeStamps/MH01.txt

# clion 参数设置 运行MH_01_easy
#../../Vocabulary/ORBvoc.txt
#../../Examples/Stereo/EuRoC.yaml
#/home/cgm/DataSet/EuRoC_Dataset/MH_01_easy/mav0/cam0/data
#/home/cgm/DataSet/EuRoC_Dataset/MH_01_easy/mav0/cam1/data
#/home/cgm/ORB_SLAM2_detailed_comments-master/Examples/Monocular/EuRoC_TimeStamps/MH01.txt

# clion 参数设置 运行MH_05_difficult
#../../Vocabulary/ORBvoc.txt
#../../Examples/Stereo/EuRoC.yaml
#/home/cgm/DataSet/EuRoC_Dataset/MH_05_difficult/mav0/cam0/data
#/home/cgm/DataSet/EuRoC_Dataset/MH_05_difficult/mav0/cam1/data
#/home/cgm/ORB_SLAM2_detailed_comments-master/Examples/Monocular/EuRoC_TimeStamps/MH05.txt
