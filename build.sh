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

echo "Building Pangolin"
cd ../..
git clone https://github.com/stevenlovegrove/Pangolin
cd Pangolin
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j 5

echo "Building Feature Compression"
cd ../..
git clone https://github.com/d-vo/featureCompression
cd featureCompression
mkdir build
mkdir install
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$(pwd)/../install
make -j 5 install



cd ../../../

echo "Uncompress vocabulary ..."

cd Vocabulary
tar -xf ORBvoc2.txt.tar.gz
cd ..

echo "Configuring and building ORB_SLAM2 ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
