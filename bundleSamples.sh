
Version="0.9.0"

if [ -z "$1" ]
then
   filename="SolAR_SLAM_$Version"
else
   filename=$1
fi

# Update configuration files by replacing win-cl-1.1 by linux in module paths
echo "**** Update module path in configuration file (win-cl-14.1 -> linux-gcc)"
sed -i 's/win-cl-14.1/linux-gcc/' $PWD/bin/Debug/*_conf.xml
sed -i 's/win-cl-14.1/linux-gcc/' $PWD/bin/Release/*_conf.xml
sed -i 's/win-cl-14.1/linux-gcc/' $PWD/SolARSample*/*_conf.xml
sed -i 's/win-cl-14.1/linux-gcc/' $PWD/SolARPipeline*/tests/SolARPipelineTest*/*_conf.xml

echo "**** Install dependencies locally"
remaken install packagedependencies.txt
remaken install packagedependencies.txt -c debug

echo "**** Bundle dependencies in bin folder"
for file in $(find ./SolARSample* ./SolARPipeline*/tests/SolARPipelineTest* -path "*_conf.xml")
do
   echo "install dependencies for config file: $file"
   remaken bundleXpcf $file -d ./bin/Release -s modules
   remaken bundleXpcf $file -d ./bin/Debug -s modules -c debug
done

# Make data configuration files point to bundle libraries as well
cp $PWD/data/SolARSample_SLAM_TUM_conf.xml $PWD/data/SolARSample_SLAM_TUM_conf.xml.copy
cp $PWD/data/SolARPipelineTest_SLAM_TUM_conf.xml $PWD/data/SolARPipelineTest_SLAM_TUM_conf.xml.copy
sed -i 's/path=".*"/path="modules"/g' $PWD/data/SolARSample_SLAM_TUM_conf.xml
sed -i 's/path=".*"/path="modules"/g' $PWD/data/SolARPipelineTest_SLAM_TUM_conf.xml

cp ./runFromBundle.sh ./run.sh
mv ./run.sh ./bin/Release/
cp ./runFromBundle.sh ./run.sh
mv ./run.sh ./bin/Debug


zip --symlinks -r "./bin/${filename}_release.zip" ./bin/Release ./README.md ./installData.sh ./LICENSE data/SolARPipelineTest_SLAM_TUM_conf.xml data/SolARSample_SLAM_TUM_conf.xml data/tum_camera_calibration.yml
zip --symlinks -r "./bin/${filename}_debug.zip" ./bin/Debug ./README.md ./installData.sh ./LICENSE data/SolARPipelineTest_SLAM_TUM_conf.xml data/SolARSample_SLAM_TUM_conf.xml data/tum_camera_calibration.yml

# Restore data configuration file values for clean git status
mv $PWD/data/SolARSample_SLAM_TUM_conf.xml.copy $PWD/data/SolARSample_SLAM_TUM_conf.xml
mv $PWD/data/SolARPipelineTest_SLAM_TUM_conf.xml.copy $PWD/data/SolARPipelineTest_SLAM_TUM_conf.xml