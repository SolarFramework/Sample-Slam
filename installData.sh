# Download bag of words vocabulary
curl https://github.com/SolarFramework/SolARModuleFBOW/releases/download/fbowVocabulary/fbow_voc.zip -L -o fbow_voc.zip
unzip -o fbow_voc.zip -d ./data
rm fbow_voc.zip

curl https://repository.solarframework.org/generic/FbowVoc/popsift_uint8.fbow -L -o ./data/fbow_voc/popsift_uint8.fbow --create-dirs

:: Download yolact and fcn models
curl https://repository.solarframework.org/generic/learnedModels/FCN/fcn_resnet50.onnx -L -o .\data\fcn_resnet50.onnx
curl https://repository.solarframework.org/generic/learnedModels/yolact/yolact.onnx -L -o .\data\yolact.onnx

:: Download PSPNet and DeepLabV3 models 
curl https://repository.solarframework.org/generic/LearnedModels/PSPNet.zip -L -o PSPNet.zip
unzip -o PSPNet.zip -d ./data
rm PSPNet.zip 
curl https://repository.solarframework.org/generic/LearnedModels/DeepLabV3.zip -L -o DeepLabV3.zip
unzip -o DeepLabV3.zip -d ./data
rm DeepLabV3.zip 

# Download  captures
curl https://repository.solarframework.org/generic/captures/hololens/bcomLab/loopDesktopA.zip -L -o loopDesktopA.zip
unzip -o loopDesktopA.zip -d ./data
rm loopDesktopA.zip

curl https://vision.in.tum.de/rgbd/dataset/freiburg3/rgbd_dataset_freiburg3_long_office_household_validation-rgb.avi -L -o ./data/rgbd_dataset_freiburg3_long_office_household_validation-rgb.avi

