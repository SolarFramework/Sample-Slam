@echo off
:: Download bag of words vocabulary
curl https://github.com/SolarFramework/SolARModuleFBOW/releases/download/fbowVocabulary/fbow_voc.zip -L -o fbow_voc.zip
powershell Expand-Archive fbow_voc.zip -DestinationPath .\data -F
del fbow_voc.zip

curl https://repository.solarframework.org/generic/FbowVoc/popsift_uint8.fbow -L -o ./data/fbow_voc/popsift_uint8.fbow

:: Download yolact and fcn models
curl https://repository.solarframework.org/generic/LearnedModels/FCN/fcn_resnet50.onnx -L -o .\data\fcn_resnet50.onnx
curl https://repository.solarframework.org/generic/LearnedModels/yolact/yolact.onnx -L -o .\data\yolact.onnx

:: Download PSPNet and DeepLabV3 models 
echo Download PSPNet model 
curl https://repository.solarframework.org/generic/LearnedModels/PSPNet.zip -L -o PSPNet.zip
powershell Expand-Archive PSPNet.zip -DestinationPath .\data -F
del PSPNet.zip
echo Download DeepLabV3 model 
curl https://repository.solarframework.org/generic/LearnedModels/DeepLabV3.zip -L -o DeepLabV3.zip
powershell Expand-Archive DeepLabV3.zip -DestinationPath .\data -F
del DeepLabV3.zip

:: Download a AR device capture
curl https://repository.solarframework.org/generic/captures/hololens/bcomLab/loopDesktopA.zip -L -o loopDesktopA.zip
powershell Expand-Archive loopDesktopA.zip -DestinationPath .\data -F
del loopDesktopA.zip
curl https://repository.solarframework.org/generic/captures/hololens/hololens_calibration.json -L -o .\data\hololens_calibration.json

:: Download  captures
curl https://vision.in.tum.de/rgbd/dataset/freiburg3/rgbd_dataset_freiburg3_long_office_household_validation-rgb.avi -L -o .\data\rgbd_dataset_freiburg3_long_office_household_validation-rgb.avi

