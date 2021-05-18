#include "pch.h"
#include "TextureMaterialManager.h"

#include "opencv2\opencv.hpp"

TMErrCode TextureMaterialManager::initialize(std::string parameterPath)
{
	TMErrCode errCode;
	Parameter param;

	errCode = fileManager.readParamater(parameterPath, param);

	// todo 결과 확인용 - 지울 것
	std::cout << param.inputType << std::endl;
	std::cout << param.meshPath << std::endl;
	std::cout << param.imagesFolder << std::endl;
	std::cout << param.psvSensorCameraParamPath << std::endl;
	std::cout << param.atvSensorParamPath << std::endl;


	return errCode;
}

