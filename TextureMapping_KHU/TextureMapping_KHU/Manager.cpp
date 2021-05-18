#include "pch.h" // use stdafx.h in Visual Studio 2017 and earlier

#include "Manager.h"
#include<iostream>

int TextureMaterialManager::initialize(std::string parameterPath)
{
	std::cout << "Parameter Path : " << parameterPath << '\n';

	fileManager.readParameter(parameterPath, parameters);

	fileManager.loadMesh(parameters.meshPath, texturedMesh);

	if (parameters.psvSensorCameraParamPath != "" && parameters.psvSensorImagesParamPath != "")
	{
		std::vector<std::string> passiveInfoPath;
		passiveInfoPath.push_back(parameters.psvSensorCameraParamPath);
		passiveInfoPath.push_back(parameters.psvSensorImagesParamPath);
		fileManager.readPassiveSensorInfo(passiveInfoPath, imgInfos);
	}

	//fileManager.readActiveSensorInfo(parameters.atvSensorParamPath, aInfo);

	fileManager.loadImages(parameters.imagesFolder, imgInfos);

	return 1;
}
