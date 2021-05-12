#include "pch.h" // use stdafx.h in Visual Studio 2017 and earlier

#include "Manager.h"
#include<iostream>

int TextureMaterialManager::initialize(std::string parameterPath)
{
	std::cout << "Parameter Path : " << parameterPath << '\n';

	fileManager.readParameter(parameterPath, parameters);

	fileManager.loadMesh(parameters.meshPath, texturedMesh);

	// readPassiveSensorInfo Ãß°¡

	//fileManager.readActiveSensorInfo(parameters.atvSensorParamPath, aInfo);

	fileManager.loadImages(parameters.imagesFolder, imgInfos);


	return 1;
}
