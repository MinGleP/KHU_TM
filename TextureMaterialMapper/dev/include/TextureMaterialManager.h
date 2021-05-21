#pragma once

#ifdef TEXTUREMATERIALMAPPER_EXPORTS
#define DLL_TYPE  __declspec(dllexport)   
#else
#define DLL_TYPE  __declspec(dllimport)   
#endif

#include "Def.h"
#include "TextureMapper.h"
#include "MaterialClassifier.h"
#include "FileManager.h"
#include<string>
#include<vector>

class DLL_TYPE TextureMaterialManager
{
public:
	
	TMErrCode initialize(std::string parameterPath);
	

private:
	TextureMapper textureMapper;
	MaterialClassifier materialClassifier;
	FileManager fileManager;
	Parameter parameters;
	TexturedMesh texturedMesh;
	ActiveSensorInfo aInfo;
	std::vector<ImgInfo> imgInfos;

};

