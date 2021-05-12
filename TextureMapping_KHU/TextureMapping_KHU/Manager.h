#pragma once

#ifdef TEXTUREMAPPINGKHU_EXPORTS
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
private:
	TextureMapper textureMapper;
	MaterialClassifier materialClassifier;
	FileManager fileManager;
	Parameter parameters;
	TexturedMesh texturedMesh;
	ActiveSensorInfo aInfo;
	std::vector<ImgInfo> imgInfos;

public:

	int initialize(std::string parameterPath);
	//void initialize

	int runTextureMapping(TexturedMesh& textureMesh);
	int runMaterialClassification(TexturedMesh& textureMesh);

};



