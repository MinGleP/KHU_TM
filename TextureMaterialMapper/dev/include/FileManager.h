#pragma once

#include <string>

#include "Def.h"
#include "TextureMapper.h"
#include "MaterialClassifier.h"
#include "FileManager.h"

class FileManager
{
public:
	TMErrCode readParamater(std::string parameterPath, Parameter& parameter);
	TMErrCode loadImages(std::string imageFolder, std::vector<ImgInfo>& imgInfos);
	TMErrCode loadMesh(std::string meshPath, TexturedMesh& texturedMesh);
	TMErrCode readPassiveSensorInfo(const std::vector<std::string>& passiveInfoPath, std::vector<ImgInfo>& imgInfos);
	TMErrCode readActiveSensorInfo(std::string activeInfoPath, ActiveSensorInfo& aInfo);
	TMErrCode saveTexturedMesh(std::string outPath, const TexturedMesh& texturedMesh);
	TMErrCode loadTexturedMesh(std::string inputPath, TexturedMesh& texturedMesh);
	TMErrCode saveTexturedMeshwithMaterial(std::string outPath, const TexturedMesh& texturedMesh, const Material& material);
	TMErrCode loadTexturedMeshwithMaterial(std::string inputPath, TexturedMesh& texturedMesh, Material& material);

private:

};

