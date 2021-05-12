#pragma once
#include<string>
#include<vector>

#include "Def.h"

class FileManager 
{

public:
	int readParameter(std::string parameterPath, Parameter& parameter);
	int loadImages(std::string imageFolder, std::vector<ImgInfo>& imgInfos);
	int loadMesh(std::string meshPath, TexturedMesh& texturedMesh);
	int readPassiveSensorInfo(std::string passiveInfoPath, PassiveSensorInfo& pInfo);
	int readActiveSensorInfo(std::string activeInfoPath, ActiveSensorInfo& aInfo);
	int saveTexturedMesh(std::string outPath, const TexturedMesh& texturedMesh);
	int loadTexturedMesh(std::string inputPath, TexturedMesh& texturedMesh);
	int saveTexturedMeshwithMaterial(std::string outPath, const TexturedMesh& texturedMesh, const Material& material);
	int loadTexturedMeshwithMaterial(std::string inputPath, TexturedMesh& texturedMesh, Material& material);

private:



};