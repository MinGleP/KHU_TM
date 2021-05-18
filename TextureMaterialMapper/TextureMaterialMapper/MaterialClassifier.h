#pragma once

#include<string>
#include<vector>

#include "Def.h"

class MaterialClassifier
{
private:


public:
	int initialize();
	int runMaterialClassification(std::vector<TexturedMesh>& textureMesh, std::vector<ImgInfo>& imgInfo, std::vector<PassiveSensorInfo>& pInfo, std::vector<ActiveSensorInfo>& aInfo, std::vector<Material>& material);
	int getStatus();
	int getClassifiedMaterial();
};