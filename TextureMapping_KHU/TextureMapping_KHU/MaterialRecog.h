#pragma once

#include<string>
#include<vector>

struct Material
{

};

struct TextureMesh
{

};

struct ImgInfo
{

};

struct PassiveSensorInfo
{

};

struct ActiveSensorInfo
{

};

class MaterialRecog
{
private:
	std::vector<TextureMesh> textureMesh;
	std::vector<Material> material;
	std::vector<ImgInfo> imgInfo;
	std::vector<PassiveSensorInfo> pInfo;
	std::vector<ActiveSensorInfo> aInfo;


public:
	int initialize();
	int runMaterialClassification(std::vector<TextureMesh>& textureMesh, std::vector<ImgInfo>& imgInfo, std::vector<PassiveSensorInfo>& pInfo, std::vector<ActiveSensorInfo>& aInfo, std::vector<Material>& material);
	int getStatus();
	int getClassifiedMaterial();


};