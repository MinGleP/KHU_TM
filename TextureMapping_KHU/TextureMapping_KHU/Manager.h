#ifndef __MANAGER_H
#define __MANAGER_H

#ifdef DLLEXPORT
#define DLL_TYPE  __declspec(dllexport)   
#else
#define DLL_TYPE  __declspec(dllimport)   
#endif

#include "TextureMapper.h"
#include<string>

namespace tmManger {

class TextureMaterialManager 
{
private:


public:

	int initialize(std::string parameterPath);
	//void initialize

	int runTextureMapping(TexturedMesh& textureMesh);
	int runMaterialClassification(TexturedMesh& textureMesh);

};

typedef void* Manager;

extern "C" DLL_TYPE Manager manager();
extern "C" DLL_TYPE int managerInit(Manager manager, std::string parameterPath);
extern "C" DLL_TYPE int runTextureMapping(Manager manager, std::string parameterPath);
extern "C" DLL_TYPE int runMaterialClassification(Manager manager, std::string parameterPath);

}
#endif