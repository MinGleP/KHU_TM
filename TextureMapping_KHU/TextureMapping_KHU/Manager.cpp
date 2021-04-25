#include "pch.h" // use stdafx.h in Visual Studio 2017 and earlier
#define DLLEXPORT
#include "Manager.h"
#include<iostream>
using namespace tmManger;

int TextureMaterialManager::initialize(std::string parameterPath)
{
	std::cout << "Parameter Path : " << parameterPath << '\n';

	return 1;
}

extern "C" DLL_TYPE Manager manager()
{
	TextureMaterialManager* tmap = new TextureMaterialManager();
	std::cout << "Manager Declaration\n";
	return (Manager)tmap;
}

extern "C" DLL_TYPE int managerInit(Manager manager, std::string parameterPath)
{
	TextureMaterialManager* m_manager = (TextureMaterialManager*)manager;
	int success = m_manager->initialize(parameterPath);
	
	return 1;
}
