#include "Manager.h"

int main()
{
	tmManger::Manager manager = tmManger::manager();

	tmManger::managerInit(manager, "PARAMETER PATH");

	return 0;
}