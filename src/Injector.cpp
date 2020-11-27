#include "Injector.h"

/*
Here all Injector variables should be initialized
*/
Injector::Injector() :InjectorAbstract(EGO_ID)
{

}


void Injector::capture()
{
	
	if (this->getCurrentSimTime() > 15)
	{
		auto n_vehicles = this->getVehiclesDownstream();

		for (auto& veh : n_vehicles)
		{
			veh->setAsTarget();
		}
		startAction(8, 8);
	}
}

void Injector::action(NearbyVehicle* veh, const std::vector<NearbyVehicle*> actionNveh)
{
	veh->color = YELLOW;
	veh->acceleration = -1;
}


/*
Here user methods can be specified. Also take a look into capture.h. 

User is also allowed to write own classes which can specify functionality
and instance of it can be used within the capture.
*/