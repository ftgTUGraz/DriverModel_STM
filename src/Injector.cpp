#include "Injector.h"

/*
Here all Injector variables should be initialized
*/
Injector::Injector() :InjectorAbstract(EGO_ID)
{
	this->t2 = 0;
	this->oldPosX = 0.0;
	this->oldPosY = 0.0;
	this->newPosX = 0.0;
	this->newPosY = 0.0;
}


void Injector::capture()
{
	this->t2 = 0;
	this->oldPosX = 0.0;
	this->oldPosY = 0.0;
	this->newPosX = 0.0;
	this->newPosY = 0.0;
	left.clear();
	right.clear();

	if (this->getCurrentSimTime() > 15)
	{
		auto n_vehicles = this->getVehiclesDownstream();
		int cnt = 0;
		for (auto& veh : n_vehicles)
		{
			if (veh->relative_lane != 0)
			{
				veh->setAsTarget();
			}

		}
		lane_change_executed = false;
		this->lc_id = -1;
		startAction(4, 8);
	}
}

void Injector::action(NearbyVehicle* veh, const std::vector<NearbyVehicle*> actionNveh)
{
	veh->color = YELLOW;
	double slope;
	t2 = t2 + this->getCurrentTimeStep(); // accumulate by timestep
	newPosX = lat_ctrl.fctPosX(t2, Tm, meanVelocity); // update current position X
	newPosY = lat_ctrl.fctPosY(t2, Tm, h); // update current position Y
	slope = lat_ctrl.LaneChangeAngle(t2, Tm, newPosX, newPosY, oldPosX, oldPosY); // convert to lane change angle related to middle lane

	
	int lce_selected = this->calculateDistance(veh->distance, veh->relative_velocity, 0);

	if (lce_selected == 1)
	{
		if (!lane_change_executed)
		{
			lane_change_executed = true;
			this->lc_id = veh->vehicle_id;
		}
		if (veh->relative_lane == 1 && lane_change_executed && veh->vehicle_id == this->lc_id)
		{
			veh->desired_lane_angle = -slope;
			veh->active_lane_change = -1;
			veh->data_rel_target_lane = -1;
		}
		else if (veh->relative_lane == -1 && lane_change_executed && veh->vehicle_id == this->lc_id)
		{
			veh->desired_lane_angle = slope;
			veh->active_lane_change = 1;
			veh->data_rel_target_lane = 1;
		}


		oldPosX = newPosX; // current position X will be resotred as the old position X 
		oldPosY = newPosY; // current position Y will be resotred as the old position Y
	}
}


/*
Here user methods can be specified. Also take a look into capture.h. 

User is also allowed to write own classes which can specify functionality
and instance of it can be used within the capture.
*/

int  Injector::calculateDistance(double distance, double ego_v, int min_index)
{

	if (distance >= 5 && distance <= 100)
	{
		return 1;
	}
	return -1;
}

int Injector::getMinLCE()
{
	int min_index = this->lce[0];
	for (int i = 0; i < 3; i++)
	{
		if (lce[i] < min_index)
		{
			min_index = i;
		}
	}

	return min_index;
}

void Injector::action_end()
{
	if (lane_change_executed)
	{
		this->lce[this->getMinLCE()]++;
	}
}