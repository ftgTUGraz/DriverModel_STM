#include "NearbyVehicle.h"
#include "Utilities.h"


NearbyVehicle::NearbyVehicle(EgoVehicle ego)
{
	this->turning_indicator = ego.turning_indicator;
	this->acceleration = ego.desired_acceleration;
	this->desired_velocity = ego.desired_velocity;
	this->relative_velocity = 0;
	this->distance = 0;
	this->vehicle_id = ego.vehicle_id;
	this->relative_lane = 0;
	this->relative_position = 0;
	this->selected_as_target = false;
	this->desired_lane_angle = ego.desired_lane_angle;
	this->data_rel_target_lane = ego.rel_target_lane;
}

NearbyVehicle::NearbyVehicle()
{
	this->selected_as_target = false;
	this->turning_indicator = 0;
	this->desired_velocity = VEH_VELOCITY;
}

void NearbyVehicle::setAsTarget(bool val)
{
	this->selected_as_target = val;
}

std::tuple<int, int>  NearbyVehicle::calculateDistanceCol(double speed_ego)
{

	utilities::printDebug(std::ostringstream() << "##################################################\n");
	utilities::printDebug(std::ostringstream() << "Ego Speed: " << speed_ego << "\n");
	utilities::printDebug(std::ostringstream() << "Limit 1: " << speed_ego * T1 << "\n");
	utilities::printDebug(std::ostringstream() << "Limit 2: " << speed_ego * T2 << "\n");
	utilities::printDebug(std::ostringstream() << "Limit 3: " << speed_ego * T3 << "\n");
	utilities::printDebug(std::ostringstream() << "Limit 4: " << speed_ego * T4 << "\n");
	utilities::printDebug(std::ostringstream() << "##################################################\n");


	int row = this->relative_lane;
	if (this->distance >= speed_ego * T1 && this->distance < speed_ego * T2)
	{
		this->color = ORANGE;
		return { row, 0 };
	}
	else if (this->distance >= speed_ego * T2 && this->distance < speed_ego * T3)
	{
		this->color = YELLOW;
		return { row, 1 };
	}
	else if (this->distance >= speed_ego * T3 && this->distance < speed_ego * T4)
	{
		this->color = WHITE;
		return { row, 2 };
	}

	return { NO_LANE, -1 };
}