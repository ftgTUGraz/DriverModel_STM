#include "Injector.h"

/*
Here all Injector variables should be initialized
*/
Injector::Injector() :InjectorAbstract(EGO_ID)
{
	this->distanceMatrixRows.resize(NUM_OF_LANES);
	for (auto& it : this->distanceMatrixRows)
	{
		it.first = NO_LANE;
		it.second.resize(NUM_OF_COLS, nullptr);
	}
	this->distanceMatrix.resize(NUM_OF_LANES);
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
	//veh->acceleration = this->accelerationModel(veh);
	veh->acceleration = this->accBreakingModel(veh);
}


/*
Here user methods can be specified. Also take a look into capture.h. 

User is also allowed to write own classes which can specify functionality
and instance of it can be used within the capture.
*/

double Injector::accelerationModel(NearbyVehicle* veh)
{
	double init_speed = 0;
	auto s = initial_speeds.find(veh->vehicle_id);
	if (s != initial_speeds.end())
	{
		init_speed = s->second;
	}
	else
	{
		init_speed = getCurrentVehicleVelocity();
		initial_speeds.insert(std::pair<long int, double>(veh->vehicle_id, init_speed));
	}

	veh->color = YELLOW;
	double t = this->getCurrentSimTime() - this->getActionStartTime();

	return (breakingModel.accelerationProfiles(t, init_speed));
}


double Injector::accBreakingModel(NearbyVehicle* veh)
{
	double init_accel = 0;
	auto s = initial_accel.find(veh->vehicle_id);
	if (s != initial_accel.end())
	{
		init_accel = s->second;
	}
	else
	{
		init_accel = veh->acceleration;
		initial_accel.insert(std::pair<long int, double>(veh->vehicle_id, init_accel));
	}

	veh->color = YELLOW;
	double t = this->getCurrentSimTime() - this->getActionStartTime();
	return breakingModel.ACCDccelerationProfiles(t, init_accel);
}




std::vector<std::vector<NearbyVehicle*>> Injector::getDistanceMatrix()
{
	int row = 0, col = 0;
	for (auto& it : this->distanceMatrixRows)
	{
		it.first = NO_LANE;
		it.second.clear();
		it.second.resize(NUM_OF_COLS, nullptr);
	}
	// Iterate through all vehicles in front
	for (auto& vehFront : this->getVehiclesDownstream())
	{
		// get vehFront position within the matrix (relative to ego)
		std::tie(row, col) = this->calculateDistanceCol(vehFront->distance, vehFront->relative_lane, ego.current_velocity);

		//no lane marks that vehicle is not within any predifined column, therefore skiped
		if (row != NO_LANE)
		{
			bool found = false;
			// iterate through all rows and find if any is already marked as current lane (first of pair marks to which lane row corresponds)
			for (auto& lanes : this->distanceMatrixRows)
			{
				if (lanes.first == row)
				{
					if (lanes.second[col] != nullptr)
					{
						if (lanes.second[col]->distance > vehFront->distance)
						{
							lanes.second[col] = vehFront;
						}
					}
					else
					{
						lanes.second[col] = vehFront;
					}
					found = true;
					break;
				}
			}
			// if no row correspond to this lane, first which is not populated (marked with NO_LANE) is taken and marked
			if (!found)
			{
				for (auto& lanes : this->distanceMatrixRows)
				{
					if (lanes.first == NO_LANE)
					{
						lanes.first = row;
						lanes.second[col] = vehFront;
						break;
					}
				}
			}
		}
	}

	// after rows are filled with lane content, sort them in descending order since most left lane has highest index
	std::sort(distanceMatrixRows.begin(), distanceMatrixRows.end(), [](const std::pair<int, std::vector<NearbyVehicle*>>& lhs, std::pair<int, std::vector<NearbyVehicle*>>& rhs)
		{
			return lhs.first > rhs.first;
		});

	// extract lanes into the simple 2D array
	// positions within matrix which are empty are marked with nullptr, therefore nullptr check required before an access
	// possible optimization by using vector reference (avoiding unnecessary copy)
	for (int i = 0; i < NUM_OF_LANES; i++)
	{
		distanceMatrix[i] = distanceMatrixRows[i].second;
	}
	return this->distanceMatrix;
}

std::vector<std::vector<bool>>  Injector::getEventMatrix(std::vector<std::vector<NearbyVehicle*>>& distanceMatrix)
{
	std::vector<std::vector<bool>> eventMatrix(NUM_OF_LANES, std::vector<bool>(NUM_OF_COLS, false));
	for (size_t lane = 0; lane < NUM_OF_LANES; lane++)
	{
		for (size_t col = 0; col < NUM_OF_COLS; col++)
		{
			auto vehicle = distanceMatrix[lane][col];
			if (vehicle != nullptr)
			{
				eventMatrix[lane][col] = true;
			}
		}
	}

	return eventMatrix;
}

bool Injector::triggerEvent(std::vector<std::vector<bool>>& eventMatrix)
{
	int sum_event = 0;
	for (auto& row : eventMatrix)
	{
		for (auto& col : row)
		{
			sum_event += col;
		}
	}

	if (sum_event == 0)
	{
		utilities::printDebug(std::ostringstream() << "EVENT EMPTY\n");
		return false;
	}

	std::vector<std::pair<int, int>> differences = calculateDifferences(eventMatrix);
	int min = differences[0].second;
	for (size_t i = 0; i < differences.size(); i++)
	{
		if (comparisonCounter[differences[i].first] < REPETITION_THRESHOLD && differences[i].second == min)
		{
			comparisonCounter[differences[i].first]++;
			//TRIGGER EVENT
			utilities::printDebug(std::ostringstream() << "TRIGGER EVENT\n");
			utilities::printDebug(std::ostringstream() << "Index: " << differences[i].first << "\n");
			utilities::printDebug(std::ostringstream() << "Diff: " << differences[i].second << "\n");
			//lastTrigger = getCurrentSimTime();
			//DataStorage::instance().writeTimeStamp(lastTrigger, differences[i].first);
			//eventTriggered = true;
			//startStoringData(true);
			//actionNveh = frontNveh;
			return true;
		}

		if (differences[i].second > min)
		{
			utilities::printDebug(std::ostringstream() << "NO EVENT TO TRIGGER\n");
			//NO EVENT TRIGGERED
			break;
		}
	}
	return false;
}


std::vector<std::pair<int, int>> Injector::calculateDifferences(std::vector<std::vector<bool>>& eventMatrix)
{
	std::vector<std::pair<int, int>> differences(COMBINATIONS, std::make_pair(0, 0));

	for (int index = 0; index < COMBINATIONS; index++)
	{
		differences[index].first = index;
	}

	for (size_t row = 0; row < COMBINATIONS; row++)
	{
		differences[row].second = rowDifference(eventMatrix, comparisonMatrix[row]);
	}

	std::sort(differences.begin(), differences.end(), [](const std::pair<int, int>& lhs, const std::pair<int, int>& rhs)
		{
			return lhs.second < rhs.second;
		});


	return differences;
}

int Injector::rowDifference(std::vector<std::vector<bool>>& eventMatrix, std::vector<int>& comparisonMatrixRow)
{
	int diff = 0;

	for (size_t row = 0; row < eventMatrix.size(); row++)
	{
		for (size_t col = 0; col < eventMatrix[row].size(); col++)
		{
			int e_val = eventMatrix[row][col];

			if (comparisonMatrixRow[(row * eventMatrix[row].size()) + col] == XX)
			{
				diff += e_val;
			}
		}
	}

	return diff;
}

std::tuple<int, int>  Injector::calculateDistanceCol(double distance, long row, double speed_ego)
{
	if (distance >= speed_ego * T1 && distance < speed_ego * T2)
	{
		return { row, 0 };
	}
	else if (distance >= speed_ego * T2 && distance < speed_ego * T3)
	{
		return { row, 1 };
	}
	else if (distance >= speed_ego * T3 && distance < speed_ego * T4)
	{
		return { row, 2 };
	}

	return { NO_LANE, -1 };
}