#include "SpatialGrid.h"

SpatialGrid::SpatialGrid()
{
}

void SpatialGrid::addValue(std::tuple<int, int, int> t, int i)
{
	int x, y, z;
	std::tie(x, y, z) = t;
	ensureExists(x, y, z);
	value.at(x).at(y).at(z).insert(i);
}

// does not check if exists!
void SpatialGrid::removeValue(std::tuple<int, int, int> t, int i)
{
	int x, y, z;
	std::tie(x, y, z) = t;
	value.at(x).at(y).at(z).erase(i);
}

void SpatialGrid::ensureExists(int x, int y, int z)
{
	// if x not in grid, add it 
	if (value.find(x) == value.end()) {
		std::unordered_map<int, std::unordered_map<int, std::unordered_set<int>>> newMap = {};
		value.insert(make_pair(x, newMap));
	}

	// if y not in x-row, add it 
	if (value.at(x).find(y) == value.at(x).end()) {
		std::unordered_map<int, std::unordered_set<int>> newMap = {};
		value.at(x).insert(make_pair(y, newMap));
	}

	// if z not in y-col, add it 
	if (value.at(x).at(y).find(z) == value.at(x).at(y).end()) {
		std::unordered_set<int> newSet = {};
		value.at(x).at(y).insert(make_pair(z, newSet));
	}
}

bool SpatialGrid::isEmpty(int x, int y, int z)
{
	if (value.find(x) == value.end()) {
		return true;
	}

	if (value.at(x).find(y) == value.at(x).end()) {
		return true;
	}

	if (value.at(x).at(y).find(z) == value.at(x).at(y).end()) {
		return true;
	}

	return value.at(x).at(y).at(z).empty();
}

// does not check if exists!
const std::unordered_set<int>& SpatialGrid::get(int x, int y, int z)
{
	return value.at(x).at(y).at(z);
}
