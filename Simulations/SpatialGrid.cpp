#include "SpatialGrid.h"

SpatialGrid::SpatialGrid()
{
}

void SpatialGrid::addValue(int x, int y, int i)
{
	ensureExists(x, y);
	value.at(x).at(y).insert(i);
}

// does not check if exists!
void SpatialGrid::removeValue(int x, int y, int i)
{
	value.at(x).at(y).erase(i);
}

void SpatialGrid::ensureExists(int x, int y)
{
	// if x not in grid, add it 
	if (value.find(x) == value.end()) {
		std::unordered_map<int, std::unordered_set<int>> newMap = {};
		value.insert(make_pair(x, newMap));
	}

	// if y not in x-row, add it 
	if (value.at(x).find(y) == value.at(x).end()) {
		std::unordered_set<int> newSet = {};
		value.at(x).insert(make_pair(y, newSet));
	}
}

bool SpatialGrid::isEmpty(int x, int y)
{
	if (value.find(x) == value.end()) {
		return true;
	}

	// if y not in x-row, add it 
	if (value.at(x).find(y) == value.at(x).end()) {
		return true;
	}

	return value.at(x).at(y).empty();
}

// does not check if exists!
const std::unordered_set<int>& SpatialGrid::get(int x, int y)
{
	return value.at(x).at(y);
}
