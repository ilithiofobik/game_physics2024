#ifndef SPATIALGRID_H
#define SPATIALGRID_H

#include <unordered_map>
#include <unordered_set>

class SpatialGrid {
public:
	// Construtors
	SpatialGrid();

	void addValue(std::tuple<int, int, int> t, int i);
	void removeValue(std::tuple<int, int, int> t, int i);
	void ensureExists(int x, int y, int z);
	bool isEmpty(int x, int y, int z);
	const std::unordered_set<int>& get(int x, int y, int z);

private:
	std::unordered_map < int, std::unordered_map<int, std::unordered_map<int, std::unordered_set<int>>>> value;
};
#endif