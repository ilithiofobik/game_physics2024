#ifndef SPATIALGRID_H
#define SPATIALGRID_H

#include <unordered_set>
#include <tuple>
#define BIG_PRIME 2003

class SpatialGrid {
public:
	// Construtors
	SpatialGrid();

	void addValue(std::tuple<int, int, int> t, int i);
	void removeValue(std::tuple<int, int, int> t, int i);
	bool isEmpty(int x, int y, int z);
	const std::unordered_set<int>& get(int x, int y, int z);

private:
	int getHash(std::tuple<int, int, int> t);

private:
	std::unordered_set<int> value[BIG_PRIME];
	//std::unordered_map < int, std::unordered_map<int, std::unordered_map<int, std::unordered_set<int>>>> value;
};
#endif