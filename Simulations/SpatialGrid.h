#ifndef SPATIALGRID_H
#define SPATIALGRID_H

#include <unordered_map>
#include <unordered_set>

class SpatialGrid {
public:
	// Construtors
	SpatialGrid();

	void addValue(int x, int y, int i);
	void removeValue(int x, int y, int i);
	void ensureExists(int x, int y);
	bool isEmpty(int x, int y);
	const std::unordered_set<int>& get(int x, int y);

private:
	std::unordered_map<int, std::unordered_map<int, std::unordered_set<int>>> value;
};
#endif