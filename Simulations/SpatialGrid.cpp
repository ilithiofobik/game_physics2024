#include "SpatialGrid.h"

SpatialGrid::SpatialGrid()
{
	for (int i = 0; i < BIG_PRIME; i++) {
		std::unordered_set<int> s;
		value[i] = s;
	}
}

void SpatialGrid::addValue(std::tuple<int, int, int> t, int i)
{
	int h = getHash(t);
	(value[h]).insert(i);
}

void SpatialGrid::moveValue(std::tuple<int, int, int> a, std::tuple<int, int, int> b, int i)
{
	int ha = getHash(a);
	int hb = getHash(b);
	(value[ha]).erase(i);
	(value[hb]).insert(i);
}

bool SpatialGrid::isEmpty(int x, int y, int z)
{
	int h = getHash({ x,y,z });
	return value[h].empty();
}

const std::unordered_set<int>& SpatialGrid::get(int x, int y, int z)
{
	int h = getHash({ x,y,z });
	return value[h];
}

int SpatialGrid::getHash(std::tuple<int, int, int> t)
{
	int a = std::get<0>(t) * 73856093;
	int b = std::get<1>(t) * 19349663;
	int c = std::get<2>(t) * 83492791;
	int x = (a ^ b ^ c) % BIG_PRIME;
	return std::abs(x);
}
