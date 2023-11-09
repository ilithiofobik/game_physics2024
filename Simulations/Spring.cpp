#include "Spring.h"

Spring::Spring(int p1, int p2, float s, float iL) {
	point1 = p1;
	point2 = p2;
	stiffness = s;
	initialLength = iL;
}