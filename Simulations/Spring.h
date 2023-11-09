#ifndef SPRING_h
#define SPRING_h

class Spring {
public:
	// Construtors
	Spring(int p1, int p2, float s, float iL);

	int point1;
	int point2;
	float stiffness;
	float initialLength;
};
#endif