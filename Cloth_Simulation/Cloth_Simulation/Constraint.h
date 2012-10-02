/*
 *  Constraint.h
 *  ClothSimulation
 */

#ifndef constraint_h
#define constraint_h
#include "Vertex.h"

#include <Eigen/Dense>

enum constraintType {
	equality,
	inequality
};


class Constraint {		//for a cardinality of two
public:
	constraintType type;
	float stiffness;
	Vertex v1;
	Vertex v2;
	float initialLength;
	int v1_x;
	int v1_y;
	int v2_x;
	int v2_y;
	
	Constraint( const Vertex&, const Vertex&, float, constraintType=equality );
	Constraint( int, int, int, int, float, float, constraintType=equality );
	
	bool stretch();
	bool stretch( const Eigen::Vector3f&, const Eigen::Vector3f& );
};

#endif