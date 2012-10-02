#ifndef bendconstraint_h
#define bendconstraint_h
#include "Vertex.h"

#include <Eigen/Dense>



class bendConstraint {
public:
	float stiffness;
	float dihedralAngle;
	int v_1x;
	int v_1y;
	int v_2x;
	int v_2y;
	int v_3x;
	int v_3y;
	int v_4x;
	int v_4y;
	
	bendConstraint(int, int, int, int, int, int, int, int, float, const Vertex&, const Vertex&, const Vertex&, const Vertex&);
	bool bend( const Eigen::Vector3f&, const Eigen::Vector3f&, const Eigen::Vector3f&, const Eigen::Vector3f& );
};
#endif