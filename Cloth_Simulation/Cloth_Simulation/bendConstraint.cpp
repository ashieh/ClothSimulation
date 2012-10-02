#include "bendConstraint.h"
#include "Vertex.h"
#include <Eigen/Dense>
using namespace Eigen;


bendConstraint::bendConstraint( int vert1x, int vert1y, int vert2x, int vert2y, int vert3x, int vert3y, int vert4x, int vert4y, float stiff, const Vertex& vert1, const Vertex& vert2, const Vertex& vert3, const Vertex& vert4 )
		   : v_1x( vert1x ), v_1y( vert1y ), v_2x(vert2x), v_2y(vert2y), v_3x(vert3x), v_3y(vert3y), v_4x(vert4x), v_4y(vert4y), stiffness( stiff )
{
	Vector3f na = ((vert3.position-vert1.position).cross(vert2.position-vert1.position))/((vert3.position-vert1.position).cross(vert2.position-vert1.position).norm());
	Vector3f nb = ((vert3.position-vert1.position).cross(vert4.position-vert1.position))/((vert3.position-vert1.position).cross(vert4.position-vert1.position).norm());
	dihedralAngle = acos(na.dot(nb)); 
}


bool bendConstraint::bend(const Vector3f& p1, const Vector3f& p2, const Vector3f& p3, const Vector3f& p4 ) {
	
	float newDihedralAngle = 0;
	Vector3f na = ((p3-p1).cross(p2-p1))/((p3-p1).cross(p2-p1)).norm();
	Vector3f nb = ((p3-p1).cross(p4-p1))/((p3-p1).cross(p4-p1)).norm();
	newDihedralAngle = acos(na.dot(nb));
	
	return (newDihedralAngle-dihedralAngle == 0.0f);
}
	
		
		