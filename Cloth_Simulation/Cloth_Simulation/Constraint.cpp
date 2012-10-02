/*
 *  Constraint.cpp
 *  ClothSimulation
 *
 *  Created by James Turley on 11/28/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "Constraint.h"
#include "Vertex.h"
#include <iostream>

#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

Constraint::Constraint( const Vertex& vert1, const Vertex& vert2, float stiff, constraintType theType )
		   : v1( vert1 ), v2( vert2 ), stiffness( stiff ), type( theType )
{
	Vector3f temp = vert1.position - vert2.position;
	initialLength = temp.norm();
}

Constraint::Constraint( int v1x, int v1y, int v2x, int v2y, float stiff, float length, constraintType theType )
           : v1_x( v1x ), v1_y( v1y ), v2_x( v2x ), v2_y( v2y ), stiffness( stiff ), initialLength( length), type( theType )
{
}



bool Constraint::stretch( ) {

	Vector3f temp = v1.position - v2.position;
	//cout << temp.norm()-initialLength << endl;
	return ( temp.norm() - initialLength == 0.0f ); //|p1 - p2| - l0

}

bool Constraint::stretch( const Vector3f& p1, const Vector3f& p2 ) {
	Vector3f temp = p1 - p2;
	//cout << temp.norm()-initialLength << endl;
	float norm = temp.norm();
	//cout << "difference in length: " << norm - initialLength << endl;
	return ( norm - initialLength == 0.0f );
}

/*
//triangle1 = (p1, p3, p2)
//triangle2 = (p1, p2, p3)
bool Constraint::bend( const Vector3f& p1, const Vector3f& p2, const Vector3f& p3, const Vector3f& p4 ) {
	
	Vector3f temp21 = p2 - p1;
	Vector3f temp31 = p3 - p1;
	Vector3f temp41 = p4 - p1;
	
	Vector3f denomA = temp21.cross( temp31 );
	Vector3f denomB = temp21.cross( temp41 );
	
	Vector3f tempA = temp21.cross( temp31 ) / denomA.norm();
	Vector3f tempB = temp21.cross( temp41 ) / denomB.norm();
	
	if ( type == equality ) {
		return acos( tempA.dot( tempB ) ) - initialAngle;
	}
}
*/