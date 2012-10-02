/*
 *  Vertex.cpp
 *  ClothSimulation
 *
 */

#include "Vertex.h"
#include <Eigen/Dense>

using namespace Eigen;

Vertex::Vertex( )
        :mass( 1.0f ), position( Vector3f( 0.0f, 0.0f, 0.0f) ), velocity( Vector3f( 0.0f, 0.0f, 0.0f ) ), weight( 1.0f )
{}
Vertex::Vertex( float m, const Vector3f& p, const Vector3f& v )
		: mass( m ), position( p ), velocity( v ), weight( 1/m )
{}



