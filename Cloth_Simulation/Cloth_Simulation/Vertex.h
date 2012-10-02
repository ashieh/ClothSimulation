/*
 *  Vertex.h
 *  ClothSimulation
 *
 */

#ifndef vertex_h
#define vertex_h

#include <Eigen/Dense>

class Vertex {
public:
	float mass;
	Eigen::Vector3f position;
	Eigen::Vector3f velocity;
	float weight;
	
	Vertex();
	Vertex( float, const Eigen::Vector3f&, const Eigen::Vector3f& );
	
};

#endif