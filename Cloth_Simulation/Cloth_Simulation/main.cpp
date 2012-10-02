// main.cpp



#include "Vertex.h"
#include "Constraint.h"

#include <vector>
#include <iostream>
#include <fstream>
#include <cmath>
#include <time.h>
#include <math.h>
#include <Eigen/Dense>
#include "bendConstraint.h"

#ifdef OSX
#include <GLUT/glut.h>
#include <OpenGL/glu.h>
#else
#include <GL/glut.h>
#include <GL/glu.h>
#endif

using namespace Eigen;
using namespace std;

float normLength( const Vector3f&, const Vector3f& );
void keyPressed(unsigned char key, int x, int y);
void beginSimulation();
void projectConstraints( vector< vector< Vertex* > >& );
void sphereCollision(const Vector3f& center,const float radius, vector< vector< Vertex* > >& positionGrid );
void cubeCollision(int minX, int minY, int minZ, int maxX, int maxY, int maxZ, vector< vector< Vertex* > >& positionGrid);

class Viewport {
public:
	int w, h; // width and height
};





//****************************************************
// Global Variables
//****************************************************
Viewport	viewport;
vector< Vertex* > vertices;
vector< Constraint* > constraints;
vector< bendConstraint* > bendConstraints;
vector< vector< Vertex* > > clothGrid;
float timestep;
const Vector3f GRAVITY_ACCELERATION = Vector3f(0.0f, -9.80665f, 0.0f);
const float CLOTH_MASS = 1.0f;
const float CLOTH_DROP_HEIGHT = 1.0f;
const int NUM_ITERATIONS = 2;
int clothWidth;
int clothLength;
float spread;
float stiffness = 0.1f;
float ball_radius = 2.0f;

//****************************************************
// reshape viewport if the window is resized
//****************************************************
void reshape(int w, int h)  
{
	// Adjust the viewport if the user resizes the window
	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION); 
	glLoadIdentity();  
	if (h==0)  
		gluPerspective(80,(float)w,1.0,5000.0);
	else
		gluPerspective (80,( float )w /( float )h,1.0,5000.0 );
	glMatrixMode(GL_MODELVIEW);  
	glLoadIdentity(); 
}

//****************************************************
// sets the window up
//****************************************************
void init(void)
{
	glClearColor(0.2f, 0.2f, 0.4f, 0.5f);
	glClearDepth(1.0f);
	
	// Display with depth cues to prevent the sphere or cloth from incorrectly covering eachother
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_LIGHTING);
	
	// Used to light the bottom back side
	glEnable(GL_LIGHT0);
	GLfloat lightPos[4] = {1.0,-0.5,-0.2,0.0};
	glLightfv(GL_LIGHT0,GL_POSITION,(GLfloat *) &lightPos);

	
	// Lights the top left
	glEnable(GL_LIGHT1);
	GLfloat lightAmbient1[4] = {0.0,0.0,0.0,0.0};
	GLfloat lightPos1[4] = {-1.0,1.0,0,0.0};
	GLfloat lightDiffuse1[4] = {0.5,0.5,0.3,0.0};
	glLightfv(GL_LIGHT1,GL_POSITION,(GLfloat *) &lightPos1);
	glLightfv(GL_LIGHT1,GL_AMBIENT,(GLfloat *) &lightAmbient1);
	glLightfv(GL_LIGHT1,GL_DIFFUSE,(GLfloat *) &lightDiffuse1);
	
	
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE,GL_TRUE);
}

void beginSimulation() {
	vector< vector< Vertex* > >positionGrid;
	
	for ( int i = 0; i < clothWidth ; ++i ) {
		vector< Vertex* > temp;
		positionGrid.push_back( temp );
	}
	
	
	//change velocity here
	for ( vector< int >::size_type x = 0; x < clothWidth; ++x ) {
		for ( vector< int >::size_type y = 0; y < clothLength; ++y ) {
			clothGrid[x][y]->velocity = clothGrid[x][y]->velocity + ( timestep * ( GRAVITY_ACCELERATION * clothGrid[x][y]->weight ) );
		}
	}
	
	//damp velocity here
	
	//change positions	
	
	
	for ( vector< int >::size_type x = 0; x < clothWidth; ++x ) {
		Vector3f velocity( 0.0f, 0.0f, 0.0f );
		
		for ( vector< int >::size_type y = 0; y < clothLength; ++y ) {
			Vector3f position= clothGrid[x][y]->position + ( timestep * clothGrid[x][y]->velocity );
			positionGrid[x].push_back( new Vertex( CLOTH_MASS, position, velocity ) );
		}
	}

	//		
	//		for ( vector< int >::size_type x = 0; x < clothGrid.size(); ++x ) {
	//			for ( vector< int >::size_type y = 0; x < clothGrid[x].size(); ++y ) {
	//				generateCollisionConstraints( newClothGrid[x][y]->position, clothGrid[x][y]->position );
	//			}
	//		}
	
	//solver
	for ( int i = 0; i < NUM_ITERATIONS; ++i ) {
		
		projectConstraints( positionGrid );
		Vector3f center(3.5, -3, 3.5);
		cubeCollision(4, -6, -2, 8, -2, 2, positionGrid);
		
	}
	
	
	//update velocity and position
	for ( vector< int >::size_type x = 0; x < clothWidth; ++x ) {
		for ( vector< int >::size_type y = 0; y < clothWidth; ++y ) {
			clothGrid[x][y]->velocity = ( positionGrid[x][y]->position - clothGrid[x][y]->position ) / timestep;
			clothGrid[x][y]->position = positionGrid[x][y]->position;
		}
	}
	
	//cout << clothGrid[0][0]->position << endl;
	
}

void projectConstraints( vector< vector< Vertex* > >& positionGrid ) {
	//satisfy distance constraint
	for ( vector< int >::size_type x = 0; x < constraints.size(); ++x ) {
		Constraint* currentConstraint = constraints[x];
		Vertex* v1 = positionGrid[currentConstraint->v1_x][currentConstraint->v1_y];
		Vertex* v2 = positionGrid[currentConstraint->v2_x][currentConstraint->v2_y];
		if ( !currentConstraint->stretch( v1->position, v2->position ) ) {
			Vector3f deltaP1;
			Vector3f deltaP2;
			
			Vector3f distanceVector = v1->position - v2->position;
			float distanceVectorNorm = normLength( v1->position, v2->position );			
			float distanceConstraintValue  = distanceVectorNorm - currentConstraint->initialLength; // (|p1 - p2| - L )
			
			deltaP1 = ( -v1->weight * distanceConstraintValue / ( v1->weight + v2->weight ) ) * ( distanceVector / distanceVectorNorm );
			deltaP2 = (  v2->weight * distanceConstraintValue / ( v1->weight + v2->weight ) ) * ( distanceVector / distanceVectorNorm );
			
			float stiffnessCoefficient = pow( (1.0f - stiffness), (1.0f / (float) NUM_ITERATIONS) );

			
			
			v1->position += deltaP1 * stiffnessCoefficient;
			v2->position += deltaP2 * stiffnessCoefficient;
			
			//cout << v1->position << endl;
		}
		
	}
	
	for ( vector< int >::size_type x = 0; x < bendConstraints.size(); ++x ) {
		bendConstraint* currentConstraint = bendConstraints[x];
		Vertex* v1 = positionGrid[currentConstraint->v_1x][currentConstraint->v_1y];
		Vertex* v2 = positionGrid[currentConstraint->v_2x][currentConstraint->v_2y];
		Vertex* v3 = positionGrid[currentConstraint->v_3x][currentConstraint->v_3y];
		Vertex* v4 = positionGrid[currentConstraint->v_4x][currentConstraint->v_4y];
		if ( false ){
			Vector3f p1 = v1->position;
			Vector3f p2 = v2->position;
			Vector3f p3 = v3->position;
			Vector3f p4 = v4->position;
			
		Vector3f n1 = p1.cross(p2)/((p1.cross(p2)).norm());
		Vector3f n2 = p1.cross(p4)/((p1.cross(p4)).norm());
		float d = n1.dot(n2);
		float weight = v1->weight; 
		Vector3f q3 = (p1.cross(n2) + (n1.cross(p1))*d)/((p1.cross(p2)).norm());
		Vector3f q4 = (p1.cross(n1) + (n2.cross(p1))*d)/((p1.cross(p4)).norm());
		Vector3f q2 = -(p3.cross(n2) + (n1.cross(p3))*d)/((p1.cross(p2)).norm()) - (p4.cross(n1) + (n2.cross(p4))*d)/((p1.cross(p4)).norm());
		Vector3f q1 = -q2-q3-q4;
		float tempNum = -weight * sqrt(1-(d*d))*(acos(d)-currentConstraint->dihedralAngle);
		float tempDen = weight*(q4.norm()*q4.norm()) + weight*(q2.norm()*q2.norm()) + weight*(q3.norm()*q3.norm());
		Vector3f deltaP1 = tempNum/tempDen*q1;
		tempDen = weight*(q4.norm()*q4.norm()) + weight*(q1.norm()*q1.norm()) + weight*(q3.norm()*q3.norm());
		Vector3f deltaP2 = tempNum/tempDen*q2;
		tempDen = weight*(q4.norm()*q4.norm()) + weight*(q1.norm()*q1.norm()) + weight*(q2.norm()*q2.norm());
		Vector3f deltaP3 = tempNum/tempDen*q3;
		tempDen = weight*(q2.norm()*q2.norm()) + weight*(q1.norm()*q1.norm()) + weight*(q3.norm()*q3.norm());
		Vector3f deltaP4 = tempNum/tempDen*q4;
		float stiffnessCoefficient = pow(1.0f - stiffness, 1.0f/(float)NUM_ITERATIONS);
		v1->position += deltaP1;
		v2->position += deltaP2;
		v3->position += deltaP3;
		v4->position += deltaP4;
	
	}
	 
	
	
	}
	
}

// |v1 - v2|
float normLength( const Vector3f& v1, const Vector3f&v2 ) {
	Vector3f temp = v1 - v2;
	return temp.norm();
}

void sphereCollision(const Vector3f& center,const float radius, vector< vector< Vertex* > >& positionGrid )
{
	for ( vector< int >::size_type x = 0; x < clothWidth; ++x ) {
		for ( vector< int >::size_type y = 0; y < clothLength; ++y ) {
			Vector3f v = positionGrid[x][y]->position-center;
			float length = v.norm();
			if ( length < radius) { // if the particle is inside the ball
				Vertex* p = positionGrid[x][y];
				v.normalize();
				p->position = (v * radius) + center; // project the particle to the surface of the ball
			}
		}
	}
}
void cubeCollision(int minX, int minY, int minZ, int maxX, int maxY, int maxZ, vector< vector< Vertex* > >& positionGrid )
{
	for ( vector< int >::size_type x = 0; x < clothWidth; ++x ) {
		for ( vector< int >::size_type y = 0; y < clothLength; ++y ) {
			Vector3f point = positionGrid[x][y]->position;
			
			if ( point(0)>=minX && point(0) <=maxX && point(1)>=minY && point(1) <= maxY && point(2) >= minZ && point(2) <= maxZ) { // if the particle is inside the cube
				Vertex* p = positionGrid[x][y];
				float check1 = abs(point(0)-minX);
				float check2 = abs(point(0)-maxX);
				float check3 = abs(point(1) - minY);
				float check4 = abs(point(1)-maxY);
				float check5 = abs(point(2) - minZ);
				float check6 = abs(point(2) - maxZ);
				if(check1 < check2 && check1 < check3 && check1 < check4 && check1 < check5 && check1 < check6){
					Vector3f intersect(minX, point(1), point(2));
					p->position = intersect;
				}else if(check2 < check1 && check2 < check3 && check2 < check4 && check2 < check5 && check2 < check6){
					Vector3f intersect(maxX, point(1), point(2));
					p->position = intersect;
				}else if(check3 < check1 && check3 < check2 && check3 < check4 && check3 < check5 && check3 < check6){
					Vector3f intersect(point(0), minY, point(2));
					p->position = intersect;
				}else if(check4 < check1 && check4 < check2 && check4 < check3 && check4 < check5 && check4 < check6){
					Vector3f intersect(point(0), maxY, point(2));
					p->position = intersect;
				}else if(check5 < check1 && check5 < check2 && check5 < check3 && check5 < check4 && check5 < check6){
					Vector3f intersect(point(0), point(1), minZ);
					p->position = intersect;
				}else{
					Vector3f intersect(point(0), point(1), maxZ);
					p->position = intersect;
				}

			}
		}
	}
}




//***************************************************
// function that does the actual drawing
//***************************************************
void display() {
	
	beginSimulation();
	
	// Clear the screen with the blue color set in the init
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	
	// Move the scene in front of the camera then rotate it before doing any drawing
	glPushMatrix();
	glTranslatef(-4,3.5,-14.0f);
	glRotatef(25,0,1,0);
	//draw cloth
	glColor3f(0.0f, 1.0f, 0.0f);
	for ( vector< int >::size_type x = 0; x < clothWidth-1; ++x ) {
		for ( vector< int >::size_type y = 0; y < clothLength-1; ++y ) {
			
			if ( ( x + y ) % 2 == 0 ) {
				glColor3f( 1.0f, 1.0f, 1.0f );
			} else {
				glColor3f( 0.0f, 0.0f, 0.0f );
			}

			/*
			glBegin(GL_TRIANGLES); //   Triangle: L
			glVertex3f( clothGrid[x][y]->position(0), clothGrid[x][y]->position(1),clothGrid[x][y]->position(2) );
			glVertex3f( clothGrid[x][y+1]->position(0), clothGrid[x][y+1]->position(1),clothGrid[x][y+1]->position(2) );
			glVertex3f( clothGrid[x+1][y+1]->position(0), clothGrid[x+1][y+1]->position(1),clothGrid[x+1][y+1]->position(2) );
			glEnd();
			
			glBegin(GL_TRIANGLES); //   Triangle: \-|
			glVertex3f( clothGrid[x][y]->position(0), clothGrid[x][y]->position(1),clothGrid[x][y]->position(2) );
			glVertex3f( clothGrid[x+1][y+1]->position(0), clothGrid[x][y+1]->position(1),clothGrid[x+1][y+1]->position(2) );
			glVertex3f( clothGrid[x+1][y]->position(0), clothGrid[x+1][y]->position(1),clothGrid[x+1][y]->position(2) );
			glEnd();
			*/
			glBegin(GL_POLYGON);
			glVertex3f( clothGrid[x][y]->position(0), clothGrid[x][y]->position(1),clothGrid[x][y]->position(2) );
			glVertex3f( clothGrid[x][y+1]->position(0), clothGrid[x][y+1]->position(1),clothGrid[x][y+1]->position(2) );
			glVertex3f( clothGrid[x+1][y+1]->position(0), clothGrid[x+1][y+1]->position(1),clothGrid[x+1][y+1]->position(2) );
			glVertex3f( clothGrid[x+1][y]->position(0), clothGrid[x+1][y]->position(1),clothGrid[x+1][y]->position(2) );
			glEnd();
		}
	}
	
	// Move the ball then draw
	glPushMatrix();
	glTranslatef(6, -4,0);
	glColor3f(0.4f,0.0f,0.0f);
	//glutSolidSphere(ball_radius-0.1,50,50);
	glutSolidCube(3.7);
	// Clear the transforms and rotations applied earlier
	glPopMatrix();
	glPopMatrix();
	
	// Display the new frame
	glutSwapBuffers();
	glutPostRedisplay();
}


void keyboard( unsigned char key, int x, int y ) 
{
	switch ( key ) {
			
			// On ESC quit
		case 27:    
			exit ( 0 );
			break;  
			
			// On Space drop the cloth or pause its faul
		case 32:
			beginSimulation();
			break;
	}
}

void idleFunc() {
}

/*
 Inputs:
 -Timestep
 -Cloth Length in vertices
 -Cloth Widgth in vertices
 -Spreadout Factor
 
 */
int main(int argc, char *argv[]) {
	
	if ( argc != 5 )
		exit(1);
	
	timestep = atof( argv[1] );
	clothLength = atoi( argv[2] );
	clothWidth = atoi( argv[3] );
	spread  = atof( argv[4] );
	
	

	for ( int i = 0; i < clothWidth ; ++i ) {
		vector< Vertex* > temp;
		clothGrid.push_back( temp );
	}
	
	for ( vector< int >::size_type x = 0; x < clothWidth; ++x ) {
		Vector3f velocity( 0.0f, 0.0f, 0.0f );
		
		for ( vector< int >::size_type y = 0; y < clothLength; ++y ) {
			Vector3f position( x * spread, CLOTH_DROP_HEIGHT, y * spread );
			clothGrid[x].push_back( new Vertex( CLOTH_MASS, position, velocity ) );
		}
	}
	
	//debug forloop
//	for ( vector< int >::size_type x = 0; x < clothWidth; ++x ) {
//		for ( vector< int >::size_type y = 0; y < clothLength; ++y ) {
//			cout << clothGrid[x][y]->position << endl;
//			cout << x <<  " " << y << endl;
//		}
//	}
	
	for ( vector< int >::size_type x = 0; x < clothWidth; ++x ) {
		for ( vector< int >::size_type y = 0; y < clothLength; ++y ) {
			float vertDistance;
			
			if(x==0 && y==0){
				vertDistance = normLength( clothGrid[x][y]->position, clothGrid[x][y+1]->position );
				Constraint* cDown = new Constraint( x, y, x, y+1, stiffness, vertDistance );
				
				vertDistance = normLength( clothGrid[x][y]->position, clothGrid[x+1][y+1]->position );
				Constraint* cDownRight = new Constraint( x, y, x+1, y+1, stiffness, vertDistance );
				
				vertDistance = normLength( clothGrid[x][y]->position, clothGrid[x+1][y]->position );
				Constraint* cRight = new Constraint( x, y, x+1, y, stiffness, vertDistance );
				
				constraints.push_back( cDown );
				constraints.push_back( cDownRight );
				constraints.push_back( cRight );
			}else if( y==0 && x<clothWidth-1 && x>0){
				vertDistance = normLength(clothGrid[x-1][y]->position, clothGrid[x][y]->position);
				Constraint* cLeft = new Constraint(x-1, y, x, y, stiffness, vertDistance);

				vertDistance = normLength(clothGrid[x-1][y+1]->position, clothGrid[x][y]->position);
				Constraint* cDownLeft = new Constraint(x-1, y+1, x, y, stiffness, vertDistance);

				vertDistance = normLength( clothGrid[x][y]->position, clothGrid[x][y+1]->position );
				Constraint* cDown = new Constraint( x, y, x, y+1, stiffness, vertDistance );
				
				vertDistance = normLength( clothGrid[x][y]->position, clothGrid[x+1][y+1]->position );
				Constraint* cDownRight = new Constraint( x, y, x+1, y+1, stiffness, vertDistance );
				
				vertDistance = normLength( clothGrid[x][y]->position, clothGrid[x+1][y]->position );
				Constraint* cRight = new Constraint( x, y, x+1, y, stiffness, vertDistance );

				
				constraints.push_back(cLeft);
				constraints.push_back(cDownLeft);
				constraints.push_back( cDown );
				constraints.push_back( cDownRight );
				constraints.push_back( cRight );
			}else if(x==clothWidth-1 && y == 0){
				vertDistance = normLength(clothGrid[x-1][y]->position, clothGrid[x][y]->position);
				Constraint* cLeft = new Constraint(x-1, y, x, y, stiffness, vertDistance);

				vertDistance = normLength(clothGrid[x-1][y+1]->position, clothGrid[x][y]->position);
				Constraint* cDownLeft = new Constraint(x-1, y+1, x, y, stiffness, vertDistance);

				vertDistance = normLength( clothGrid[x][y]->position, clothGrid[x][y+1]->position );
				Constraint* cDown = new Constraint( x, y, x, y+1, stiffness, vertDistance );

				constraints.push_back(cLeft);
				constraints.push_back(cDownLeft);
				constraints.push_back( cDown );
			}else if(x==0 && y<clothLength-1 && y>0){
				vertDistance = normLength(clothGrid[x][y-1]->position, clothGrid[x][y]->position);
				Constraint* cUp = new Constraint(x, y-1, x, y, stiffness, vertDistance);

				vertDistance = normLength(clothGrid[x+1][y-1]->position, clothGrid[x][y]->position);
				Constraint* cUpRight = new Constraint(x+1, y-1, x, y, stiffness, vertDistance);

				vertDistance = normLength( clothGrid[x][y]->position, clothGrid[x][y+1]->position );
				Constraint* cDown = new Constraint( x, y, x, y+1, stiffness, vertDistance );
				
				vertDistance = normLength( clothGrid[x][y]->position, clothGrid[x+1][y+1]->position );
				Constraint* cDownRight = new Constraint( x, y, x+1, y+1, stiffness, vertDistance );
				
				vertDistance = normLength( clothGrid[x][y]->position, clothGrid[x+1][y]->position );
				Constraint* cRight = new Constraint( x, y, x+1, y, stiffness, vertDistance );
				
				constraints.push_back(cUp);
				constraints.push_back(cUpRight);
				constraints.push_back( cDown );
				constraints.push_back( cDownRight );
				constraints.push_back( cRight );
			}else if(x==0 && y==clothLength-1){
				vertDistance = normLength(clothGrid[x][y-1]->position, clothGrid[x][y]->position);
				Constraint* cUp = new Constraint(x, y-1, x, y, stiffness, vertDistance);

				vertDistance = normLength(clothGrid[x+1][y-1]->position, clothGrid[x][y]->position);
				Constraint* cUpRight = new Constraint(x+1, y-1, x, y, stiffness, vertDistance);

				vertDistance = normLength( clothGrid[x][y]->position, clothGrid[x+1][y]->position );
				Constraint* cRight = new Constraint( x, y, x+1, y, stiffness, vertDistance );

				constraints.push_back(cUp);
				constraints.push_back(cUpRight);
				constraints.push_back(cRight);
			}else if(x>0 && x<clothWidth-1 && y==clothLength-1){
				vertDistance = normLength(clothGrid[x-1][y]->position, clothGrid[x][y]->position);
				Constraint* cLeft = new Constraint(x-1, y, x, y, stiffness, vertDistance);
				
				vertDistance = normLength(clothGrid[x-1][y-1]->position, clothGrid[x][y]->position);
				Constraint* cUpLeft = new Constraint(x-1, y-1, x, y, stiffness, vertDistance);

				vertDistance = normLength(clothGrid[x][y-1]->position, clothGrid[x][y]->position);
				Constraint* cUp = new Constraint(x, y-1, x, y, stiffness, vertDistance);

				vertDistance = normLength(clothGrid[x+1][y-1]->position, clothGrid[x][y]->position);
				Constraint* cUpRight = new Constraint(x+1, y-1, x, y, stiffness, vertDistance);

				vertDistance = normLength( clothGrid[x][y]->position, clothGrid[x+1][y]->position );
				Constraint* cRight = new Constraint( x, y, x+1, y, stiffness, vertDistance );

				constraints.push_back(cLeft);
				constraints.push_back(cUpLeft);
				constraints.push_back(cUp);
				constraints.push_back(cUpRight);
				constraints.push_back(cRight);
			}else if(x==clothWidth-1 && y==clothLength-1){
				vertDistance = normLength(clothGrid[x-1][y]->position, clothGrid[x][y]->position);
				Constraint* cLeft = new Constraint(x-1, y, x, y, stiffness, vertDistance);
				
				vertDistance = normLength(clothGrid[x-1][y-1]->position, clothGrid[x][y]->position);
				Constraint* cUpLeft = new Constraint(x-1, y-1, x, y, stiffness, vertDistance);

				vertDistance = normLength(clothGrid[x][y-1]->position, clothGrid[x][y]->position);
				Constraint* cUp = new Constraint(x, y-1, x, y, stiffness, vertDistance);

				constraints.push_back(cLeft);
				constraints.push_back(cUpLeft);
				constraints.push_back(cUp);
			}else if(x==clothWidth-1 && y > 0 && y<clothLength-1){
				vertDistance = normLength( clothGrid[x][y]->position, clothGrid[x][y+1]->position );
				Constraint* cDown = new Constraint( x, y, x, y+1, stiffness, vertDistance );

				vertDistance = normLength(clothGrid[x-1][y+1]->position, clothGrid[x][y]->position);
				Constraint* cDownLeft = new Constraint(x-1, y+1, x, y, stiffness, vertDistance);
				
				vertDistance = normLength(clothGrid[x-1][y]->position, clothGrid[x][y]->position);
				Constraint* cLeft = new Constraint(x-1, y, x, y, stiffness, vertDistance);

				vertDistance = normLength(clothGrid[x-1][y-1]->position, clothGrid[x][y]->position);
				Constraint* cUpLeft = new Constraint(x-1, y-1, x, y, stiffness, vertDistance);

				vertDistance = normLength(clothGrid[x][y-1]->position, clothGrid[x][y]->position);
				Constraint* cUp = new Constraint(x, y-1, x, y, stiffness, vertDistance);

				constraints.push_back(cDown);
				constraints.push_back(cDownLeft);
				constraints.push_back(cLeft);
				constraints.push_back(cUpLeft);
				constraints.push_back(cUp);

			}else{
				vertDistance = normLength(clothGrid[x-1][y]->position, clothGrid[x][y]->position);
				Constraint* cLeft = new Constraint(x-1, y, x, y, stiffness, vertDistance);
				
				vertDistance = normLength(clothGrid[x-1][y-1]->position, clothGrid[x][y]->position);
				Constraint* cUpLeft = new Constraint(x-1, y-1, x, y, stiffness, vertDistance);

				vertDistance = normLength(clothGrid[x][y-1]->position, clothGrid[x][y]->position);
				Constraint* cUp = new Constraint(x, y-1, x, y, stiffness, vertDistance);

				vertDistance = normLength(clothGrid[x+1][y-1]->position, clothGrid[x][y]->position);
				Constraint* cUpRight = new Constraint(x+1, y-1, x, y, stiffness, vertDistance);

				vertDistance = normLength( clothGrid[x][y]->position, clothGrid[x+1][y]->position );
				Constraint* cRight = new Constraint( x, y, x+1, y, stiffness, vertDistance );

				vertDistance = normLength( clothGrid[x][y]->position, clothGrid[x+1][y+1]->position );
				Constraint* cDownRight = new Constraint( x, y, x+1, y+1, stiffness, vertDistance );

				vertDistance = normLength( clothGrid[x][y]->position, clothGrid[x][y+1]->position );
				Constraint* cDown = new Constraint( x, y, x, y+1, stiffness, vertDistance );

				vertDistance = normLength(clothGrid[x-1][y+1]->position, clothGrid[x][y]->position);
				Constraint* cDownLeft = new Constraint(x-1, y+1, x, y, stiffness, vertDistance);

				constraints.push_back(cLeft);
				constraints.push_back(cUpLeft);
				constraints.push_back(cUp);
				constraints.push_back(cUpRight);
				constraints.push_back(cRight);
				constraints.push_back(cDownRight);
				constraints.push_back(cDown);
				constraints.push_back(cDownLeft);

				}
			}
		}
			

			

	for ( vector< int >::size_type x = 0; x < clothWidth-1; ++x ) {
		for ( vector< int >::size_type y = 0; y < clothLength-1; ++y ){
			Vertex* indexPosition = clothGrid[x][y];
			if(y==clothLength-2 && x==clothWidth-2){
				bendConstraint* c1 = new bendConstraint( x, y, x, y+1, x+1, y+1, x+1, y, stiffness, *indexPosition, *clothGrid[x][y+1], *clothGrid[x+1][y+1], *clothGrid[x+1][y]);
				bendConstraints.push_back(c1);
			}else if (x == clothWidth-2){
				bendConstraint* c1 = new bendConstraint( x, y, x, y+1, x+1, y+1, x+1, y, stiffness, *indexPosition, *clothGrid[x][y+1], *clothGrid[x+1][y+1], *clothGrid[x+1][y]);
				//bendConstraint* c2 = new bendConstraint( x, y+1, x, y, x+1, y+1, x+1, y+2, stiffness, *clothGrid[x][y+1], *indexPosition, *clothGrid[x+1][y+1], *clothGrid[x+1][y+2]);
				bendConstraints.push_back(c1);
				//bendConstraints.push_back(c2);
			}else if(y == clothLength-2){
				bendConstraint* c1 = new bendConstraint( x, y, x, y+1, x+1, y+1, x+1, y, stiffness, *indexPosition, *clothGrid[x][y+1], *clothGrid[x+1][y+1], *clothGrid[x+1][y]);
				//bendConstraint* c2 = new bendConstraint( x+1, y, x, y, x+1, y+1, x+2, y+1, stiffness, *clothGrid[x+1][y], *indexPosition, *clothGrid[x+1][y+1], *clothGrid[x+2][y+1]);
				bendConstraints.push_back(c1);
				//bendConstraints.push_back(c2);
			}else{
				bendConstraint* c1 = new bendConstraint( x, y, x, y+1, x+1, y+1, x+1, y, stiffness, *indexPosition, *clothGrid[x][y+1], *clothGrid[x+1][y+1], *clothGrid[x+1][y]);
				//bendConstraint* c2 = new bendConstraint( x+1, y, x, y, x+1, y+1, x+2, y+1, stiffness, *clothGrid[x+1][y], *indexPosition, *clothGrid[x+1][y+1], *clothGrid[x+2][y+1]);
				//bendConstraint* c3 = new bendConstraint( x, y+1, x, y, x+1, y+1, x+1, y+2, stiffness, *clothGrid[x][y+1], *indexPosition, *clothGrid[x+1][y+1], *clothGrid[x+1][y+2]);
				bendConstraints.push_back(c1);
				//bendConstraints.push_back(c2);
				//bendConstraints.push_back(c3);
				
			}
		}
	}
	
  	// Initialize a window
	glutInit( &argc, argv );
	glutInitDisplayMode( GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH ); 
	glutInitWindowSize(1200, 800 ); 
	glutCreateWindow( "Cloth Simulation" );
	
	// Initialize the GL context with predefined values
	init();
	
	// Link the window to the rest of the code
	glutDisplayFunc(display);  
	glutReshapeFunc(reshape);
	glutKeyboardFunc(keyboard);
	//glutIdleFunc(idleFunc);
	
	// Begin the window's event polling
	glutMainLoop();
	return 0;
}








