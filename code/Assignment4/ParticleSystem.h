#pragma once

#include <vector>
#include <list>
#include "GUILib/GLMesh.h"
#include "CollisionPlane.h"
#include "Cuboid.h"

using namespace std;

struct ParticleInit {
	P3D position;
	P3D velocity;
	double mass;
};

using namespace Eigen;

class ParticleSystem {
private:
	// Vectors to pass to OpenGL for drawing.
	// Each time step, the relevant data are copied into these lists.
	int count;
	vector<Cuboid*> cubes;
	vector<CollisionPlane> planes;
	vector<V3D> randomForces;
	vector<V3D> randomOffsets;
	
public:
	ParticleSystem(GLShaderMaterial* material);
	~ParticleSystem();
	bool applyingRandomForces;

	void applyCollisionImpulse(Cuboid &c1, Cuboid &c2, P3D &collisionPoint, V3D &collisionNormal);

	void applyForces(double delta);
	void handleCubeCollisions();
	void integrate_PBF(double delta);

	void startVacuumForces();
	void stopVacuumForces();

	// Functions for display and interactivity
	void drawParticleSystem();

	// Whether or not we should draw springs and particles as lines and dots respectively.
	static bool drawParticles;
	static bool enableGravity;
};
