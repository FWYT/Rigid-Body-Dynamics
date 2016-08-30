#ifdef _WIN32
#include <include/glew.h>
#else
#include <GL/glew.h>
#endif

#define GLFW_INCLUDE_GLU
#include <GLFW/glfw3.h>

#include "ParticleSystem.h"
#include "GUILib/OBJReader.h"
#include "Utils/Logger.h"
#include "Constants.h"
#include <math.h>
#include <time.h>

const double BOX_WIDTH = 10;

V3D randomVector() {
    double x = 2 * ((double)rand() / RAND_MAX) - 1;
    double y = 2 * ((double)rand() / RAND_MAX) - 1;
    double z = 2 * ((double)rand() / RAND_MAX) - 1;
    V3D position(x, y, z);
    return position;
}

ParticleSystem::ParticleSystem(GLShaderMaterial* material) {

	srand(time(NULL));

	for (int i = 0; i < 10; i++) {
		double width = 2 * ((double)rand() / RAND_MAX) + 0.5;
		double height = 2 * ((double)rand() / RAND_MAX) + 0.5;
		double depth = 2 * ((double)rand() / RAND_MAX) + 0.5;
		double mass = ((double)rand() / RAND_MAX) + 2;
		double distFromOrigin = 8 * ((double)rand() / RAND_MAX) + 2;

		double r = ((double)rand() / RAND_MAX);
		double g = ((double)rand() / RAND_MAX);
		double b = ((double)rand() / RAND_MAX);

		V3D position = randomVector();
		position.normalize();
		position[1] = abs(position[1]);
		position *= distFromOrigin;

		Matrix3x3 rotation = Matrix3x3::Identity();
		MatrixUtils::orthonormalize(rotation);

		Cuboid* cuboid = new Cuboid(width, height, depth, mass, P3D(position), rotation);
		cuboid->setMaterial(material);
		cuboid->setColor(r, g, b, 1);
		cubes.push_back(cuboid);
	}

	applyingRandomForces = false;

	CollisionPlane floor(P3D(0, 0, 0), V3D(0, 1, 0));
	CollisionPlane left(P3D(-BOX_WIDTH, 0, 0), V3D(1, 0, 0));
	CollisionPlane right(P3D(BOX_WIDTH, 0, 0), V3D(-1, 0, 0));
	CollisionPlane front(P3D(0, 0, -BOX_WIDTH), V3D(0, 0, 1));
	CollisionPlane back(P3D(0, 0, BOX_WIDTH), V3D(0, 0, -1));
	CollisionPlane ceiling(P3D(0, 5, 0), V3D(0, -1, 0));

	planes.push_back(floor);
	planes.push_back(left);
	planes.push_back(right);
	planes.push_back(front);
	planes.push_back(back);
	planes.push_back(ceiling);

}

ParticleSystem::~ParticleSystem() {
	for (Cuboid* c : cubes) {
		delete c;
	}
}

bool ParticleSystem::drawParticles = true;
bool ParticleSystem::enableGravity = true;

// Gravitational constant.
const V3D GRAVITY = V3D(0, -9.8, 0);

// Applies external forces to particles in the system.
// This is currently limited to just gravity.
void ParticleSystem::applyForces(double delta) {
	for (int i = 0; i < cubes.size(); i++) {
		Cuboid* c = cubes[i];
		if (enableGravity) {
			c->applyForce(c->centerOfMass(), GRAVITY * c->mass);
		}
		if (applyingRandomForces) {
			P3D loc = c->centerOfMass() + randomOffsets[i];
			c->applyForce(loc, randomForces[i]);
		}
	}
}

/*  Apply a collision response impulse to cubes c1 and c2 that are in collision.
	collisionPoint is the point of impact, and collisionNormal is the normal.
	The normal is assumed to point away from c1 and towards c2.
*/
void ParticleSystem::applyCollisionImpulse(Cuboid &c1, Cuboid &c2, P3D &collisionPoint, V3D &collisionNormal) {
	// TODO: implement me

	//relative vel 
	V3D v_r = c2.linearVelocity - c1.linearVelocity;

	//positions
	V3D r1 = c1.worldToLocalCoordinates(collisionPoint);
	V3D r2 = c2.worldToLocalCoordinates(collisionPoint);

	V3D term1 = (c1.computeWorldCoordsInverseMOI()*(r1.cross(collisionNormal))).cross(r1);
	V3D term2 = (c2.computeWorldCoordsInverseMOI()*(r2.cross(collisionNormal))).cross(r2);

	//calculate impulse magnitude
	double J = (((-1 * (1 + COEFF_RESITUTION))*v_r).dot(collisionNormal)) /
		(pow(c1.mass, -1) + pow(c2.mass, -1) + (term1 + term2).dot(collisionNormal));

	//linear impulses
	V3D l1 = (J / c1.mass)*collisionNormal;
	c1.applyLinearImpulse(l1*-1);

	V3D l2 = (J / c2.mass)*collisionNormal;
	c2.applyLinearImpulse(l2);

	//angular impulses
	V3D a1 = J*c1.computeWorldCoordsInverseMOI()*(r1.cross(collisionNormal));
	c1.applyAngularImpulse(a1*-1);

	V3D a2 = J*c2.computeWorldCoordsInverseMOI()*(r2.cross(collisionNormal));
	c2.applyAngularImpulse(a2);
			   
}

void ParticleSystem::handleCubeCollisions() {
	for (int i = 0; i < cubes.size(); i++) {
		for (int j = i+1; j < cubes.size(); j++) {
			P3D collisionPoint;
			V3D collisionNormal;
			double distance = 0;
			if (cubes[i]->checkCollision(*cubes[j], collisionPoint, collisionNormal, distance)) {
				V3D correction = collisionNormal * distance;
				collisionPoint += correction;
				cubes[i]->worldTranslation += correction / 2;
				cubes[j]->worldTranslation -= correction / 2;
				applyCollisionImpulse(*cubes[j], *cubes[i], collisionPoint, collisionNormal);
			}
			
			else if (cubes[j]->checkCollision(*cubes[i], collisionPoint, collisionNormal, distance)) {
				V3D correction = collisionNormal * distance;
				collisionPoint += correction;
				cubes[j]->worldTranslation += correction / 2;
				cubes[i]->worldTranslation -= correction / 2;
				applyCollisionImpulse(*cubes[i], *cubes[j], collisionPoint, collisionNormal);
			}
			
		}
	}
}

void ParticleSystem::startVacuumForces() {

	randomForces.clear();
	randomOffsets.clear();

	V3D barycenter;

	for (auto &c : cubes) {
		barycenter += c->worldTranslation;
	}

	barycenter /= cubes.size();

	for (auto &c : cubes) {
		V3D offset = randomVector();
		offset.normalize();
		offset[1] = abs(offset[1]);
		offset *= 0.1;
		randomOffsets.push_back(offset);

		V3D force = (barycenter - c->worldTranslation);
		force.normalize();
		force *= 10 * c->mass;
		force[1] = 10 * c->mass;
		randomForces.push_back(force);
	}

	applyingRandomForces = true;
}

void ParticleSystem::stopVacuumForces() {
	applyingRandomForces = false;
}

// Integrate one time step.
void ParticleSystem::integrate_PBF(double delta) {
	applyForces(delta);
	for (auto &c : cubes) {
		for (auto &p : planes) {
			p.applyCollisionResponse(*c);
		}
	}

	handleCubeCollisions();

	for (auto &c : cubes) {
		c->update(delta);
	}
}

void ParticleSystem::drawParticleSystem() {
	for (auto &c : cubes) {
		c->draw();
	}
}
