#ifdef _WIN32
#include <include/glew.h>
#else
#include <GL/glew.h>
#endif

#define GLFW_INCLUDE_GLU
#include <GLFW/glfw3.h>

#include "Cuboid.h"
#include "Constants.h"

using namespace std;

Cuboid::Cuboid(double w, double h, double d, double m, P3D worldCenter, Matrix3x3 rotation) {
	worldTranslation = V3D(worldCenter);
	worldRotation = rotation;

	width = w;
	height = h;
	depth = d;
	mass = m;

	netForce.zero();
	netTorque.zero();
	linearVelocity = V3D(0, 0, 0);
	angularVelocity = V3D(0, 0, 0.);

	computeBodyCoordsMOI();
	generateVertices();

	setColor(1, 1, 1, 1);
}

void Cuboid::setColor(double r, double g, double b, double a) {
	color_r = r;
	color_g = g;
	color_b = b;
	color_a = a;
}

// Compute the moment of inertia tensor in body coordinates.
// You should store the moment of inertia itself in the field
// bodyCoordsMOI, and store its inverse in bodyCoordsInverseMOI.
void Cuboid::computeBodyCoordsMOI() {
	// TODO: implement me
	bodyCoordsMOI = Matrix3x3::Identity();
	bodyCoordsMOI(0, 0) = (1 / 12.0)*mass*(pow(height, 2) + pow(depth, 2));
	bodyCoordsMOI(1, 1) = (1 / 12.0)*mass*(pow(width, 2) + pow(depth, 2));
	bodyCoordsMOI(2, 2) = (1 / 12.0)*mass*(pow(width, 2) + pow(height, 2));

	// Keep this here -- storing it separately will save time later.
	bodyCoordsInverseMOI = bodyCoordsMOI.inverse();
}

// Compute the moment of inertia tensor in world coordinates.
Matrix3x3 Cuboid::computeWorldCoordsMOI() {
	// TODO: implement me
	return worldRotation*bodyCoordsMOI*(worldRotation.transpose());
}

// Compute the inverse moment of inertia tensor in world coordinates.
// If you use bodyCoordsInverseMOI, you shouldn't have to invert any matrix here.
Matrix3x3 Cuboid::computeWorldCoordsInverseMOI() {
	// TODO: implement me
	return worldRotation*bodyCoordsInverseMOI*(worldRotation.transpose());
}

// Apply a force to this cuboid, at a given location.
// Note that forceLocation will be given in world coordinates.
// You should convert it to body coordinates before doing the torque calculations.
// You should add the linear force to the accumulator netForce,
// and the torque to the accumulator netTorque.
void Cuboid::applyForce(P3D forceLocation, V3D force) {
	// TODO: implement me
	netForce += force;
	V3D localForce = worldToLocalCoordinates(forceLocation);
	netTorque += localForce.cross(force);
}

// Integrate this cuboid's position and rotation forward one step in time,
// using the accumulated forces and torques netForce and netTorque.
void Cuboid::update(double delta) {
	// TODO: implement me
	
	//velocity and position
	V3D dV = (1 / mass)*netForce;
	V3D Vchange = delta*dV;
	linearVelocity = linearVelocity + Vchange;

	V3D posChange = delta * linearVelocity;
	worldTranslation += posChange;


	//angular and rotation
	Matrix3x3 I = computeWorldCoordsMOI();
	V3D Iw = V3D((I(0, 0)*angularVelocity[0]) + (I(0, 1)*angularVelocity[1]) + (I(0, 2)*angularVelocity[2]),
		(I(1, 0)*angularVelocity[0]) + (I(1, 1)*angularVelocity[1]) + (I(1, 2)*angularVelocity[2]),
		(I(2, 0)*angularVelocity[0]) + (I(2, 1)*angularVelocity[1]) + (I(2, 2)*angularVelocity[2]));

	V3D dAngular = computeWorldCoordsInverseMOI()*(netTorque - (angularVelocity.cross(Iw)));
	V3D change = delta*dAngular;
	V3D newAngular = angularVelocity + change;
	angularVelocity = newAngular;

	angularVelocity *= 0.8;
	
	Matrix3x3 skewCross = MatrixUtils::crossProductMatrix(angularVelocity);
	
	worldRotation = (computeWorldCoordsMOI() + (delta*skewCross))*worldRotation;
	MatrixUtils::orthonormalize(worldRotation);
	// After done, clear force and torque accumulators for next time step
	netTorque.zero();
	netForce.zero();
}

void Cuboid::generateVertices() {
	P3D frontUpLeft(-width / 2, height / 2, -depth / 2);
	P3D frontDownLeft(-width / 2, -height / 2, -depth / 2);
	P3D frontDownRight(width / 2, -height / 2, -depth / 2);
	P3D frontUpRight(width / 2, height / 2, -depth / 2);
	P3D backUpLeft(-width / 2, height / 2, depth / 2);
	P3D backDownLeft(-width / 2, -height / 2, depth / 2);
	P3D backDownRight(width / 2, -height / 2, depth / 2);
	P3D backUpRight(width / 2, height / 2, depth / 2);

	P3D points[] = { frontUpLeft, frontDownLeft, frontDownRight, frontUpRight, backUpLeft, backDownLeft, backDownRight, backUpRight };

	for (P3D &p : points) {
		boundingR = std::max(boundingR, p.norm());
	}

	edges[0] = CuboidEdge(frontUpLeft, frontDownLeft);
	edges[1] = CuboidEdge(frontDownLeft, frontDownRight);
	edges[2] = CuboidEdge(frontDownRight, frontUpRight);
	edges[3] = CuboidEdge(frontUpRight, frontUpLeft);
	edges[4] = CuboidEdge(frontUpLeft, backUpLeft);
	edges[5] = CuboidEdge(frontUpRight, backUpRight);
	edges[6] = CuboidEdge(frontDownLeft, backDownLeft);
	edges[7] = CuboidEdge(frontDownRight, backDownRight);
	edges[8] = CuboidEdge(backDownLeft, backUpLeft);
	edges[9] = CuboidEdge(backUpLeft, backUpRight);
	edges[10] = CuboidEdge(backUpRight, backDownRight);
	edges[11] = CuboidEdge(backDownRight, backDownLeft);

	faces[0] = CuboidFace(points[0], points[1], points[2], points[3]);
	faces[1] = CuboidFace(points[3], points[2], points[6], points[7]);
	faces[2] = CuboidFace(points[4], points[5], points[1], points[0]);
	faces[3] = CuboidFace(points[4], points[0], points[3], points[7]);
	faces[4] = CuboidFace(points[1], points[5], points[6], points[2]);
	faces[5] = CuboidFace(points[7], points[6], points[5], points[4]);

	for (int i = 0; i < 8; i++) {
		vertices[i] = points[i];
	}

	std::vector<P3D> frontVec, backVec;
	for (int i = 0; i < 4; i++) {
		frontVec.push_back(points[i]);
	}
	for (int i = 4; i < 8; i++) {
		backVec.push_back(points[i]);
	}

	mesh.addPrism(frontVec, backVec);
	mesh.computeNormals();
}

void Cuboid::draw() {
	double rotationMatrix4[] = {
		worldRotation(0, 0), worldRotation(1, 0), worldRotation(2, 0), 0,
		worldRotation(0, 1), worldRotation(1, 1), worldRotation(2, 1), 0,
		worldRotation(0, 2), worldRotation(1, 2), worldRotation(2, 2), 0,
		0, 0, 0, 1 };
	
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glTranslated(worldTranslation[0], worldTranslation[1], worldTranslation[2]);
	glMultMatrixd(rotationMatrix4);

	mesh.material.setFloatParam("diffuse_color", color_r, color_g, color_b, color_a);
	mesh.drawMesh();

	glPopMatrix();
}

void Cuboid::setMaterial(GLShaderMaterial *material) {
	mesh.setMaterial(*material);
}

void Cuboid::getTransformedVertices(std::vector<P3D> &output) {
	output.clear();
	for (int i = 0; i < 8; i++) {
		V3D transformed = V3D(worldRotation * vertices[i]) + worldTranslation;
		output.push_back(P3D(transformed));
	}
}

V3D Cuboid::velocityOfPoint(P3D point) {
	V3D linearPart = linearVelocity;
	V3D r = V3D(point) - worldTranslation;
	if (angularVelocity.norm() == 0) {
		return linearPart;
	}
	V3D angularPart = angularVelocity.cross(r);
	return linearPart + angularPart;
}

V3D Cuboid::currentNetForce() {
	return netForce;
}

P3D Cuboid::centerOfMass() {
	return P3D(worldTranslation);
}

void Cuboid::applyLinearImpulse(V3D impulse) {
	linearVelocity += impulse;
}

void Cuboid::applyAngularImpulse(V3D impulse) {
	angularVelocity += impulse;
}

V3D Cuboid::worldToLocalCoordinates(P3D worldPos) {
	return worldPos - P3D(worldTranslation);
}

void printVector(char* s, V3D v) {
	Logger::consolePrint("%s = <%f, %f, %f>", s, v[0], v[1], v[2]);
}

double Cuboid::boundingRadius() {
	return boundingR;
}

const double PADDING = 0.01;

bool Cuboid::checkCollision(Cuboid &other, P3D &collisionPoint, V3D &collisionNormal, double &distance) {
	
	double distanceBound = boundingRadius() + other.boundingRadius();
	double centerOfMassDist = (centerOfMass() - other.centerOfMass()).norm();
	if (centerOfMassDist > distanceBound) return false;

	Matrix3x3 otherRT = other.worldRotation.transpose();
	Matrix3x3 toOtherSpace = Matrix3x3(otherRT * worldRotation);
	
	double xBound = other.width / 2 + PADDING;
	double yBound = other.height / 2 + PADDING;
	double zBound = other.depth / 2 + PADDING;

	for (V3D p : vertices) {
		V3D pOtherSpace = (worldTranslation - other.worldTranslation) + V3D(toOtherSpace * p);
		double x = pOtherSpace[0];
		double y = pOtherSpace[1];
		double z = pOtherSpace[2];

		if (-xBound < x && x < xBound &&
			-yBound < y && y < yBound &&
			-zBound < z && z < zBound) {

			V3D transformedInWorld;
			transformedInWorld = V3D(worldRotation * p) + worldTranslation;
			collisionPoint = transformedInWorld;

			collisionNormal = worldTranslation - other.worldTranslation;
			collisionNormal.normalize();

			V3D rotatedNormal(otherRT * collisionNormal);

			double tX, tY, tZ;
			if (rotatedNormal[0] > 0) tX = (xBound - x) / abs(rotatedNormal[0]);
			else if (rotatedNormal[0] < 0) tX = (x + xBound) / abs(rotatedNormal[0]);
			else tX = INFINITY;
			if (rotatedNormal[1] > 0) tY = (yBound - y) / abs(rotatedNormal[1]);
			else if (rotatedNormal[1] < 0) tY = (y + yBound) / abs(rotatedNormal[1]);
			else tY = INFINITY;
			if (rotatedNormal[2] > 0) tZ = (zBound - z) / abs(rotatedNormal[2]);
			else if (rotatedNormal[2] < 0) tZ = (z + zBound) / abs(rotatedNormal[2]);
			else tZ = INFINITY;

			distance = min(tX, min(tY, tZ));
			
			return true;
		}
	}
	return false;
}