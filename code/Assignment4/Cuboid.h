#pragma once

#include "MathLib/P3D.h"
#include "MathLib/V3D.h"
#include "MatrixUtils.h"
#include "GUILib/GLMesh.h"
#include <random>
#include <vector>

struct CuboidEdge {
	P3D p1, p2;

	CuboidEdge() : p1(0, 0, 0), p2(0, 0, 0) {}

	CuboidEdge(P3D x, P3D y) {
		p1 = x;
		p2 = y;
	}
};

struct CuboidFace {
	P3D center;
	V3D normal;

	CuboidFace() : center(0, 0, 0), normal(0, 0, 0) {}

	CuboidFace(P3D c, V3D n) {
		center = c;
		normal = n;
	}

	CuboidFace(P3D p1, P3D p2, P3D p3, P3D p4) {
		V3D v1 = p3 - p2;
		V3D v2 = p1 - p2;
		normal = v2.cross(v1);
		normal.normalize();
		center = (p1 + p2 + p3 + p4) / 4;
	}
};

class Cuboid {
private:
	// Physical parameters
	double width;
	double height;
	double depth;

	double boundingR;

	// Local coordinates moment of inertia
	Matrix3x3 bodyCoordsMOI;
	Matrix3x3 bodyCoordsInverseMOI;
	void computeBodyCoordsMOI();

	void generateVertices();

	double color_r, color_g, color_b, color_a;
	V3D vertices[8];
	CuboidEdge edges[12];
	CuboidFace faces[6];
	GLMesh mesh;

public:
	// Accumulators for each time step
	V3D netForce;
	V3D netTorque;
	// Transformations to world coordinates
	V3D worldTranslation;
	Matrix3x3 worldRotation;
	// Current velocities
	V3D linearVelocity;
	V3D angularVelocity;

	double mass;

	Cuboid(double w, double h, double d, double m, P3D worldCenter, Matrix3x3 worldRotation);
	void applyForce(P3D worldPosition, V3D force);
	Matrix3x3 computeWorldCoordsMOI();
	Matrix3x3 computeWorldCoordsInverseMOI();
	void update(double delta);
	void draw();
	void setColor(double r, double g, double b, double a);
	void setMaterial(GLShaderMaterial *material);

	// Get the corners of this cuboid, transformed into world space coordinates
	void getTransformedVertices(std::vector<P3D> &output);

	double boundingRadius();
	
	// Compute the velocity of a given point, considered as a part of this body
	V3D velocityOfPoint(P3D point);
	V3D currentNetForce();
	P3D centerOfMass();
	// Apply impulses to the body
	void applyLinearImpulse(V3D impulse);
	void applyAngularImpulse(V3D impulse);
	// Transform a position given in world coordinates into body (local) coordinates
	V3D worldToLocalCoordinates(P3D worldPos);

	bool checkCollision(Cuboid &other, P3D &collisionPoint, V3D &collisionNormal, double &distance);
};