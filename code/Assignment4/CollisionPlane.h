#pragma once

#include "MathLib/P3D.h"
#include "MathLib/V3D.h"
#include "Cuboid.h"

class CollisionPlane {
private:
	P3D pointOnPlane;
	V3D normal;
public:
	CollisionPlane(P3D p, V3D n);
	bool isColliding(P3D p);
	V3D frictionTangent(Cuboid &c, P3D &p);
	void applyCollisionResponse(Cuboid &c);
};
