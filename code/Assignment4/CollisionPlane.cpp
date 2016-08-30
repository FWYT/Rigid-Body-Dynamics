#include "CollisionPlane.h"
#include <math.h>
#include "Constants.h"
#include <vector>
#include <iostream>

CollisionPlane::CollisionPlane(P3D p, V3D n) {
	pointOnPlane = p;
	normal = n.normalized();
}

bool CollisionPlane::isColliding(P3D p) {
	V3D displacement = p - pointOnPlane;
	double d = displacement.dot(normal);
	return (d < 0);
}

// Compute the tangential direction at a contact point
// along which the frictional impulse will be applied.
V3D CollisionPlane::frictionTangent(Cuboid &c, P3D &p) { 
	// TODO: implement me
	
	//get relative velocity
	V3D relV = c.velocityOfPoint(p);
	V3D normalComp = (relV.dot(normal))*normal;
	V3D diff = V3D(relV[0] - normalComp[0], relV[1] - normalComp[1], relV[2] - normalComp[2]);

	if (!diff[0] && !diff[1] && !diff[2]) //all zeros
	{
		V3D projForce = V3D(c.netForce[0] - normalComp[0], c.netForce[1] - normalComp[1], c.netForce[2] - normalComp[2]);
		if (projForce[0] || projForce[1] || projForce[2])
		{
			return projForce.normalized();
		}
		else
		{
			return V3D(0, 0, 0);
		}
	}

	diff = diff.normalized();

	return diff;
}

// If the given cuboid is colliding with this plane, then move it
// out of the plane, and apply normal and frictional responses.
void CollisionPlane::applyCollisionResponse(Cuboid &c) {
	std::vector<P3D> vertices;
	c.getTransformedVertices(vertices);
	std::vector<P3D> collided;
	std::vector<P3D>::iterator it;
	it = collided.begin();

	bool collide = false; //save vertices thats true
	for (auto &v : vertices)
	{
		if (isColliding(v))
		{
			it = collided.insert(it, v);
			collide = true;
		}
	}

	//positional correction
	//double maxDiff = 0;
	V3D diff = V3D(0, 0, 0);
	for (auto &v : collided)
	{
		//floor 
		if (v[1] - pointOnPlane[1] < 0)
		{
			if (abs(v[1] - pointOnPlane[1])>diff[1])
			{
				diff[1] = abs(v[1] - pointOnPlane[1]);
			}
		}

		//ceiling
		if (v[1] > 5)
		{
			std::cout << "ceiling" << std::endl;
			if (v[1] - pointOnPlane[1] > diff[1])
			{
				diff[1] = -(v[1] - pointOnPlane[1]);
			}
		}

		//left wall
		if (v[0] < -10)
		{
			if (abs(v[0] - pointOnPlane[0]>diff[0]))
			{
				diff[0] = abs(v[0] - pointOnPlane[0]);
			}
		}

		//right wall
		if (v[0] > 10)
		{
			if (v[0] - pointOnPlane[0] > diff[0])
			{
				diff[0] = -(v[0] - pointOnPlane[0]);
			}
		}

		//front wall
		if (v[2] < -10)
		{
			if (abs(v[2] - pointOnPlane[2])>diff[2])
			{
				diff[2] = abs(v[2] - pointOnPlane[2]);
			}

		}

		//back wall
		if (v[2] > 10)
		{
			if (v[2] - pointOnPlane[2] > diff[2])
			{
				diff[2] = -(v[2] - pointOnPlane[2]);
			}
		}
		/*if (abs(v[1] - pointOnPlane[1]) > maxDiff)
		{
			maxDiff = abs(v[1] - pointOnPlane[1]);
		}*/
	}
	/*for (auto &v : vertices)
	{
		v[1] += maxDiff;
	}*/
	//c.worldTranslation[1] += maxDiff;
	c.worldTranslation += diff;
	

	for (auto &v: collided)
	{
		V3D r = c.worldToLocalCoordinates(v);
		V3D relV = c.velocityOfPoint(v);
		//calculate the impulse magnitude
		//all term involving body 1 is zero since plane is immobile
		double top = ((-1 * (1 + COEFF_RESITUTION))*relV).dot(normal);
		double j = top /
				(pow(c.mass, -1) + (((c.computeWorldCoordsInverseMOI()*
				(r.cross(normal)).cross(r)).dot(normal))));

		if (j < 0)
		{
			continue;
		}

		//linear impulses
		V3D impulse = ((j / c.mass)*normal);
		c.applyLinearImpulse(impulse);

		//angular impulses
		V3D aIm = j*c.computeWorldCoordsInverseMOI()*(r.cross(normal));
		c.applyAngularImpulse(aIm);


		//compute frictional impulse mag
		V3D tangent = frictionTangent(c, v);
		if (tangent[0] || tangent[1] || tangent[2]) //tangent not zero
		{
			double jf;
			double js = COEFF_STATIC_FRICTION*j;
			double jd = COEFF_DYNAMIC_FRICTION*j;
			//double jf = 1;
			//friction impulse
			if ((relV.dot(tangent) == 0) || ((c.mass*relV).dot(tangent) <= js))
			{
				jf = -((c.mass*relV).dot(tangent));
			}
			else
			{
				jf = (-jd);
			}

			//apply linear and angular
			V3D impulseF = ((jf / c.mass)*tangent);
			c.applyLinearImpulse(impulseF);

			V3D aImF = jf*c.computeWorldCoordsInverseMOI()*(r.cross(tangent));
			c.applyAngularImpulse(aImF);
		

		}


	}
	// TODO: implement me
}
