#pragma once

#include "MathLib/P3D.h"
#include "MathLib/V3D.h"

class MatrixUtils {
public:
	// Compute the cross product matrix of a vector.
	static Matrix3x3 crossProductMatrix(V3D v);
	// Orthonormalize a matrix using Gram-Schmidt.
	static void orthonormalize(Matrix3x3 &mat);
};