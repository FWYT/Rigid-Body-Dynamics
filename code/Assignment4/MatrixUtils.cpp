#include "MatrixUtils.h"

Matrix3x3 MatrixUtils::crossProductMatrix(V3D v) {
	Matrix3x3 M;
	M(0, 0) = 0;
	M(0, 1) = -v[2];
	M(0, 2) = v[1];
	M(1, 0) = v[2];
	M(1, 1) = 0;
	M(1, 2) = -v[0];
	M(2, 0) = -v[1];
	M(2, 1) = v[0];
	M(2, 2) = 0;
	return M;
}

void MatrixUtils::orthonormalize(Matrix3x3 &mat) {
	V3D col1 = V3D(mat.col(0));
	V3D col2 = V3D(mat.col(1));
	V3D col3 = V3D(mat.col(2));

	// Normalize first column
	col1.normalize();

	// Orthogonalize second column
	// Subtract parallel component of col2 with col1
	V3D col2parallel = V3D(col2.dot(col1) * col1);
	col2 = col2 - col2parallel;
	col2.normalize();

	// Orthogonalize third column
	// Subtract parallel component of col3 with col1
	V3D col3parallel1 = V3D(col3.dot(col1) * col1);
	col3 = col3 - col3parallel1;
	// Same for col3 with col2
	V3D col3parallel2 = V3D(col3.dot(col2) * col2);
	col3 = col3 - col3parallel2;
	col3.normalize();

	mat.col(0) = col1;
	mat.col(1) = col2;
	mat.col(2) = col3;
}