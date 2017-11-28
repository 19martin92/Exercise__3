#pragma once
#include "SimpleMesh.h"

class ProcrustesAligner {
public:
	Matrix4f estimatePose(const std::vector<Vector3f>& sourcePoints, const std::vector<Vector3f>& targetPoints) {
		ASSERT(sourcePoints.size() == targetPoints.size() && "The number of source and target points should be the same, since every source point is matched with corresponding target point.");

		// We estimate the pose between source and target points using Procrustes algorithm.
		// Our shapes have the same scale, therefore we don't estimate scale. We estimated rotation and translation
		// from source points to target points.

		auto sourceMean = computeMean(sourcePoints);
		auto targetMean = computeMean(targetPoints);
		
		Matrix3f rotation = estimateRotation(sourcePoints, sourceMean, targetPoints, targetMean);
		Vector3f translation = computeTranslation(sourceMean, targetMean, rotation);

		Matrix4f estimatedPose = Matrix4f::Identity();
		estimatedPose.block(0, 0, 3, 3) = rotation;
		estimatedPose.block(0, 3, 3, 1) = translation;

		return estimatedPose;
	}

private:
	Vector3f computeMean(const std::vector<Vector3f>& points) {
		// TODO: Compute the mean of input points.
		Vector3f result = Vector3f::Zero();

		for (int i = 0; i < points.size(); i++)
		{
			result.x() += points[i].x();
			result.y() += points[i].y();
			result.z() += points[i].z();
		}
		result.x() /= points.size();
		result.y() /= points.size();
		result.z() /= points.size();

		return result;
	}

	Matrix3f estimateRotation(const std::vector<Vector3f>& sourcePoints, const Vector3f& sourceMean, const std::vector<Vector3f>& targetPoints, const Vector3f& targetMean) {
		// TODO: Estimate the rotation from source to target points, following the Procrustes algorithm. 
		// To compute the singular value decomposition you can use JacobiSVD() from Eigen.
		
		// zero-center all points
		std::vector<Vector3f> sourcePointsZero = sourcePoints;
		std::vector<Vector3f> targetPointsZero = targetPoints;

		for (int i = 0; i < sourcePoints.size(); i++)
		{
			sourcePointsZero[i] -= sourceMean;
			targetPointsZero[i] -= targetMean;
		}

		// calculate x and xd
		Vector3f x = Vector3f::Zero();
		Vector3f xd = Vector3f::Zero();

		for (int i = 0; i < sourcePointsZero.size(); i++)
		{
			x += sourcePointsZero[i];
			xd += targetPointsZero[i];
		}

		// create A
		// A = xT * xd
		Matrix3f A = xd*x.transpose();	

		// jacobi of matrix A
		JacobiSVD<Matrix3f> svd(A);

		// R = U*VT
		Matrix3f R = svd.matrixU() * svd.matrixV().transpose();

		return R;
	}

	Vector3f computeTranslation(const Vector3f& sourceMean, const Vector3f& targetMean, const Matrix3f& rotation) {
		// TODO: Compute the translation vector from source to target opints.
		Vector3f result = targetMean;

		result -= rotation * sourceMean;

		return result;
	}
};