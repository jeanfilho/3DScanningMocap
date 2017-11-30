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

		Vector3f mean(0, 0, 0);
		for(int i = 0; i < points.size(); i++)
		{
			mean += points[i];
		}
		mean /= points.size();

		return mean;
	}

	Matrix3f estimateRotation(const std::vector<Vector3f>& sourcePoints, const Vector3f& sourceMean, const std::vector<Vector3f>& targetPoints, const Vector3f& targetMean) {
		// TODO: Estimate the rotation from source to target points, following the Procrustes algorithm. 
		// To compute the singular value decomposition you can use JacobiSVD() from Eigen.

		MatrixXf Xs(sourcePoints.size(), 3);
		for (int i = 0; i < sourcePoints.size(); i++)
		{
			Xs(i, 0) = sourcePoints[i].x() - sourceMean.x();
			Xs(i, 1) = sourcePoints[i].y() - sourceMean.y();
			Xs(i, 2) = sourcePoints[i].z() - sourceMean.z();
		}

		MatrixXf Xt(targetPoints.size(), 3);
		for (int i = 0; i < targetPoints.size(); i++)
		{
			Xt(i, 0) = targetPoints[i].x() - targetMean.x();
			Xt(i, 1) = targetPoints[i].y() - targetMean.y();
			Xt(i, 2) = targetPoints[i].z() - targetMean.z();
		}

		JacobiSVD<Matrix3f> svd(Xt.transpose()*Xs, ComputeFullU | ComputeFullV);
		return  svd.matrixU() * svd.matrixV().transpose();
	}

	Vector3f computeTranslation(const Vector3f& sourceMean, const Vector3f& targetMean, const Matrix3f& rotation) {
		// TODO: Compute the translation vector from source to target opints.
		auto translation = targetMean - rotation * sourceMean;
		
		return translation;
	}
};