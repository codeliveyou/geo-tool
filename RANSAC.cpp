#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <limits>

// Structure for a 2D point.
struct Point {
    double x;
    double y;
};

// Structure for a 2D rigid transformation: rotation (theta) and translation (tx, ty).
struct Transform {
    double theta; // Rotation angle in radians.
    double tx;    // Translation in x.
    double ty;    // Translation in y.
};
	// Member function that returns the inverse transformation.
	// If T transforms a point p to p' (i.e., p' = R * p + t),
	// then T() returns T⁻¹ that transforms p' back to p.
	Transform T() const {
	    Transform inv;
	    // Inverse rotation is simply -theta.
	    inv.theta = -theta;
	    // Compute the inverse translation: -Rᵀ * t.
	    double cosTheta = std::cos(theta);
	    double sinTheta = std::sin(theta);
	    // Rᵀ  = [ cosθ, sinθ ; -sinθ, cosθ ]
	    inv.tx = -(cosTheta * tx + sinTheta * ty);
	    inv.ty = -(-sinTheta * tx + cosTheta * ty);
	    return inv;
	}


// Apply the transformation T to a 2D point pt.
Point applyTransform(const Transform& T, const Point& pt) {
    // Rotation: x' = cos(theta)*x - sin(theta)*y, y' = sin(theta)*x + cos(theta)*y
    double cosTheta = std::cos(T.theta);
    double sinTheta = std::sin(T.theta);
    Point ret;
    ret.x = cosTheta * pt.x - sinTheta * pt.y + T.tx;
    ret.y = sinTheta * pt.x + cosTheta * pt.y + T.ty;
    return ret;
}

// Estimate a transformation from two correspondences (src1->dst1 and src2->dst2).
// Returns true if a valid transformation is computed.
bool estimateTransformFromTwo(const Point& src1, const Point& src2,
                              const Point& dst1, const Point& dst2,
                              Transform& T)
{
    // Compute differences (vectors) in source and destination.
    double dx_src = src2.x - src1.x;
    double dy_src = src2.y - src1.y;
    double dx_dst = dst2.x - dst1.x;
    double dy_dst = dst2.y - dst1.y;

    // Check for degenerate pairs (points too close).
    double norm_src = std::hypot(dx_src, dy_src);
    double norm_dst = std::hypot(dx_dst, dy_dst);
    if (norm_src < 1e-6 || norm_dst < 1e-6) {
        return false;
    }

    // Compute angles of the source and destination vectors.
    double angle_src = std::atan2(dy_src, dx_src);
    double angle_dst = std::atan2(dy_dst, dx_dst);
    // Estimate rotation angle needed.
    T.theta = angle_dst - angle_src;

    // Compute translation so that R(src1) + t = dst1.
    double cosTheta = std::cos(T.theta);
    double sinTheta = std::sin(T.theta);
    double src1_rotated_x = cosTheta * src1.x - sinTheta * src1.y;
    double src1_rotated_y = sinTheta * src1.x + cosTheta * src1.y;
    T.tx = dst1.x - src1_rotated_x;
    T.ty = dst1.y - src1_rotated_y;

    return true;
}

// Run RANSAC to estimate the best transformation given correspondences.
// srcPoints: points from the source (e.g., LiDAR).
// dstPoints: corresponding points on the destination (e.g., floor plan).
// iterations: number of RANSAC iterations.
// threshold: maximum distance error to count an inlier.
Transform runRANSAC(const std::vector<Point>& srcPoints,
                    const std::vector<Point>& dstPoints,
                    int iterations,
                    double threshold)
{
    size_t n = srcPoints.size();
    if(n != dstPoints.size() || n < 2) {
        std::cerr << "Insufficient or mismatched correspondences!\n";
        return {0.0, 0.0, 0.0};
    }

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, n - 1);

    int bestInlierCount = 0;
    Transform bestTransform = {0.0, 0.0, 0.0};

    for (int iter = 0; iter < iterations; ++iter) {
        // Select two distinct random indices.
        int idx1 = dis(gen);
        int idx2 = dis(gen);
        while (idx2 == idx1) {
            idx2 = dis(gen);
        }

        Transform candidate;
        if (!estimateTransformFromTwo(srcPoints[idx1], srcPoints[idx2],
                                      dstPoints[idx1], dstPoints[idx2],
                                      candidate))
        {
            continue; // Degenerate sample, skip.
        }

        // Count the number of inliers for this candidate.
        int inlierCount = 0;
        for (size_t i = 0; i < n; ++i) {
            Point transformed = applyTransform(candidate, srcPoints[i]);
            double error = std::hypot(transformed.x - dstPoints[i].x,
                                      transformed.y - dstPoints[i].y);
            if (error < threshold) {
                ++inlierCount;
            }
        }

        // Update best transformation if candidate is better.
        if (inlierCount > bestInlierCount) {
            bestInlierCount = inlierCount;
            bestTransform = candidate;
        }
    }

    std::cout << "Best inlier count: " << bestInlierCount << " / " << n << "\n";
    return bestTransform;
}

int main() {
    // Example: synthetic dataset of correspondences.
    // Define some LiDAR (source) points.
    std::vector<Point> lidarPoints = {
        {1.0, 2.0},
        {2.0, 3.0},
        {3.0, 4.0},
        {4.0, 5.0},
        {5.0, 6.0}
    };

    // Suppose the true transformation is: rotation by 0.2 radians and translation (tx, ty) = (1.0, -0.5)
    Transform trueTransform = {0.2, 1.0, -0.5};

    // Generate corresponding floor plan points by applying the true transformation.
    std::vector<Point> floorPlanPoints;
    for (const auto& pt : lidarPoints) {
        floorPlanPoints.push_back(applyTransform(trueTransform, pt));
    }

    // You may add extra points to floorPlanPoints if needed,
    // but here we assume the correspondences are known (same size vectors).

    // Run RANSAC to estimate the transformation.
    int iterations = 1000;
    double threshold = 0.2; // Define a suitable threshold for your expected noise.
    Transform estimatedTransform = runRANSAC(lidarPoints, floorPlanPoints, iterations, threshold);

    std::cout << "Estimated Transformation:\n";
    std::cout << "  Rotation (radians): " << estimatedTransform.theta << "\n";
    std::cout << "  Translation: (" << estimatedTransform.tx << ", " << estimatedTransform.ty << ")\n";

    // Optionally, compare with the ground truth.
    std::cout << "\nGround Truth Transformation:\n";
    std::cout << "  Rotation (radians): " << trueTransform.theta << "\n";
    std::cout << "  Translation: (" << trueTransform.tx << ", " << trueTransform.ty << ")\n";

    return 0;
}
