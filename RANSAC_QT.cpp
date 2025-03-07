// main.cpp
//--------------------------------------------------
#include <QCoreApplication>
#include <QDebug>
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
    
    // Member function that returns the inverse (reverse) transformation.
    // If T transforms a point p to p' (p' = R * p + t),
    // then T() returns T⁻¹ that transforms p' back to p.
    Transform T() const {
        Transform inv;
        inv.theta = -theta;  // Inverse rotation
        double cosTheta = std::cos(theta);
        double sinTheta = std::sin(theta);
        // Inverse translation is -Rᵀ * t.
        inv.tx = -(cosTheta * tx + sinTheta * ty);
        inv.ty = -(-sinTheta * tx + cosTheta * ty);
        return inv;
    }
};

// Applies the transformation T to a 2D point pt.
Point applyTransform(const Transform& T, const Point& pt) {
    double cosTheta = std::cos(T.theta);
    double sinTheta = std::sin(T.theta);
    Point ret;
    ret.x = cosTheta * pt.x - sinTheta * pt.y + T.tx;
    ret.y = sinTheta * pt.x + cosTheta * pt.y + T.ty;
    return ret;
}

// Helper function to find the nearest neighbor in a set of points.
// Given a point 'p', it returns the point in 'points' with minimal Euclidean distance.
Point findNearestPoint(const Point& p, const std::vector<Point>& points) {
    double minDist = std::numeric_limits<double>::max();
    Point bestPoint = {0.0, 0.0};
    for (const auto& candidate : points) {
        double dx = candidate.x - p.x;
        double dy = candidate.y - p.y;
        double dist = std::hypot(dx, dy);
        if (dist < minDist) {
            minDist = dist;
            bestPoint = candidate;
        }
    }
    return bestPoint;
}

// Estimate a transformation from two pairs of correspondences.
// For each pair, the correspondence is: LiDAR point -> (its nearest floor plan point)
// Returns true if a valid transformation is computed.
bool estimateTransformFromTwo(const Point& src1, const Point& src2,
                              const Point& dst1, const Point& dst2,
                              Transform& T)
{
    // Compute vectors between the two source points and between the two destination points.
    double dx_src = src2.x - src1.x;
    double dy_src = src2.y - src1.y;
    double dx_dst = dst2.x - dst1.x;
    double dy_dst = dst2.y - dst1.y;
    
    // Check for degenerate pairs (points too close).
    double norm_src = std::hypot(dx_src, dy_src);
    double norm_dst = std::hypot(dx_dst, dy_dst);
    if (norm_src < 1e-6 || norm_dst < 1e-6)
        return false;
    
    // Estimate rotation difference.
    double angle_src = std::atan2(dy_src, dx_src);
    double angle_dst = std::atan2(dy_dst, dx_dst);
    T.theta = angle_dst - angle_src;
    
    // Compute translation so that R(src1)+t = dst1.
    double cosTheta = std::cos(T.theta);
    double sinTheta = std::sin(T.theta);
    double src1_rotated_x = cosTheta * src1.x - sinTheta * src1.y;
    double src1_rotated_y = sinTheta * src1.x + cosTheta * src1.y;
    T.tx = dst1.x - src1_rotated_x;
    T.ty = dst1.y - src1_rotated_y;
    
    return true;
}

// Run RANSAC to estimate the transformation that maps LiDAR (source) points
// to the floor plan (destination). The floor plan points are assumed to be much denser.
// For each candidate pair, we find the corresponding floor plan points using findNearestPoint.
Transform runRANSAC(const std::vector<Point>& lidarPoints,
                    const std::vector<Point>& floorPlanPoints,
                    int iterations,
                    double threshold)
{
    size_t n = lidarPoints.size();
    if(n < 2) {
        std::cerr << "Not enough LiDAR points for RANSAC!\n";
        return {0.0, 0.0, 0.0};
    }
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, n - 1);
    
    int bestInlierCount = 0;
    Transform bestTransform = {0.0, 0.0, 0.0};
    
    for (int iter = 0; iter < iterations; ++iter) {
        // Randomly select two distinct LiDAR points.
        int idx1 = dis(gen);
        int idx2 = dis(gen);
        while (idx2 == idx1)
            idx2 = dis(gen);
        
        Point src1 = lidarPoints[idx1];
        Point src2 = lidarPoints[idx2];
        
        // For these two LiDAR points, find their nearest floor plan points.
        Point dst1 = findNearestPoint(src1, floorPlanPoints);
        Point dst2 = findNearestPoint(src2, floorPlanPoints);
        
        Transform candidate;
        if (!estimateTransformFromTwo(src1, src2, dst1, dst2, candidate))
            continue;
        
        // Count inliers: for each LiDAR point, transform it and compare it to its nearest floor plan point.
        int inlierCount = 0;
        for (const auto &pt : lidarPoints) {
            Point projected = applyTransform(candidate, pt);
            // Find the nearest floor plan point to the projected point.
            Point nearestFP = findNearestPoint(projected, floorPlanPoints);
            double error = std::hypot(projected.x - nearestFP.x, projected.y - nearestFP.y);
            if (error < threshold)
                ++inlierCount;
        }
        
        if (inlierCount > bestInlierCount) {
            bestInlierCount = inlierCount;
            bestTransform = candidate;
        }
    }
    
    std::cout << "Best inlier count: " << bestInlierCount << " out of " << n << " LiDAR points." << std::endl;
    return bestTransform;
}

int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);
    
    // Example synthetic data.
    // LiDAR (source) points (sparser).
    std::vector<Point> lidarPoints = {
        {1.0, 2.0},
        {2.0, 3.0},
        {3.0, 4.0},
        {4.0, 5.0},
        {5.0, 6.0}
    };
    
    // Floor plan (destination) points (denser).
    // For this example, we generate many points along a transformed grid.
    Transform trueTransform = {0.2, 1.0, -0.5};  // The ground truth transformation.
    std::vector<Point> floorPlanPoints;
    // Create a dense grid around the LiDAR points after transformation.
    for (int i = 0; i < 20; ++i) {
        for (int j = 0; j < 20; ++j) {
            // Use the true transform on some base grid and add slight noise.
            Point base = {static_cast<double>(i) * 0.5, static_cast<double>(j) * 0.5};
            Point transformed = applyTransform(trueTransform, base);
            // Optionally add a tiny random noise if desired.
            floorPlanPoints.push_back(transformed);
        }
    }
    
    // Run RANSAC.
    int iterations = 1000;
    double threshold = 0.3; // Choose threshold based on expected noise/accuracy.
    Transform estimatedTransform = runRANSAC(lidarPoints, floorPlanPoints, iterations, threshold);
    
    qDebug() << "Estimated Transformation:";
    qDebug() << "  Rotation (radians):" << estimatedTransform.theta;
    qDebug() << "  Translation:" << "(" << estimatedTransform.tx << "," << estimatedTransform.ty << ")";
    
    // Demonstrate the inverse transform.
    Transform inverseTransform = estimatedTransform.T();
    qDebug() << "\nInverse (Reverse) Transformation:";
    qDebug() << "  Rotation (radians):" << inverseTransform.theta;
    qDebug() << "  Translation:" << "(" << inverseTransform.tx << "," << inverseTransform.ty << ")";
    
    // Optionally, compare with the ground truth.
    qDebug() << "\nGround Truth Transformation:";
    qDebug() << "  Rotation (radians):" << trueTransform.theta;
    qDebug() << "  Translation:" << "(" << trueTransform.tx << "," << trueTransform.ty << ")";
    
    return app.exec();
}
