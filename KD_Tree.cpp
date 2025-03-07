#include <iostream>
#include <vector>
#include <algorithm>
#include <limits>
#include <cmath>

struct Point {
    double x, y;
};

struct KDNode {
    Point point;
    KDNode* left;
    KDNode* right;

    KDNode(const Point& pt) : point(pt), left(nullptr), right(nullptr) {}
};

// Recursive function to build the KD-Tree
KDNode* buildKDTree(std::vector<Point>::iterator begin,
                    std::vector<Point>::iterator end,
                    int depth = 0) {
    if (begin >= end)
        return nullptr;

    int axis = depth % 2; // 0 for x, 1 for y
    auto comparator = [axis](const Point &a, const Point &b) {
        return (axis == 0) ? (a.x < b.x) : (a.y < b.y);
    };

    auto mid = begin + (end - begin) / 2;
    std::nth_element(begin, mid, end, comparator);
    KDNode* node = new KDNode(*mid);
    node->left = buildKDTree(begin, mid, depth + 1);
    node->right = buildKDTree(mid + 1, end, depth + 1);
    return node;
}

// Helper function to calculate squared Euclidean distance
double squaredDistance(const Point& a, const Point& b) {
    return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
}

// Recursive function for nearest neighbor search in the KD-Tree
void nearestNeighbor(KDNode* root, const Point& target, int depth,
                     Point& best, double& bestDist) {
    if (!root) return;
    
    double d = squaredDistance(root->point, target);
    if (d < bestDist) {
        bestDist = d;
        best = root->point;
    }

    int axis = depth % 2;
    double diff = (axis == 0) ? (target.x - root->point.x) : (target.y - root->point.y);

    KDNode* first = (diff < 0) ? root->left : root->right;
    KDNode* second = (diff < 0) ? root->right : root->left;

    nearestNeighbor(first, target, depth + 1, best, bestDist);

    // Check if we need to search the other subtree
    if (diff * diff < bestDist)
        nearestNeighbor(second, target, depth + 1, best, bestDist);
}

int main() {
    // Sample points
    std::vector<Point> points = {
        {2, 3}, {5, 4}, {9, 6},
        {4, 7}, {8, 1}, {7, 2}
    };

    // Build the KD-Tree from points
    KDNode* root = buildKDTree(points.begin(), points.end());

    // Define a query point for nearest neighbor search
    Point query = {9, 2};
    Point best = {0, 0};
    double bestDist = std::numeric_limits<double>::max();
    
    nearestNeighbor(root, query, 0, best, bestDist);
    
    std::cout << "Nearest neighbor to (" << query.x << ", " << query.y << ") is: ("
              << best.x << ", " << best.y << ")\n";

    // IMPORTANT: In a production system, be sure to free memory by traversing the tree.
    // Here, for brevity, we skip deletion.
    
    return 0;
}