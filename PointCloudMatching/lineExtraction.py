import numpy as np
import matplotlib.pyplot as plt
from sklearn.linear_model import RANSACRegressor
from sklearn.preprocessing import PolynomialFeatures
from sklearn.pipeline import make_pipeline
import time, math

from utility import point, read_scan_points

def extract_segments_ransac(points, min_samples=20, residual_threshold=0.5, max_trials=200):
    lines = []
    remaining_points = points.copy()

    step = 20
    t = time.time()
    while step > 0 and len(remaining_points) > min_samples:
        step -= 1
        ransac = RANSACRegressor(residual_threshold=residual_threshold, max_trials=max_trials)
        ransac.fit(remaining_points[:, 0].reshape(-1, 1), remaining_points[:, 1])
        inlier_mask = ransac.inlier_mask_
        inliers = remaining_points[inlier_mask]
        if len(inliers) > min_samples:
            slope = ransac.estimator_.coef_[0]
            intercept = ransac.estimator_.intercept_
            # y = slope * x + intercept
            x0 = 1 / math.sqrt(1 + slope * slope)
            lines.append([point(0., intercept), point(x0, slope * x0 + intercept)])
        remaining_points = remaining_points[~inlier_mask]
        print("Extracted Lines = ", len(lines), '\n')
    print("Time =", time.time() - t)

    filtered_points = []
    for p in points:
        for seg in lines:
            if abs((seg[1] - seg[0]) ^ (point(p[0], p[1]) - seg[0])) < residual_threshold:
                filtered_points.append(point(p[0], p[1]))
                break

    return filtered_points, lines

def plot_points_and_segments(points, segments):
    plt.figure(figsize=(10, 6))
    plt.scatter([p.x for p in points], [p.y for p in points], color='blue', s=10, label='Points')
    
    for p1, p2 in segments:
        plt.plot([p1.x, p2.x], [p1.y, p2.y], color='red', linewidth=2, label=f'Segment: ({p1.x:.2f}, {p1.y:.2f}) to ({p2.x:.2f}, {p2.y:.2f})')
    
    plt.title('Extracted Line Segments from Point Cloud')
    plt.xlabel('X')
    plt.ylabel('Y')
    # plt.xlim(-10, 15)
    # plt.ylim(-10, 15)
    plt.legend()
    plt.grid(True)
    plt.show()

def main():
    scan_points1 = read_scan_points('./L&M/2x_range/lidar-0000.txt')
    scan_points2 = read_scan_points('./L&M/2x_range/wall-0020.txt')

    np.random.seed(42)
    x = np.array([i[0] for i in scan_points1])
    y = np.array([i[1] for i in scan_points1])

    points = np.column_stack((x, y))

    filtered_points, segments = extract_segments_ransac(points)

    print(len(segments))

    plot_points_and_segments(filtered_points, segments)

if __name__ == "__main__":
    main()
