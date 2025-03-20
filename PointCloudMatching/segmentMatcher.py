import numpy as np
import matplotlib.pyplot as plt
import math

def closest_segment(source_segment, target_segments):
    min_distance = float('inf')
    closest = None
    for target_segment in target_segments:
        distance = segment_distance(source_segment, target_segment)
        if distance < min_distance:
            min_distance = distance
            closest = target_segment
    return closest, min_distance

def segment_distance(seg1, seg2):
    return np.linalg.norm(np.array(seg1[0]) - np.array(seg2[0])) + np.linalg.norm(np.array(seg1[1]) - np.array(seg2[1]))

def icp(source_segments, target_segments, max_iterations=100, tolerance=1e-5):
    source_segments = np.array(source_segments)
    target_segments = np.array(target_segments)
    
    rotation = np.eye(2)
    translation = np.zeros(2)
    
    for iteration in range(max_iterations):
        matched_segments = []
        for source_segment in source_segments:
            closest, distance = closest_segment(source_segment, target_segments)
            matched_segments.append((source_segment, closest))
        
        src_points = np.array([seg[0] for seg, _ in matched_segments] + [seg[1] for seg, _ in matched_segments])
        tgt_points = np.array([seg[0] for _, seg in matched_segments] + [seg[1] for _, seg in matched_segments])
        
        src_centroid = np.mean(src_points, axis=0)
        tgt_centroid = np.mean(tgt_points, axis=0)
        
        src_centered = src_points - src_centroid
        tgt_centered = tgt_points - tgt_centroid
        
        H = np.dot(src_centered.T, tgt_centered)
        
        U, _, Vt = np.linalg.svd(H)
        
        R_opt = np.dot(Vt.T, U.T)
        
        if np.linalg.det(R_opt) < 0:
            Vt[-1, :] *= -1
            R_opt = np.dot(Vt.T, U.T)
        
        t_opt = tgt_centroid - np.dot(R_opt, src_centroid)
        
        transformed_segments = []
        for seg in source_segments:
            transformed_seg = (np.dot(R_opt, seg[0]) + t_opt, np.dot(R_opt, seg[1]) + t_opt)
            transformed_segments.append(transformed_seg)
        
        if np.linalg.norm(t_opt) < tolerance and np.linalg.norm(R_opt - rotation) < tolerance:
            break
        
        rotation = R_opt
        translation = t_opt
        source_segments = np.array(transformed_segments)
    
    return rotation, translation, transformed_segments

def plot_segments(source_segments, target_segments, transformed_segments):
    plt.figure(figsize=(10, 6))
    
    for (x1, y1), (x2, y2) in source_segments:
        plt.plot([x1, x2], [y1, y2], 'b--', label='Original Source' if 'Original Source' not in plt.gca().get_legend_handles_labels()[1] else "")
    
    for (x1, y1), (x2, y2) in transformed_segments:
        plt.plot([x1, x2], [y1, y2], 'g-', label='Transformed Source' if 'Transformed Source' not in plt.gca().get_legend_handles_labels()[1] else "")
    
    for (x1, y1), (x2, y2) in target_segments:
        plt.plot([x1, x2], [y1, y2], 'r-', label='Target' if 'Target' not in plt.gca().get_legend_handles_labels()[1] else "")
    
    plt.title('Segment Alignment')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.grid(True)
    plt.show()

def main():
    source_segments = [((1, 0), (0, 2)), ((0, 2), (1.5, 2))]
    target_segments = [((1, 1), (3, 2)), ((3, 2), (3, 1))]
    
    rotation, translation, transformed_segments = icp(source_segments, target_segments)
    
    print("Optimal Rotation:\n", rotation)
    print("Optimal Translation:\n", translation)
    
    plot_segments(source_segments, target_segments, transformed_segments)

if __name__ == "__main__":
    main()
