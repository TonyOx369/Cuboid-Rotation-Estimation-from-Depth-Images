import numpy as np
import open3d as o3d
from rosbags.highlevel import AnyReader
from pathlib import Path
from scipy.spatial import ConvexHull
from sklearn.decomposition import PCA
import cv2


def depth_to_point_cloud(depth_image, fx, fy, cx, cy):
    """
    Converts a depth image to a point cloud and returns the mapping
    from 3D point indices back to their original 2D pixel coordinates.
    """
    height, width = depth_image.shape
    v, u = np.indices((height, width)) # v=rows, u=columns

    Z = depth_image
    valid_mask = Z > 0

    # Get the (row, col) coordinates of all valid pixels
    # This is the mapping we need
    original_pixel_coords = np.vstack((v[valid_mask], u[valid_mask])).T

    # Project to 3D space
    X = (u[valid_mask] - cx) * Z[valid_mask] / fx
    Y = (v[valid_mask] - cy) * Z[valid_mask] / fy
    Z_3d = Z[valid_mask]

    points = np.vstack((X, Y, Z_3d)).T
    
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    
    # Return both the point cloud and the pixel coordinate mapping
    return pcd, original_pixel_coords


def visualize_shaded_depth_image(depth_array_meters, inlier_pixel_coords):
    """
    Displays the depth image with the detected plane region shaded.
    
    Args:
        depth_array_meters (np.array): The n x m array of distances in meters.
        inlier_pixel_coords (np.array): An array of (row, col) coordinates for the inliers.
    """
    # 1. Create a color-mapped version of the depth image, same as before
    normalized_image = cv2.normalize(depth_array_meters, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    display_image = cv2.applyColorMap(normalized_image, cv2.COLORMAP_JET)

    # 2. Create a semi-transparent white overlay for the shaded region
    overlay = np.zeros_like(display_image, dtype=np.uint8)
    
    # Set the pixels corresponding to the plane to white in the overlay
    # Note: NumPy expects indices in (row, column) format, which is what we have.
    # We need to transpose the coordinates for advanced indexing.
    rows, cols = inlier_pixel_coords[:, 0], inlier_pixel_coords[:, 1]
    overlay[rows, cols] = (255, 255, 255) # White

    # 3. Blend the original color image with the overlay
    # This creates the semi-transparent shading effect
    alpha = 0.4 # Transparency factor
    shaded_image = cv2.addWeighted(overlay, alpha, display_image, 1 - alpha, 0)

    # 4. Display the final image
    print("    -> Displaying shaded 2D depth image. Press any key to continue.")
    cv2.imshow("Shaded Depth Image", shaded_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


# --- All other functions (read_bag_file, find_largest_plane, etc.) remain the same ---
def read_bag_file(bag_path):
    with AnyReader([Path(bag_path)]) as reader:
        connections = [x for x in reader.connections if x.topic == '/depth']
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            if msg.encoding == '16UC1':
                depth_image = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
                depth_image_meters = depth_image.astype(np.float32) / 1000.0
            elif msg.encoding == '32FC1':
                depth_image_meters = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
            else:
                continue
            yield depth_image_meters

def find_largest_plane(pcd, distance_threshold=0.01):
    plane_model, inliers = pcd.segment_plane(distance_threshold=distance_threshold, ransac_n=3, num_iterations=1000)
    inlier_cloud = pcd.select_by_index(inliers)
    return plane_model, inlier_cloud, inliers

def get_face_properties(plane_model, inlier_cloud):
    normal_vector = np.array(plane_model[:3])
    if normal_vector[2] > 0:
        normal_vector = -normal_vector
    camera_normal = np.array([0, 0, -1])
    cos_angle = np.dot(normal_vector, camera_normal) / (np.linalg.norm(normal_vector) * np.linalg.norm(camera_normal))
    angle_rad = np.arccos(np.clip(cos_angle, -1.0, 1.0))
    angle_deg = np.degrees(angle_rad)
    points_3d = np.asarray(inlier_cloud.points)
    if len(points_3d) < 3:
        return angle_deg, 0.0
    pca = PCA(n_components=2)
    points_2d = pca.fit_transform(points_3d)
    hull = ConvexHull(points_2d)
    area = hull.volume
    return angle_deg, area

def find_rotation_axis(normals_list):
    if len(normals_list) < 2:
        return None
    pca = PCA(n_components=3)
    pca.fit(normals_list)
    return pca.components_[-1]

def main():
    # --- Configuration ---
    BAG_FILE_PATH = r'/home/sid/rotation/depth'
    VISUALIZE_SHADED_IMAGE = True # Set to True to see the shaded 2D view
    
    FX, FY = 525.0, 525.0
    CX, CY = 319.5, 239.5

    results_table, normal_vectors, image_number = [], [], 0

    print("Processing ROS bag file...")
    for depth_image in read_bag_file(BAG_FILE_PATH):
        image_number += 1
        print(f"-> Processing image {image_number}...")

        # Get the point cloud AND the pixel mapping
        pcd, original_pixel_coords = depth_to_point_cloud(depth_image, FX, FY, CX, CY)
        if not pcd.has_points():
            continue

        pcd_downsampled = pcd.voxel_down_sample(voxel_size=0.02)
        
        # We need the indices relative to the downsampled cloud, so we must find them again
        # This is a bit more complex, so a simpler way for visualization is to find the plane in the original pcd
        plane_model, inlier_cloud, inlier_indices = find_largest_plane(pcd, distance_threshold=0.02)
        
        if len(inlier_cloud.points) < 100:
            continue
        
        if VISUALIZE_SHADED_IMAGE:
            # Use the inlier_indices to get the corresponding original pixel coordinates
            inlier_pixel_coords = original_pixel_coords[inlier_indices]
            visualize_shaded_depth_image(depth_image, inlier_pixel_coords)

        angle, area = get_face_properties(plane_model, inlier_cloud)
        results_table.append({'image': image_number, 'angle_deg': angle, 'area_m2': area})
        normal_vectors.append(plane_model[:3])

    print("\nProcessing complete.")
    rotation_axis = find_rotation_axis(np.array(normal_vectors))

    print("\n--- Results Table ---")
    print(f"{'Image #':<10} | {'Normal Angle (deg)':<20} | {'Visible Area (m^2)':<20}")
    print("-" * 55)
    with open("results_table.txt", "w") as f:
        f.write(f"{'Image #':<10}, {'Normal Angle (deg)':<20}, {'Visible Area (m^2)':<20}\n")
        for res in results_table:
            print(f"{res['image']:<10} | {res['angle_deg']:<20.2f} | {res['area_m2']:<20.4f}")
            f.write(f"{res['image']:<10}, {res['angle_deg']:.2f}, {res['area_m2']:.4f}\n")

    print("\n--- Axis of Rotation ---")
    print(rotation_axis)
    np.savetxt("rotation_axis.txt", rotation_axis, header="Axis of Rotation Vector (X, Y, Z)")

if __name__ == '__main__':
    main()