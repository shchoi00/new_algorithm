import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

def get_rainbow_color(frequency, max_frequency):
    # Normalize frequency to a value between 0 and 1
    normalized_freq = frequency / max_frequency
    # Use a colormap that goes from red (high frequency) to violet (low frequency)
    color = plt.get_cmap('rainbow')(normalized_freq)
    return color[:3]  # Exclude the alpha channel

def visualize_shared_xy_pcd(input_file, voxel_size=0.1, output_file='filtered_pcd.pcd'):
    # Load the point cloud
    pcd = o3d.io.read_point_cloud(input_file)
    points = np.asarray(pcd.points)

    min_z = np.min(points[:, 2])
    max_z = np.max(points[:, 2])
    height = max_z - min_z
    print(max_z)
    print(min_z)

    # Calculate the threshold as 30% of the height
    # threshold = 0.16 * height * 10
    
    

    # Compute voxel indices for each point
    voxel_indices = np.floor(points / voxel_size).astype(int)
    print(voxel_indices)
    print(max(voxel_indices[:, 2]))
    print(min(voxel_indices[:, 2]))
    threshold = (max(voxel_indices[:, 2]) - min(voxel_indices[:, 2])) / 5 
    print(height)
    print(threshold)
    # Group points by their (x, y) voxel indices and count occurrences
    voxel_to_points = {}
    xy_frequency = {}
    z_already_counted = {}  # New dictionary to track Z-coordinates for each voxel
    
    for voxel_index, point in zip(voxel_indices, points):
        xy_key = tuple(voxel_index[:2])
        z_index = voxel_index[2]

        if xy_key not in voxel_to_points:
            voxel_to_points[xy_key] = []
            xy_frequency[xy_key] = 0
        

        # Check if this Z index has already been counted for this XY key
        if not any(z_index == existing_point_z_index for existing_point, existing_point_z_index in voxel_to_points[xy_key]):
            
            xy_frequency[xy_key] += 1

        # Add the point and its Z index to the list
        voxel_to_points[xy_key].append((point, z_index))
 
    # Separate points and colors for the new point cloud
    max_frequency = max(xy_frequency.values())
    colored_points = []
    # print(max(xy_frequency.values()))
    # print((xy_frequency.values()))
    for xy_key, pts in voxel_to_points.items():
        if xy_frequency[xy_key] >= threshold:
            color = get_rainbow_color(xy_frequency[xy_key], max_frequency)
            for pt, _ in pts:
                # print(pt)
                colored_points.append((*pt, *color))

    if not colored_points:
        print("No points meet the shoulder threshold.")
        return

    colored_points = np.array(colored_points)
    points_filtered = colored_points[:, :3]
    colors_filtered = colored_points[:, 3:6]

    # Create a point cloud for visualization
    filtered_pcd = o3d.geometry.PointCloud()
    filtered_pcd.points = o3d.utility.Vector3dVector(points_filtered)
    filtered_pcd.colors = o3d.utility.Vector3dVector(colors_filtered)

    # Save the filtered point cloud
    o3d.io.write_point_cloud(output_file, filtered_pcd)
    print(f"Filtered point cloud saved to {output_file}")

    # Visualize the point cloud
    o3d.visualization.draw_geometries([filtered_pcd])

def DBScan(input, output, eps):
    # Load your point cloud
    # input = "./filtered/filtered_AI_7F_05.pcd"
    pcd = o3d.io.read_point_cloud(input)

    # Perform DBSCAN clustering
    labels = np.array(pcd.cluster_dbscan(eps=eps, min_points=100, print_progress=True))

    max_label = labels.max()
    print(f"Point cloud has {max_label + 1} clusters")

    # Preparing colors for visualization
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0  # Setting noise to black
    pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])



    # Correctly convert Open3D Vector3dVector to NumPy array before indexing
    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors)

    # Now you can use numpy boolean indexing
    clustered_points = points[labels >= 0]
    clustered_colors = colors[labels >= 0]

    # Create a new point cloud for clustered points
    clustered_pcd = o3d.geometry.PointCloud()
    clustered_pcd.points = o3d.utility.Vector3dVector(clustered_points)
    clustered_pcd.colors = o3d.utility.Vector3dVector(clustered_colors)

# 
    # output = './conveted_AI_7F_05'
    # Save the entire point cloud with cluster coloring
    o3d.io.write_point_cloud(f"{output}_with_noise.pcd", pcd)

    # Save the point cloud with only clustered points
    o3d.io.write_point_cloud(f"{output}_clustered.pcd", clustered_pcd)


    # Visualize both point clouds
    o3d.visualization.draw_geometries([pcd])  # Visualize all points with cluster coloring
    o3d.visualization.draw_geometries([clustered_pcd])  # Visualize only clustered points


    return max_label + 1

def SOR_point_cloud(input, nb_neighbors=130, std_ratio=0.0001):
    """
    Apply Statistical Outlier Removal (SOR) to a point cloud.   

    :param pcd: Open3D point cloud object.
    :param nb_neighbors: Number of neighbors to consider in the statistical analysis.
    :param std_ratio: Standard deviation ratio. Points with a distance larger than 
                      this ratio times the standard deviation of the mean distance 
                      will be considered as outliers.
    :return: Filtered point cloud.
    """

    pcd = o3d.io.read_point_cloud(input)

    # Compute the neighbors and the mean distances for each point
    distances = pcd.compute_nearest_neighbor_distance()
    mean_distance = np.mean(distances)
    std_dev = np.std(distances)

    # Threshold for considering a point as an outlier
    threshold = mean_distance + std_ratio * std_dev

    # Create a mask to filter out the outliers
    inlier_mask = distances < threshold

    # Filter the point cloud
    inlier_points = np.asarray(pcd.points)[inlier_mask]

    # Create a new point cloud for the inliers
    inlier_pcd = o3d.geometry.PointCloud()
    inlier_pcd.points = o3d.utility.Vector3dVector(inlier_points)
    o3d.visualization.draw_geometries([pcd]) 
    o3d.visualization.draw_geometries([inlier_pcd])  # Visualize all points with cluster coloring
    return inlier_pcd


def find_optimal_eps(input, min_points):
    pcd = o3d.io.read_point_cloud(input)

    eps_values = np.arange(0.1, 0.32, 0.015)  # eps from 0.1 to 2 in increments of 0.2
    cluster_counts = []

    for eps in eps_values:
        labels = np.array(pcd.cluster_dbscan(eps=eps, min_points=min_points, print_progress=True))
        n_clusters = len(set(labels)) - (1 if -1 in labels else 0)
        cluster_counts.append(n_clusters)

    # Plotting
    plt.figure(figsize=(10, 5))
    plt.plot(eps_values, cluster_counts, marker='o', label='Number of Clusters')
    plt.xlabel('Epsilon Value')
    plt.ylabel('Number of Clusters')
    plt.title('DBSCAN Clustering: Epsilon Parameter Tuning')
    plt.legend()
    plt.grid(True)
    plt.show()

    # Implementing a heuristic to find the optimal eps
    # This can be a point where the curve starts to plateau
    # Here we use a simple approach: find the first eps value where the increase in cluster count slows down
    # Note: This is a heuristic and might not be optimal for all datasets.
    optimal_eps_index = np.argmax(np.diff(cluster_counts) <= 1) + 1
    optimal_eps = eps_values[optimal_eps_index]

    return optimal_eps


# # Path to your PCD file
# input_file = './converted/converted_vision_tower_B3_05.pcd'  # Replace with the path to your PCD file
# output_file = './filtered/filtered_vision_tower_B3_05_40.pcd'  # Replace with your desired output file path

input_file = './converted/converted_AI_7F_05.pcd'
output_file = './filtered/filtered_AI_7F_05_40%.pcd'
# visualize_shared_xy_pcd(input_file, output_file=output_file)
visualize_shared_xy_pcd(input_file='./converted/converted_vision_tower_B3_05.pcd', output_file= './filtered/filtered_vision_tower_B3_05_33%.pcd')
# visualize_shared_xy_pcd(input_file='./converted/converted_gachon_station_05.pcd', output_file= './filtered/converted_gachon_station_05_33%.pcd')


# DBScan(input = './filtered/filtered_AI_7F_05_40%.pcd', output='./cluster1/AI_7F_05_33__0215.pcd',eps=0.215)
# DBScan(input = './filtered/filtered_gachon_hall_05_33%.pcd', output='./cluster1/gachon_hall_05_33_0175.pcd',eps=0.175)
# DBScan(input='./filtered/converted_gachon_station_05_33%.pcd', output='./filtered/converted_gachon_station_05_0215.pcd', eps=0.215)
# SOR_point_cloud('./cluster1/AI_7F_05_33_sor0_clustered.pcd','./cluster1/AI_7F_05_33_sor1_clustered.pcd')


# eps = find_optimal_eps('./filtered/filtered_vision_tower_B3_05_33%.pcd', 100)
# eps = find_optimal_eps('./filtered/filtered_gachon_hall_05_33%.pcd', 100)
# print(eps)
# DBScan(input = './filtered/filtered_vision_tower_B3_05_33%.pcd', output='./filtered/filtered_vision_tower_B3_05_0250_clustered.pcd',eps=0.25)
