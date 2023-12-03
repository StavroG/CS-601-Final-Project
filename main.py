import open3d as o3d

# Create a RealSense sensor object
rs_sensor = o3d.t.io.RealSenseSensor()

# Initialize the sensor with default configuration
rs_sensor.init_sensor()

# Start capturing frames
rs_sensor.start_capture()

# Define the intrinsic parameters of the sensor
intrinsic = o3d.camera.PinholeCameraIntrinsic(
    o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)

# Create a list of all point clouds
pcds = []

# Capture 5 frames from the sensor
for _ in range(5):
    # Capture a frame
    frame = rs_sensor.capture_frame(True, True)  # wait for frames and align them

    # Create an RGB-D using the color and depth image from the sensor
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        frame.color.to_legacy(),
        frame.depth.to_legacy()
    )

    # Convert the RGB-D image to a point cloud
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd_image,
        intrinsic
    )

    # Add the new point cloud to the point clouds list
    pcds.append(pcd)

# Stop capturing frames
rs_sensor.stop_capture()

# Create an empty point cloud
merged_pcd = o3d.geometry.PointCloud()

for i in range(1, len(pcds)):
    target = pcds[0]
    source = pcds[i]

    # Perform ICP algorithm to align point clouds
    icp_result = o3d.pipelines.registration.registration_icp(
        source, target, max_correspondence_distance=0.02,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000)
    )

    # Get the transformation matrix from the registration result
    transformation_matrix = icp_result.transformation

    # Apply the transformation to align the second point cloud with the first
    source.transform(transformation_matrix)

    # Merge the aligned point clouds
    merged_pcd += source

    print("Finished with " + str(i) + "/" + str(len(pcds)))

# Visualize the merged point cloud
o3d.visualization.draw_geometries([merged_pcd])
