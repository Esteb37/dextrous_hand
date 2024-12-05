import numpy as np
import open3d as o3d
import cv2
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import PointField
from sensor_msgs import point_cloud2

class PointCloudVisualizer:
    def __init__(self, intrinsic_matrix, width, height, visualize=False):
        self.R_camera_to_world = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]]).astype(
            np.float64
        )
        self.depth_map = None
        self.rgb = None
        self.pcl = o3d.geometry.PointCloud()

        self.pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
            width,
            height,
            intrinsic_matrix[0][0],
            intrinsic_matrix[1][1],
            intrinsic_matrix[0][2],
            intrinsic_matrix[1][2],
        )
        self.visualize = visualize
        if visualize:
            self.vis = o3d.visualization.Visualizer() # type: ignore
            self.vis.create_window(window_name="Point Cloud")
            self.vis.add_geometry(self.pcl)
            origin = o3d.geometry.TriangleMesh.create_coordinate_frame(
                size=0.3, origin=[0, 0, 0]
            )
            self.vis.add_geometry(origin)
            view_control = self.vis.get_view_control()
            view_control.set_constant_z_far(1000)

        self.isstarted = False

    def rgbd_to_projection(self, depth_map, rgb, downsample=False, remove_noise=False):
        rgb_o3d = o3d.geometry.Image(rgb)
        depth_o3d = o3d.geometry.Image(depth_map)

        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            rgb_o3d,
            depth_o3d,
            convert_rgb_to_intensity=(len(rgb.shape) != 3),
            depth_trunc=20000,
            depth_scale=1000.0,
        )

        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd_image, self.pinhole_camera_intrinsic
        )

        if downsample:
            pcd = pcd.voxel_down_sample(voxel_size=0.01)

        if remove_noise:
            pcd = pcd.remove_statistical_outlier(30, 0.1)[0]

        self.pcl.points = pcd.points
        self.pcl.colors = pcd.colors
        self.pcl.rotate(
            self.R_camera_to_world, center=np.array([0, 0, 0], dtype=np.float64)
        )
        return self.pcl, rgbd_image

    def visualize_pcd(self):
        if self.visualize:
            self.vis.update_geometry(self.pcl)
            self.vis.poll_events()
            self.vis.update_renderer()

    def close_window(self):
        if self.visualize:
            self.vis.destroy_window()


def calibrate_with_aruco(frame, camera_matrix, dist_coeffs, marker_length = 0.1):
    # Load camera calibration data (camera matrix and distortion coefficients)
    # You should replace these with the actual values from your camera calibration
    # Define the ArUco marker dictionary and the size of the marker (in meters)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    parameters =  cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)

    camera_matrix = np.array(camera_matrix)
    dist_coeffs = np.array(dist_coeffs)
    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)


    # Detect ArUco markers in the image

    corners, ids, rejected = detector.detectMarkers(gray)

    # If at least one marker is detected
    if ids is not None:
        print(f"Detected {len(ids)} markers")
        # Estimate pose of each marker
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)

        # Loop through detected markers
        for i in range(len(ids)):
            print(f"Marker ID: {ids[i]}")
            print(f"Rotation Vector (rvec):\n {rvecs[i]}")
            print(f"Translation Vector (tvec):\n {tvecs[i]}\n")
    else:
        print("No markers found")
        return None



    # Convert the rotation vector (rvecs) to a 3x3 rotation matrix
    R_cam_marker, _ = cv2.Rodrigues(rvecs[0])  # Convert rvec to a 3x3 rotation matrix

    # Initialize a 4x4 identity matrix for the transformation
    T_cam_marker = np.eye(4)

    # Set the upper-left 3x3 part to the rotation matrix
    T_cam_marker[:3, :3] = R_cam_marker

    # Set the upper-right 3x1 part to the translation vector (tvec)
    T_cam_marker[:3, 3] = tvecs[0].flatten()

    # Now T_cam_marker is the 4x4 transformation matrix from the camera to the marker
    T_cam_world = get_camera_in_world(T_cam_marker)
    return T_cam_world


def get_camera_in_world(T_aruco_cam):
    """
    Calculate the transformation matrix of the camera in the world frame.

    Args:
        T_aruco_world (np.ndarray): 4x4 transformation matrix of ArUco in the world frame.
        T_aruco_cam (np.ndarray): 4x4 transformation matrix of ArUco in the camera frame.

    Returns:
        np.ndarray: 4x4 transformation matrix of the camera in the world frame.
    """
    T_aruco_world_position = np.array([0, 0, 0])
    T_aruco_world_euler = np.array([0, 0, 0])
    rotation_matrix = R.from_euler('xyz', T_aruco_world_euler).as_matrix()

    # Construct the 4x4 transformation matrix
    T_aruco_world = np.eye(4)
    T_aruco_world[:3, :3] = rotation_matrix
    T_aruco_world[:3, 3] = T_aruco_world_position

    # Compute the inverse of T_aruco_cam
    T_cam_aruco = np.linalg.inv(T_aruco_cam)

    # Compute T_cam_world
    T_cam_world = np.dot(T_aruco_world, T_cam_aruco)

    return T_cam_world

def compute_projection_matrix(K, T_cam_world):
    # Extract R and t from T_world_to_camera
    R = T_cam_world[:3, :3]
    t = T_cam_world[:3, 3]

    # Form the 3x4 extrinsics matrix [R | t]
    extrinsics = np.hstack((R, t.reshape(3, 1)))

    # Compute the projection matrix P
    P = K @ extrinsics
    return P

def rgb_depth_to_pointcloud(rgb_image, depth_image, camera_intrinsics, header):
    """
    Convert an RGB image and a depth image to a ROS 2 PointCloud2 message.

    :param rgb_image: RGB image as a numpy array of shape (H, W, 3).
    :param depth_image: Depth image as a numpy array of shape (H, W), in meters.
    :param camera_intrinsics: intrinsic matrix of the camera.
    :return: PointCloud2 message.
    """
    fx = camera_intrinsics[0, 0]
    fy = camera_intrinsics[1, 1]
    cx = camera_intrinsics[0, 2]
    cy = camera_intrinsics[1, 2]

    # Take only one fourth of the image
    depth_image = depth_image[::2, ::2]
    rgb_image = rgb_image[::2, ::2]

    # Get image dimensions
    height, width = depth_image.shape

    # Create mesh grid of pixel coordinates
    u, v = np.meshgrid(np.arange(width), np.arange(height))

    # Back-project depth to 3D points
    z = depth_image.flatten() / 1000.0  # Convert to meters
    x = (u.flatten() - cx) * z / fx
    y = (v.flatten() - cy) * z / fy
    z *= -1
    z += 1

    # Create 3D points (N, 3)
    points = np.stack((x, y, z), axis=-1)

    # Filter out points with zero depth
    valid = z > 0
    # Filter out points further than a meter
    valid = np.logical_and(valid, z < 2)

    points = points[valid]


    # Get corresponding RGB values
    colors = rgb_image.reshape(-1, 3)[valid]

    # Combine points and colors into a structured array
    cloud_data = np.zeros(points.shape[0], dtype=[
        ('x', np.float32), ('y', np.float32), ('z', np.float32),
        ('rgb', np.uint32)
    ])
    cloud_data['x'], cloud_data['y'], cloud_data['z'] = points[:, 0], points[:, 1], points[:, 2]
    cloud_data['rgb'] = (colors[:, 0] << 16) + (colors[:, 1] << 8) + colors[:, 2]
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)
    ]
    point_cloud_msg = point_cloud2.create_cloud(header, fields, cloud_data)
    return point_cloud_msg
