#!/usr/bin/env python

# Import modules
from pcl_helper import *

# TODO: Define functions as required

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

	# Convert ROS msg to PCL data
	cloud = ros_to_pcl(pcl_msg)

	# Voxel Grid Downsampling
	vox = cloud.make_voxel_grid_filter()
	LEAF_SIZE = .01
	vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
	cloud_filtered = vox.filter()
	filename = 'voxel_downsampled.pcd'
	pcl.save(cloud_filtered, filename)

	# PassThrough Filter
	passthrough = cloud_filtered.make_passthrough_filter()
	
	# Assign axis and range to the passthrough filter object.
	filter_axis = 'x'
	passthrough.set_filter_field_name(filter_axis)
	axis_min = 0.4
	axis_max = 3.
	passthrough.set_filter_limits(axis_min, axis_max)

	filter_axis = 'z'
	passthrough.set_filter_field_name(filter_axis)
	axis_min = 0.6
	axis_max = 1.1
	passthrough.set_filter_limits(axis_min, axis_max)

	# Finally use the filter function to obtain the resultant point cloud. 
	cloud_filtered = passthrough.filter()
	filename = 'pass_through_filtered.pcd'
	pcl.save(cloud_filtered, filename)


	# RANSAC Plane Segmentation
	seg = cloud_filtered.make_segmenter()
	# Set the model you wish to fit 
	seg.set_model_type(pcl.SACMODEL_PLANE)
	seg.set_method_type(pcl.SAC_RANSAC)

	# Max distance for a point to be considered fitting the model
	max_distance = 0.01
	seg.set_distance_threshold(max_distance)

	# Segment Function to obtain set of inlier indices/model coeffs
	inliers, coefficients = seg.segment()
	outliers, coefficients = seg.segment()

	# Extract inliers and outliers
	extracted_inliers = cloud_filtered.extract(inliers, negative=False)
	filename = 'extracted_inliers.pcd'
	pcl.save(extracted_inliers, filename)

	# Extract outliers
	extracted_outliers = cloud_filtered.extract(outliers, negative=True)
	filename = 'extracted_outliers.pcd'
	pcl.save(extracted_outliers, filename)

	cloud_table = extracted_inliers
	cloud_objects = extracted_outliers

	# Euclidean Clustering
	white_cloud = XYZRGB_to_XYZ(cloud_objects)
	tree = white_cloud.make_kdtree()

	# Create a cluster extraction object
	ec = white_cloud.make_EuclideanClusterExtraction()

	# Set tolerances for distance threshold 
	# as well as minimum and maximum cluster size (in points)
	# NOTE: These are poor choices of clustering parameters
	# Your task is to experiment and find values that work for segmenting objects.
	ec.set_ClusterTolerance(0.03)
	ec.set_MinClusterSize(30)
	ec.set_MaxClusterSize(1200)
	# Search the k-d tree for clusters
	ec.set_SearchMethod(tree)
	# Extract indices for each of the discovered clusters
	cluster_indices = ec.Extract()

	# Create a Cluster Visualization using PointCloud_PointXYZRGB
	# Create Cluster-Mask Point Cloud to visualize each cluster separately
	cluster_color = get_color_list(len(cluster_indices))

	color_cluster_point_list = []

	for j, indices in enumerate(cluster_indices):
		for i, indice in enumerate(indices):
			color_cluster_point_list.append([white_cloud[indice][0],
											white_cloud[indice][1],
											white_cloud[indice][2],
											rgb_to_float(cluster_color[j])])

	#Create new cloud containing all clusters, each with unique color
	cluster_cloud = pcl.PointCloud_PointXYZRGB()
	cluster_cloud.from_list(color_cluster_point_list)


	# Convert PCL data to ROS messages
	ros_cloud_objects = pcl_to_ros(cloud_objects)
	ros_cloud_table = pcl_to_ros(cloud_table)
	ros_cluster_cloud = pcl_to_ros(cluster_cloud)

	# Publish ROS messages
	pcl_objects_pub.publish(ros_cloud_objects)
	pcl_table_pub.publish(ros_cloud_table)
	pcl_cluster_pub.publish(ros_cluster_cloud)


if __name__ == '__main__':

	# TODO: ROS node initialization
	rospy.init_node('clustering', anonymous=True)

	# TODO: Create Subscribers
	pcl_sub = rospy.Subscriber("/sensor_stick/point_cloud", pc2.PointCloud2, pcl_callback, queue_size=1)

	# TODO: Create Publishers
	pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
	pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
	pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)

	# Initialize color_list
	get_color_list.color_list = []

# TODO: Spin while node is not shutdown
while not rospy.is_shutdown():
	rospy.spin()