#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml


# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

# Convert ROS msg to PCL data
    msg_to_pcl =  ros_to_pcl(pcl_msg)
# Statistical Outlier Filtering
    # We start by creating a filter object:
    outlier_filter = msg_to_pcl.make_statistical_outlier_filter()
    # Set the number of neighboring points to analyze for any given point
    outlier_filter.set_mean_k(5)
    # Set threshold scale factor
    x = 0.00000001
    # Any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
    outlier_filter.set_std_dev_mul_thresh(x)
    # Finally call the filter function for magic
    msg_to_pcl = outlier_filter.filter()


# Voxel Grid Downsampling
    vox = msg_to_pcl.make_voxel_grid_filter()
    LEAF_SIZE = 0.005
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    cloud_filtered = vox.filter()


# PassThrough Filter Z
    passthroughZ = cloud_filtered.make_passthrough_filter()
    filter_axis = 'z'
    passthroughZ.set_filter_field_name(filter_axis)
    axis_min = 0.6
    axis_max = 1.1
    # axis_min = 0.1
    # axis_max = 0.7
    passthroughZ.set_filter_limits(axis_min, axis_max)
    cloud_filtered = passthroughZ.filter()
    # Passthrough filter X to remove appearance of bin edges
    passthroughX = cloud_filtered.make_passthrough_filter()
    filter_axis = 'x'
    passthroughX.set_filter_field_name(filter_axis)
    axis_min = 0.4
    axis_max = 3.5
    #axis_min = 0.3
    #axis_max = 3.6
    passthroughX.set_filter_limits(axis_min, axis_max)
    cloud_filtered = passthroughX.filter()

# RANSAC Plane Segmentation
# Max distance for a point to be considered fitting the model
# Experiment with different values for max_distance
# for segmenting the table
    seg = cloud_filtered.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    max_distance = 0.008
    seg.set_distance_threshold(max_distance)

# Extract inliers and outliers
    inliers, coefficients = seg.segment()
    extracted_inliers = cloud_filtered.extract(inliers, negative=False)
    extracted_outliers = cloud_filtered.extract(inliers, negative=True)
    cloud_table = extracted_inliers
    cloud_objects = extracted_outliers

# Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    tree = white_cloud.make_kdtree()

# Create Cluster-Mask Point Cloud to visualize each cluster separately
# Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()

# Set tolerances for distance threshold
# as well as minimum and maximum cluster size (in points)

# Your task is to experiment and find values that work for segmenting objects.
    ec.set_ClusterTolerance(0.0500)
    ec.set_MinClusterSize(50)
    ec.set_MaxClusterSize(5000)

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

    #Convert PCL data to ROS messages
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cloud_table = pcl_to_ros(cloud_table)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)

# Publish ROS messages

    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cluster_cloud)


# Exercise-3 TODOs:

    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects_labels = []
    detected_objects = []
    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster
        pcl_cluster = cloud_objects.extract(pts_list)
        ros_cluster = pcl_to_ros(pcl_cluster)
                # Compute the associated feature vector
        chists = compute_color_histograms(ros_cluster, using_hsv=True)#ros_pcl_cluster, using_hsv=False)
        normals = get_normals(ros_cluster)#ros_pcl_cluster) as changed above
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))
                # Make the prediction
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)
                # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label,label_pos, index))

        # Add the detected object to the list of detected objects. Uses DetectedObject.msg, a
        # two line file that defines the msg fields
        do = DetectedObject()
        #Populate the msg fields
        do.label = label
        #Populate with cluster array
        do.cloud = ros_cluster
        #this is the detected objects list that we later compare the pick list to, outputs
        # format e.g. ['biscuits', 'soap'. 'soap2']
        detected_objects.append(do)
    #print('test = ', do.label)

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))
    # Publish the list of detected objects
    detected_objects_pub.publish(detected_objects)
    #Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    #Could add some logic to determine whether or not your object detections are robust
    #before calling pr2_mover()
    try:
        pr2_mover(detected_objects)
    except rospy.ROSInterruptException:
        pass

# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    # Initialize variables
    dict_list = []
    labels = []
    centroids = [] # to be list of tuples (x, y, z)

    # Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    dropbox_param = rospy.get_param('/dropbox')

    # Loop through the pick list, Parse parameters into individual variables
    object_list_name_list = []
    object_list_group_list = []

    for i in object_list_param:
        object_name = String()
        object_group = String()
        object_name.data = i['name']
        object_group.data = i['group']

        # Assign the arm to be used for pick_place
        arm_name = String()
        if i['group'] == 'red':
            arm_name.data = 'left'
        elif i['group'] == 'green':
            arm_name.data = 'right'

        # Get the PointCloud for a given object and obtain it's centroid
        for obj in object_list:
            if obj.label == object_name.data:
                pick_pose = Pose()
                labels.append(obj.label)
                points_arr = ros_to_pcl(obj.cloud).to_array()

                object_centroid = np.mean(points_arr, axis=0)[:3]
                centroids.append(object_centroid)

                pick_pose.position.x = np.asscalar(object_centroid[0])
                pick_pose.position.y = np.asscalar(object_centroid[1])
                pick_pose.position.z = np.asscalar(object_centroid[2])

        # Create 'place_pose' for the object
        for k in dropbox_param:
            if k['group'] == object_group.data:
                place_pose = Pose()
                place_pose.position.x = (k['position'][0])
                place_pose.position.y = (k['position'][1])
                place_pose.position.z = (k['position'][2])

        # Initialize test_scene_num
        test_scene_num = Int32()
        test_scene_num.data = 1
        yaml_dict = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose)

        # Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
        dict_list.append(yaml_dict)

        # TODO: Rotate PR2 in place to capture side tables for the collision map
        # I'm only doing the yaml exports for now, perception project instructions were a little confusing, need more time

        # Wait for 'pick_place_routine' service to come up
        rospy.wait_for_service('pick_place_routine')

        try:
            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

            # Insert your message variables to be sent as a service request
            resp = pick_place_routine(test_scene_num, arm_name, object_name, pick_pose, place_pose)

            print ("Response: ",resp.success)

        except rospy.ServiceException, e:
             print "Service call failed: %s"%e

    # Output your request parameters into output yaml file
    yaml_filename = 'output_3.yaml'

    send_to_yaml(yaml_filename, dict_list)


if __name__ == '__main__':

    # ROS node initialization
    rospy.init_node('clustering', anonymous=True)
    # Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)
    # Create Publishers
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)
    # Load Model From disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']
    # Initialize color_list
    get_color_list.color_list = []

    # Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
