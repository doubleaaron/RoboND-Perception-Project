## Project: Perception Pick & Place

![headerimage](https://github.com/doubleaaron/RoboND-Perception-Project/blob/master/images/pr2_header.jpg)

---

### Writeup / README

In this project, you must assimilate your work from previous exercises to successfully complete a tabletop pick and place operation using PR2 and it's (RGB+Depth) camera.

The purpose of this Project is that given the cluttered tabletop scenario, you must implement a perception pipeline using your work from Perception Exercises 1,2 and 3 to identify target objects from a so-called “Pick-List” in that particular order, pick up those objects and place them in corresponding dropboxes with the PR2 robot. The PR2 has been outfitted with an RGB-D sensor much like the one you used in previous exercises. This sensor however is a bit noisy, much like real sensors, so you must employ Statistical Outlier removal filtering in PCL to remove this real world noise.


### Exercise 1, 2 and 3 pipeline implemented in [perception_pipeline.py](https://github.com/doubleaaron/RoboND-Perception-Project/blob/master/pr2_robot/scripts/perception_pipeline.py)
#### 1. Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented.

Implementation of Image Recognition Pipeline:

1. Convert the point cloud which is passed in as a ROS message to PCL format.
    
    A. Initialized ROS node
    ```python
    # ROS node initialization
    rospy.init_node('clustering', anonymous=True)
    ```
    
    B. Created Subscriber for the point cloud topic using pcl_callback()
    ```python
    # Create Subscribers
    pcl_sub = rospy.Subscriber("/sensor_stick/point_cloud", pc2.PointCloud2, pcl_callback, queue_size=1)
    ```
    
    C. Created two publishers for the point cloud data for the table and the objects on the table to topics called pcl_table and pcl_objects
    ```python
    # Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    ```
    
    D. Spin while node is not shutdown
    ```python
    # Spin while node is not shutdown
    while not rospy.is_shutdown():
    rospy.spin()
    ```
    E. Publish ROS messages from your pcl_callback()
    ```python
    # Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cluster_cloud)
    ```
    
    F. Verified that the topics /pcl_objects and /pcl_table successfully showed up in RViz interface.

2. Filtering: Filter out the camera noise with the PCL statistical outlier filter. The adjustable parameters are the number k of neighbouring pixels to average over and the outlier threshold thr = mean_distance + x * std_dev. I used the RViz output image to tune these parameters judging by the visual output. I found that the values k = 8 and x = 0.3 performed best at removing as much noise as possible without deleting content.

    A. Convert the message from a ROS message (which is in PointCloud2 message format) into PCL format for python-pcl using the ros_to_pcl(ros_cloud) function in pcl_helper.py
    ```python
    # Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg)
    ```
    
    B. Downsample your point cloud by applying a Voxel Grid Filter: Downsampling decreases the density of the pointcloud that is output from the RGB-D camera. RGBD cameras provide feature rich pointclouds and are computationally expensive. Downsampling decreases the resolution of a three dimensional point cloud. A LEAF_SIZE of .01 is what I ended up using.
    
    ![voxelgrid](https://github.com/doubleaaron/RoboND-Perception-Project/blob/master/images/voxel_grid.jpg)
    ```python
    # Voxel Grid Downsampling
    vox = cloud.make_voxel_grid_filter()
    LEAF_SIZE = .01
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    cloud_filtered = vox.filter()
    filename = 'voxel_downsampled.pcd'
    pcl.save(cloud_filtered, filename)
    ```
    
    C. Apply a Pass Through Filter to isolate the table and objects. Pass Through Filtering trims down our point cloud space along specified axes, in order to decrease the sample size. We will allow a specific region (ROI) to Pass Through. Pass Through Filtering can be thought of as cropping a 3 dimensional space. I found a min/max of 0.6 and 1.1 to work fairly well for the table top.
    
    ![passthroughfilter](https://github.com/doubleaaron/RoboND-Perception-Project/blob/master/images/passthrough_filtering.png)
    
    ```python
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
    ```
    
    D. Perform RANSAC plane fitting to identify the table.
    
    ![planesegmentation](https://github.com/doubleaaron/RoboND-Perception-Project/blob/master/images/segmentation.jpg)
    
    ```python
    # RANSAC Plane Segmentation
    seg = cloud_filtered.make_segmenter()
    # Set the model you wish to fit 
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    
    # Max distance for a point to be considered fitting the model
    # Experiment with different values for max_distance 
    # for segmenting the table
    max_distance = 0.01
    seg.set_distance_threshold(max_distance)

    # Call the segment function to obtain set of inlier indices and model coefficients
    inliers, coefficients = seg.segment()
    ```

    E. Use the ExtractIndices Filter to create new point clouds containing the table and objects separately (I'll call them cloud_table and cloud_objects going forward).
    ```python
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
    ```


#### 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.

In order to detect individual objects, the point cloud needs to be clustered.

Following the lectures I applied Euclidean clustering. The parameters that worked best for me are a cluster tolerance of 0.03, a minimum cluster size of 30 and a maximum cluster size of 1200. The optimal values for Euclidean clustering depend on the leaf size defined above, since the voxel grid determines the point density in the image.

```python
    # Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    tree = white_cloud.make_kdtree()



    # Create Cluster-Mask Point Cloud to visualize each cluster separately
    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    
    # Set tolerances for distance threshold 
    # as well as minimum and maximum cluster size (in points)
    
    # Your task is to experiment and find values that work for segmenting objects.
    ec.set_ClusterTolerance(0.03)
    ec.set_MinClusterSize(30)
    ec.set_MaxClusterSize(1200)
```

![clustering](https://github.com/doubleaaron/RoboND-Perception-Project/blob/master/images/rviz_euclidean_clustering.png)
    
The search method is k-d tree, which is appropriate here since the objects are well separated the x and y directions (e.g seen when rotating the RViz view parallel to z-axis).

```python
# Search the k-d tree for clusters
ec.set_SearchMethod(tree)
# Extract indices for each of the discovered clusters
cluster_indices = ec.Extract()
```

The clusters are colored for visualization in RViz, the corresponding ROS subject is /pcl_cluster.
    
```python
# Create a Cluster Visualization using PointCloud_PointXYZRGB
# Create Cluster-Mask Point Cloud to visualize each cluster separately
cluster_color = get_color_list(len(cluster_indices))

color_cluster_point_list = []

color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                            white_cloud[indice][1],
                                            white_cloud[indice][2],
                                            rgb_to_float(cluster_color[j])])

```

![clusteringcolors](https://github.com/doubleaaron/RoboND-Perception-Project/blob/master/images/rviz_euclidean_clustering_colors.png)

The next part of the pipeline handles the actual object recognition using machine learning with scikit-learn.

#### 2. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.

Both compute_color_histograms() and compute_normal_histograms() prior to training:

![confusionmatrix01](https://github.com/doubleaaron/RoboND-Perception-Project/blob/master/images/confusion_matrix_01.jpg)
![confusionmatrix02](https://github.com/doubleaaron/RoboND-Perception-Project/blob/master/images/confusion_matrix_02.jpg)

Not good on the confusion matrix front, since compute_color_histograms() and compute_normal_histograms() in the features.py file aren't filled out. We need to do some RGB color analysis on the point cloud.

 ```python
 def compute_color_histograms(cloud, using_hsv=True):

    # Compute histograms for the clusters
    point_colors_list = []

    # Step through each point in the point cloud
    for point in pc2.read_points(cloud, skip_nans=True):
        rgb_list = float_to_rgb(point[3])
        if using_hsv:
            point_colors_list.append(rgb_to_hsv(rgb_list) * 255)
        else:
            point_colors_list.append(rgb_list)

    # Populate lists with color values
    channel_1_vals = []
    channel_2_vals = []
    channel_3_vals = []

    for color in point_colors_list:
        channel_1_vals.append(color[0])
        channel_2_vals.append(color[1])
        channel_3_vals.append(color[2])
    
    # Compute histograms of color values (Color channels Range from 0 to 256(8 bit))
    nbins=32
    bins_range=(0,256)
    
    # Compute histograms
    r_hist = np.histogram(channel_1_vals, bins=nbins, range=bins_range)
    g_hist = np.histogram(channel_2_vals, bins=nbins, range=bins_range)
    b_hist = np.histogram(channel_3_vals, bins=nbins, range=bins_range)
    
    # Extract the features
    # Concatenate and normalize the histograms
    hist_features = np.concatenate((r_hist[0], g_hist[0], b_hist[0])).astype(np.float64)
    normed_features = hist_features / np.sum(hist_features)  

    return normed_features

 ```
 
Then we'll add the histogram, computer features, concatenate and normalize them. Normailizing here is in the -1 to 1 range unlike color which is 0-255.
 
 ```python
 def compute_normal_histograms(normal_cloud):
    norm_x_vals = []
    norm_y_vals = []
    norm_z_vals = []

    for norm_component in pc2.read_points(normal_cloud,
                                          field_names = ('normal_x', 'normal_y', 'normal_z'),
                                          skip_nans=True):
        norm_x_vals.append(norm_component[0])
        norm_y_vals.append(norm_component[1])
        norm_z_vals.append(norm_component[2])

    # Compute histograms of surface normal values (Surface Normals Range from -1 to 1)
    nbins= 32
    bins_range=(-1,1)

    # Compute histograms
    x_hist = np.histogram(norm_x_vals, bins=nbins, range=bins_range)
    y_hist = np.histogram(norm_y_vals, bins=nbins, range=bins_range)
    z_hist = np.histogram(norm_z_vals, bins=nbins, range=bins_range)
    
    # TODO: Concatenate and normalize the histograms
    # Extract the features
    # Concatenate and normalize the histograms
    hist_features = np.concatenate((x_hist[0], y_hist[0], z_hist[0])).astype(np.float64)
    normed_features = hist_features / np.sum(hist_features)  

    return normed_features
 ```
 
![tconfusionmatrix01](https://github.com/doubleaaron/RoboND-Perception-Project/blob/master/images/trained_confusion_matrix_03.jpg)
![tconfusionmatrix02](https://github.com/doubleaaron/RoboND-Perception-Project/blob/master/images/trained_confusion_matrix_04.jpg)

The next things we can work on are:

    Convert RGB to HSV
    Compute features for a larger set of random orientations of the objects
    Try different binning schemes with the histogram(32,64, etc)
    Modify the SVM parameters(kernel, regularization, etc)

Convert RGB to HSV:
Within the capture_features.py file change the value of using_hsv=False to True
```python
chists = compute_color_histograms(sample_cloud, using_hsv=False)
```

Randomly Spawn more objects in the capture_features.py (for loop) for i in range(5):
```python
for i in range(30)
```

For binning schemes I settled on 32:
bins=32

Within train_svm.py I messed around with the SVM kernels available in scikit:
(linear,rbf,sigmoid)

Some other ideas would be to use some different Classifiers in scikit like ExtraTrees or GradientBoosting:
```python
clf = ExtraTreesClassifier(n_estimators=10, max_depth=None,
                           min_samples_split=2, random_state=0)
                           
clf = GradientBoostingClassifier(n_estimators=10, learning_rate=1.0,
                                 max_depth=1, random_state=0).fit(X_train, y_train)
```
If I have time I will come back to this point and play with ExtraTrees and GradientBoosting, but for now I have a good classification system going.


### Pick and Place Setup

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.

PR2 Models trained on Perception Pipeline:

![tconfusionmatrix01](https://github.com/doubleaaron/RoboND-Perception-Project/blob/master/images/pr2_trained_confusion_matrix_01.jpg)
![tconfusionmatrix02](https://github.com/doubleaaron/RoboND-Perception-Project/blob/master/images/pr2_trained_confusion_matrix_02.jpg)

Plugging everything we built into the project.py file I'll start out by changing the subscriber to the camera data (point cloud) topic /pr2/world/points

```python
# Create Subscriber for camera data (point cloud) from topic /pr2/world/points
pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)
```

Save your PickPlace requests into output_1.yaml, output_2.yaml, and output_3.yaml for each scene respectively.

In the launch file: pick_place_project.launch in pr2_robot/launch at lines 13 and 39 have world parameters you need to alter.
```
<arg name="world_name" value="$(find pr2_robot)/worlds/test3.world"/>
<rosparam command="load" file="$(find pr2_robot)/config/pick_list_3.yaml"/>
```
Identifying items in each Test World:
![pickandplace](https://github.com/doubleaaron/RoboND-Perception-Project/blob/master/images/pick_and_place_pr2.jpg)
![pickandplace01](https://github.com/doubleaaron/RoboND-Perception-Project/blob/master/images/pick_and_place_pr2_01.jpg)
![pickandplace02](https://github.com/doubleaaron/RoboND-Perception-Project/blob/master/images/pick_and_place_pr2_02.jpg)

I would like to work on the PR2 movement challenge but I have run out of time. In the future I will come back to this and complete it.

In this project I was able to filter noise out of RGBD sensor data, downsample the point cloud, segment areas out of the scene that weren't wanted, separate objects out for identification, cluster objects and color mark them as well as train a Support Vector Machine to correctly identify objects with very high accuracy and label those objects in the PR2 robot's camera and display in RVIZ.

I'd like to continue the pick and place challenge but I don't have enough time. It would also be nice to test this on my RGBD Kinect sensor at some point.
