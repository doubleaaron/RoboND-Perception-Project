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
    # NOTE: These are poor choices of clustering parameters
    # Your task is to experiment and find values that work for segmenting objects.
    ec.set_ClusterTolerance(0.03)
    ec.set_MinClusterSize(30)
    ec.set_MaxClusterSize(1200)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()
```

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

The next part of the pipeline handles the actual object recognition using machine learning.

#### 2. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.


Here is an example of how to include an image in your writeup.

![demo-1](https://user-images.githubusercontent.com/20687560/28748231-46b5b912-7467-11e7-8778-3095172b7b19.png)

### Pick and Place Setup

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.

And here's another image! 
![demo-2](https://user-images.githubusercontent.com/20687560/28748286-9f65680e-7468-11e7-83dc-f1a32380b89c.png)

Spend some time at the end to discuss your code, what techniques you used, what worked and why, where the implementation might fail and how you might improve it if you were going to pursue this project further.



---

# Required Steps for a Passing Submission:
1. Extract features and train an SVM model on new objects (see `pick_list_*.yaml` in `/pr2_robot/config/` for the list of models you'll be trying to identify). 
2. Write a ROS node and subscribe to `/pr2/world/points` topic. This topic contains noisy point cloud data that you must work with.
3. Use filtering and RANSAC plane fitting to isolate the objects of interest from the rest of the scene.
4. Apply Euclidean clustering to create separate clusters for individual items.
5. Perform object recognition on these objects and assign them labels (markers in RViz).
6. Calculate the centroid (average in x, y and z) of the set of points belonging to that each object.
7. Create ROS messages containing the details of each object (name, pick_pose, etc.) and write these messages out to `.yaml` files, one for each of the 3 scenarios (`test1-3.world` in `/pr2_robot/worlds/`).  [See the example `output.yaml` for details on what the output should look like.](https://github.com/udacity/RoboND-Perception-Project/blob/master/pr2_robot/config/output.yaml)  
8. Submit a link to your GitHub repo for the project or the Python code for your perception pipeline and your output `.yaml` files (3 `.yaml` files, one for each test world).  You must have correctly identified 100% of objects from `pick_list_1.yaml` for `test1.world`, 80% of items from `pick_list_2.yaml` for `test2.world` and 75% of items from `pick_list_3.yaml` in `test3.world`.
9. Congratulations!  Your Done!

# Extra Challenges: Complete the Pick & Place
7. To create a collision map, publish a point cloud to the `/pr2/3d_map/points` topic and make sure you change the `point_cloud_topic` to `/pr2/3d_map/points` in `sensors.yaml` in the `/pr2_robot/config/` directory. This topic is read by Moveit!, which uses this point cloud input to generate a collision map, allowing the robot to plan its trajectory.  Keep in mind that later when you go to pick up an object, you must first remove it from this point cloud so it is removed from the collision map!
8. Rotate the robot to generate collision map of table sides. This can be accomplished by publishing joint angle value(in radians) to `/pr2/world_joint_controller/command`
9. Rotate the robot back to its original state.
10. Create a ROS Client for the “pick_place_routine” rosservice.  In the required steps above, you already created the messages you need to use this service. Checkout the [PickPlace.srv](https://github.com/udacity/RoboND-Perception-Project/tree/master/pr2_robot/srv) file to find out what arguments you must pass to this service.
11. If everything was done correctly, when you pass the appropriate messages to the `pick_place_routine` service, the selected arm will perform pick and place operation and display trajectory in the RViz window
12. Place all the objects from your pick list in their respective dropoff box and you have completed the challenge!
13. Looking for a bigger challenge?  Load up the `challenge.world` scenario and see if you can get your perception pipeline working there!

## [Rubric](https://review.udacity.com/#!/rubrics/1067/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  



