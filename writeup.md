## Project: Perception Pick & Place

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

---
### Writeup / README

### Exercise 1, 2 and 3 pipeline implemented
#### 1. Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented.

Implementation of Image Recognition Pipeline:

1. Convert the point cloud which is passed in as a ROS message to PCL format.

  A. Initialized ROS node

  B. Created Subscriber for the point cloud topic using pcl_callback()
  
  C. Created two publishers for the point cloud data for the table and the objects on the table to topics called pcl_table and pcl_objects
  
  D. Spin while node is not shutdown
  
  E. Publish ROS messages from your pcl_callback()
  
  F. Relaunched segmentation.py via ./segmentation.py
  
  G. Topics /pcl_objects and /pcl_table successfully showed up in RViz.

2. Filter out the camera noise with the PCL statistical outlier filter. The adjustable parameters are the number k of neighbouring pixels to average over and the outlier threshold thr = mean_distance + x * std_dev. I used the RViz output image to tune these parameters judging by the visual output. I found that the values k = 8 and x = 0.3 performed best at removing as much noise as possible without deleting content.

3. Downsample the image with a PCL voxel grid filter to reduce processing time and memory consumption. The adjustable parameter is the leaf size which is the side length of the voxel cube to average over. A leaf size of 0.005 seemed a good compromise between gain in computation speed and resolution loss.

4. A passthrough filter to define the region of interest. This filter clips the volume to the specified range. My region of interest is within the range 0.6 < z < 1.1 and 0.34 < x < 1.0 which removes the dropboxes.

5. RANSAC plane segmentation in order to separate the table from the objects on it. A maximum threshold of 0.01 worked well to fit the geometric plane model. The cloud is then segmented in two parts the table (inliers) and the objects of interest (outliers). The segments are published to ROS as /pcl_table and /pcl_objects. From this point onwards the objects segment of the point cloud is used for further processing.

#### 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.

In order to detect individual objects, the point cloud needs to be clustered.

Following the lectures I applied Euclidean clustering. The parameters that worked best for me are a cluster tolerance of 0.02, a minimum cluster size of 40 and a maximum cluster size of 4000. The optimal values for Euclidean clustering depend on the leaf size defined above, since the voxel grid determines the point density in the image.

The search method is k-d tree, which is appropriate here since the objects are well separated the x and y directions (e.g seen when rotating the RViz view parallel to z-axis).

The clusters are colored for visualization in RViz, the corresponding ROS subject is /pcl_cluster.

The next part of the pipeline handles the actual object recognition using machine learning.

#### 2. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.


Here is an example of how to include an image in your writeup.

![demo-1](https://user-images.githubusercontent.com/20687560/28748231-46b5b912-7467-11e7-8778-3095172b7b19.png)

### Pick and Place Setup

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.

And here's another image! 
![demo-2](https://user-images.githubusercontent.com/20687560/28748286-9f65680e-7468-11e7-83dc-f1a32380b89c.png)

Spend some time at the end to discuss your code, what techniques you used, what worked and why, where the implementation might fail and how you might improve it if you were going to pursue this project further.  



