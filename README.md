# RoboND-Perception-Project
## Project: Perception Pick & Place

---

## The Steps Demonstrated in project_template.py:

1. Extracts features and train an SVM model on new objects in `pick_list_*.yaml` in `/pr2_robot/config/`. 
2. Creates a ROS node and subscribes to the `/pr2/world/points` topic. This topic contains noisy point cloud data that is filtered with a statistical output filter.
3. Uses filtering and RANSAC plane fitting to isolate the objects of interest from the rest of the scene.
4. Applies Euclidean clustering to create separate clusters for individual items.
5. Performs object recognition on these objects and assign them labels as markers in RViz.
6. Calculates the centroid (average in x, y and z) of the set of points belonging to that each object.
7. Creates ROS messages containing the details of each object (name, pick_pose, etc.) and writes these messages out to `.yaml` files, one for each of the 3 scenarios (`test1-3.world` in `/pr2_robot/worlds/`).
8. I created a project in my GitHub repo for the project including the Python code for your perception pipeline and the output `.yaml` files (3 `.yaml` files, one for each test world).  I have correctly identified 100% of objects from `pick_list_1.yaml` for `test1.world`, 80% of items from `pick_list_2.yaml` for `test2.world` and 75% of items from `pick_list_3.yaml` in `test3.world`.

## Extra Challenges: The Pick & Place

9. I created a collision map by publishing a point cloud to the `/pr2/3d_map/points` topic and changed the `point_cloud_topic` to `/pr2/3d_map/points` in `sensors.yaml` in the `/pr2_robot/config/` directory. This topic is read by Moveit!, which uses this point cloud input to generate a collision map, allowing the robot to plan its trajectory.  Keep in mind that later when you go to pick up an object, you must first remove it from this point cloud so it is removed from the collision map!
10. Rotate the robot to generate collision map of table sides. This can be accomplished by publishing joint angle value(in radians) to `/pr2/world_joint_controller/command`.
11. Rotate the robot back to its original state.
12. Create a ROS Client for the “pick_place_routine” rosservice.  In the required steps above, you already created the messages you need to use this service. Checkout the [PickPlace.srv](https://github.com/udacity/RoboND-Perception-Project/tree/master/pr2_robot/srv) file to find out what arguments you must pass to this service.
13. If everything was done correctly, when you pass the appropriate messages to the `pick_place_routine` service, the selected arm will perform pick and place operation and display trajectory in the RViz window
14. Place all the objects from your pick list in their respective dropoff box and you have completed the challenge!
15. Looking for a bigger challenge?  Load up the `challenge.world` scenario and see if you can get your perception pipeline working there!

### Exercise 1, 2 and 3 Pipeline

#### 1. Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented.

#### 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.  

#### 2. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.

Here is an example of how to include an image in your writeup.

![demo-1](https://user-images.githubusercontent.com/20687560/28748231-46b5b912-7467-11e7-8778-3095172b7b19.png)




Here's | A | Snappy | Table
--- | --- | --- | ---
1 | `highlight` | **bold** | 7.41
2 | a | b | c
3 | *italic* | text | 403
4 | 2 | 3 | abcd


Spend some time at the end to discuss your code, what techniques you used, what worked and why, where the implementation might fail and how you might improve it if you were going to pursue this project further.  



