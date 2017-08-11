# RoboND-Perception-Project
## Project: Perception Pick & Place

---

## The Steps Demonstrated in project_template.py:

1. Extracted features and train an SVM model on new objects in `pick_list_*.yaml` in `/pr2_robot/config/`.  I left the sensor_stick project in the src directory, changed the list of models and extracted HSV color and surface normals histogram features, and saved to models.sav. I then reran the svm classifier to create the training set. I only got 76% in the cnofusion matrix, but still that proved to be adequate. See Excercise 3 below for details.
2. Created a ROS node and subscribed to the `/pr2/world/points` topic. This topic contains noisy point cloud data that is filtered with a statistical output filter.
3. Used filtering and RANSAC plane fitting to isolate the objects of interest from the rest of the scene.
4. Applied Euclidean clustering to create separate clusters for individual items.
5. Performed object recognition on these objects and assign them labels as markers in RViz.
6. Calculated the centroid (average in x, y and z) of the set of points belonging to that each object.
7. Created ROS messages containing the details of each object (name, pick_pose, etc.) and writes these messages out to `.yaml` files, one for each of the 3 scenarios (`test1-3.world` in `/pr2_robot/worlds/`).
8. I created a project in my GitHub repo for the project including the Python code for your perception pipeline and the output `.yaml` files (3 `.yaml` files, one for each test world).  I have correctly identified 100% of objects from `pick_list_1.yaml` for `test1.world`, over 80% of items from `pick_list_2.yaml` for `test2.world` and over 75% of items from `pick_list_3.yaml` in `test3.world`. The YAML output files are shown later in the results section.

## Extra Challenges: The Pick & Place

9. I created a collision map by publishing a point cloud to the `/pr2/3d_map/points` topic and changed the `point_cloud_topic` to `/pr2/3d_map/points` in `sensors.yaml` in the `/pr2_robot/config/` directory. This topic is read by Moveit!, which uses this point cloud input to generate a collision map, allowing the robot to plan its trajectory.  The issue here is that when you go to pick up an object, you must first remove it from this point cloud so it is removed from the collision map!
10. I rotated the robot to generate collision map of table sides. This was accomplished by publishing joint angle value (in radians) to `/pr2/world_joint_controller/command`. I adapted the code from a prvious project to pr=erform the movement:
```
# PR2 movement support routines

def at_goal(pos, goal):
    tolerance = .05
    result = abs(pos - goal) <= abs(tolerance)
    return result

def turn_pr2(pos):
    time_elapsed = rospy.Time.now()
    pub_body.publish(pos)

    while True:
        joint_state = rospy.wait_for_message('/pr2/joint_states', JointState)
	      #print "turn_pr2: Request: %f Joint %s=%f" % (pos, joint_state.name[19], joint_state.position[19])
        if at_goal(joint_state.position[19], pos):
            time_elapsed = joint_state.header.stamp - time_elapsed
            break

    return time_elapsed
```
I then called `turn_pr2` in the project_template.py `pr_mover()` function:
```
    	# Rotate PR2 in place to capture side tables for the collision map
	if with_collision_map == True:
		print "Sending command to scan for obstacles..."
		dtime1 = turn_pr2(np.pi/2.0)	# right
    dtime2 = turn_pr2(-np.pi/2.0)	# left
    dtime3 = turn_pr2(0.0)		# back home
```
Note that this code is conditionally called as it is very slow to execute.
11. Rotate the robot back to its original state, in the 3rd call to turn_pr2 to 0.0 radians.
12. Create a ROS Client for the “pick_place_routine” rosservice.  In the required steps above, you already created the messages you need to use this service. Checkout the [PickPlace.srv](https://github.com/udacity/RoboND-Perception-Project/tree/master/pr2_robot/srv) file to find out what arguments you must pass to this service.
13. I passed the messages to the `pick_place_routine` service, and the selected arm performed the pick and place operations and displayed the trajectory in the RViz window.
14. Place all the objects from your pick list in their respective dropoff box and you have completed the challenge!
15. Looking for a bigger challenge?  Load up the `challenge.world` scenario and see if you can get your perception pipeline working there!

### Recognition Pipeline

#### 1. Exercise 1. 

Pipeline for filtering and RANSAC plane fitting implemented.
The steps are the following:

1. Downsample your point cloud by applying a Voxel Grid Filter.
2. Apply a Pass Through Filter to isolate the table and objects.
3. Perform RANSAC plane fitting to identify the table.
4. Use the ExtractIndices Filter to create new point clouds containing the table and objects separately.


#### 2. Exercise 2: 

I added clustering for segmentation to the pipeline. 

Create a python ros node that subscribes to /sensor_stick/point_cloud topic. Use the template.py file found under /sensor_stick/scripts/ to get started.

Use your code from Exercise-1 to apply various filters and segment the table using RANSAC.

Create publishers and topics to publish the segmented table and tabletop objects as separate point clouds

Apply Euclidean clustering on the table-top objects (after table segmentation is successful)

Create a XYZRGB point cloud such that each cluster obtained from the previous step has its own unique color.

Finally publish your colored cluster cloud on a separate topic 

#### 3. Exercise 3  

Features extracted and SVM trained.  Object recognition implemented.
#### Preparing for training

Launch the training.launch file to bring up the Gazebo environment:

`$ roslaunch sensor_stick training.launch`

You should see an empty scene in Gazebo with only the sensor stick robot.

#### Capturing Features

Next, in a new terminal, run the capture_features.py script to capture and save features for each of the objects in the environment. This script spawns each object in random orientations (default 5 orientations per object) and computes features based on the point clouds resulting from each of the random orientations.

`rosrun sensor_stick capture_features.py`

The features will now be captured and you can watch the objects being spawned in Gazebo. It should take 5-10 sec. for each random orientations (depending on your machine's resources) so with 7 objects total it takes awhile to complete. When it finishes running you should have a training_set.sav file.

#### Training

After that, you're ready to run the train_svm.py model to train an SVM classifier on your labeled set of features.

`rosrun sensor_stick train_svm.py`

Note: Running this exercise out of the box your classifier will have poor performance because the functions compute_color_histograms() and compute_normal_histograms() (within features.py in /sensor_stick/src/sensor_stick) are generating random junk. Fix them in order to generate meaningful features and train your classifier!

#### Classifying Segmented Objects

If everything went well you now have a trained classifier and you're ready to do object recognition! First you have to build out your node for segmenting your point cloud. This is where you'll bring in your code from Exercises 1 and 2.

Make yourself a copy of the template.py file in the sensor_stick/scripts/ directory and call it something like object_recognition.py. Inside this file, you'll find all the TODO's from Exercises 1 and 2 and you can simply copy and paste your code in there from the previous exercises.

The new code you need to add is listed under the Exercise-3 TODO's in the pcl_callback() function. You'll also need to add some new publishers for outputting your detected object clouds and label markers. For the step-by-step instructions on what to add in these Exercise-3 TODOs, see the lesson in the classroom.


### Results

<center>

| ***Confusion Matrix*** |
|:-------------:|
| ![Confusion Matrix](output/ConfusionMatrix.png) |
| *** 76% score, not great, but adequate *** |

## Test Images Showing Labeling

| ***Test Output*** |
|:-------------:|
| ![Test 1](output/test1aobjects.png) |
| ***Test 1 -  3 of 3 objects found and labeled *** |
| ![Test 2](output/test2aobjects.png) |
| ***Test 2 - 5 of 5 objects found and labeled *** |
| ![Test 3](output/test3aobjects.png) |
| ***Test 3 - 8 of 8 objects found and labeled *** |

### Test 1 YAML

```
object_list:
- arm_name: right
  object_name: biscuits
  pick_pose:
    orientation:
      w: 0.0
      x: 0.0
      y: 0.0
      z: 0.0
    position:
      x: 0.5422409772872925
      y: -0.24230962991714478
      z: 0.7053117752075195
  place_pose:
    orientation:
      w: 0.0
      x: 0.0
      y: 0.0
      z: 0.0
    position:
      x: -0.1
      y: -0.71
      z: 0.605
  test_scene_num: 1
- arm_name: right
  object_name: soap
  pick_pose:
    orientation:
      w: 0.0
      x: 0.0
      y: 0.0
      z: 0.0
    position:
      x: 0.5407626032829285
      y: -0.020037328824400902
      z: 0.6735755801200867
  place_pose:
    orientation:
      w: 0.0
      x: 0.0
      y: 0.0
      z: 0.0
    position:
      x: -0.1
      y: -0.71
      z: 0.605
  test_scene_num: 1
- arm_name: left
  object_name: soap2
  pick_pose:
    orientation:
      w: 0.0
      x: 0.0
      y: 0.0
      z: 0.0
    position:
      x: 0.44517120718955994
      y: 0.22279632091522217
      z: 0.6767338514328003
  place_pose:
    orientation:
      w: 0.0
      x: 0.0
      y: 0.0
      z: 0.0
    position:
      x: -0.1
      y: 0.71
      z: 0.605
  test_scene_num: 1
```

### Test 2 YAML

```object_list:
- arm_name: right
  object_name: biscuits
  pick_pose:
    orientation:
      w: 0.0
      x: 0.0
      y: 0.0
      z: 0.0
    position:
      x: 0.5715519189834595
      y: -0.2491622269153595
      z: 0.7049819231033325
  place_pose:
    orientation:
      w: 0.0
      x: 0.0
      y: 0.0
      z: 0.0
    position:
      x: -0.1
      y: -0.71
      z: 0.605
  test_scene_num: 2
- arm_name: right
  object_name: soap
  pick_pose:
    orientation:
      w: 0.0
      x: 0.0
      y: 0.0
      z: 0.0
    position:
      x: 0.5601557493209839
      y: 0.00291590578854084
      z: 0.6745651364326477
  place_pose:
    orientation:
      w: 0.0
      x: 0.0
      y: 0.0
      z: 0.0
    position:
      x: -0.1
      y: -0.71
      z: 0.605
  test_scene_num: 2
- arm_name: left
  object_name: book
  pick_pose:
    orientation:
      w: 0.0
      x: 0.0
      y: 0.0
      z: 0.0
    position:
      x: 0.581922709941864
      y: 0.27770471572875977
      z: 0.7205572128295898
  place_pose:
    orientation:
      w: 0.0
      x: 0.0
      y: 0.0
      z: 0.0
    position:
      x: -0.1
      y: 0.71
      z: 0.605
  test_scene_num: 2
- arm_name: left
  object_name: soap2
  pick_pose:
    orientation:
      w: 0.0
      x: 0.0
      y: 0.0
      z: 0.0
    position:
      x: 0.44417712092399597
      y: 0.22744213044643402
      z: 0.6747559905052185
  place_pose:
    orientation:
      w: 0.0
      x: 0.0
      y: 0.0
      z: 0.0
    position:
      x: -0.1
      y: 0.71
      z: 0.605
  test_scene_num: 2
- arm_name: left
  object_name: glue
  pick_pose:
    orientation:
      w: 0.0
      x: 0.0
      y: 0.0
      z: 0.0
    position:
      x: 0.6312971115112305
      y: 0.13123351335525513
      z: 0.6794822812080383
  place_pose:
    orientation:
      w: 0.0
      x: 0.0
      y: 0.0
      z: 0.0
    position:
      x: -0.1
      y: 0.71
      z: 0.605
  test_scene_num: 2
```
### Test 3 YAML

```
object_list:
- arm_name: left
  object_name: sticky_notes
  pick_pose:
    orientation:
      w: 0.0
      x: 0.0
      y: 0.0
      z: 0.0
    position:
      x: 0.43881526589393616
      y: 0.21614859998226166
      z: 0.6775033473968506
  place_pose:
    orientation:
      w: 0.0
      x: 0.0
      y: 0.0
      z: 0.0
    position:
      x: -0.1
      y: 0.71
      z: 0.605
  test_scene_num: 3
- arm_name: left
  object_name: book
  pick_pose:
    orientation:
      w: 0.0
      x: 0.0
      y: 0.0
      z: 0.0
    position:
      x: 0.49250367283821106
      y: 0.08392732590436935
      z: 0.7257388234138489
  place_pose:
    orientation:
      w: 0.0
      x: 0.0
      y: 0.0
      z: 0.0
    position:
      x: -0.1
      y: 0.71
      z: 0.605
  test_scene_num: 3
- arm_name: right
  object_name: snacks
  pick_pose:
    orientation:
      w: 0.0
      x: 0.0
      y: 0.0
      z: 0.0
    position:
      x: 0.42306026816368103
      y: -0.32529428601264954
      z: 0.7492565512657166
  place_pose:
    orientation:
      w: 0.0
      x: 0.0
      y: 0.0
      z: 0.0
    position:
      x: -0.1
      y: -0.71
      z: 0.605
  test_scene_num: 3
- arm_name: left
  object_name: eraser
  pick_pose:
    orientation:
      w: 0.0
      x: 0.0
      y: 0.0
      z: 0.0
    position:
      x: 0.5897160768508911
      y: -0.22023411095142365
      z: 0.7037498950958252
  place_pose:
    orientation:
      w: 0.0
      x: 0.0
      y: 0.0
      z: 0.0
    position:
      x: -0.1
      y: 0.71
      z: 0.605
  test_scene_num: 3
- arm_name: left
  object_name: eraser
  pick_pose:
    orientation:
      w: 0.0
      x: 0.0
      y: 0.0
      z: 0.0
    position:
      x: 0.6022871732711792
      y: 0.2850668430328369
      z: 0.6450653076171875
  place_pose:
    orientation:
      w: 0.0
      x: 0.0
      y: 0.0
      z: 0.0
    position:
      x: -0.1
      y: 0.71
      z: 0.605
  test_scene_num: 3
- arm_name: right
  object_name: soap2
  pick_pose:
    orientation:
      w: 0.0
      x: 0.0
      y: 0.0
      z: 0.0
    position:
      x: 0.4538024961948395
      y: -0.043528828769922256
      z: 0.6739063262939453
  place_pose:
    orientation:
      w: 0.0
      x: 0.0
      y: 0.0
      z: 0.0
    position:
      x: -0.1
      y: -0.71
      z: 0.605
  test_scene_num: 3
- arm_name: right
  object_name: soap
  pick_pose:
    orientation:
      w: 0.0
      x: 0.0
      y: 0.0
      z: 0.0
    position:
      x: 0.6758374571800232
      y: 0.006406047847121954
      z: 0.6731306910514832
  place_pose:
    orientation:
      w: 0.0
      x: 0.0
      y: 0.0
      z: 0.0
    position:
      x: -0.1
      y: -0.71
      z: 0.605
  test_scene_num: 3
- arm_name: left
  object_name: glue
  pick_pose:
    orientation:
      w: 0.0
      x: 0.0
      y: 0.0
      z: 0.0
    position:
      x: 0.6097415685653687
      y: 0.14362695813179016
      z: 0.6717910170555115
  place_pose:
    orientation:
      w: 0.0
      x: 0.0
      y: 0.0
      z: 0.0
    position:
      x: -0.1
      y: 0.71
      z: 0.605
  test_scene_num: 3
  ```

### Next Steps

Spend some time at the end to discuss your code, what techniques you used, what worked and why, where the implementation might fail and how you might improve it if you were going to pursue this project further.  



