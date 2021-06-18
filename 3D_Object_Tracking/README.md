# 3D_Object_Tracking

## FP0 - Writeup

### FP1 - Match 3D Obstacles

The method is implemented in "matchBoundingBoxes", taking as input previous and current data frames, and as output, return the ROI Identification (boxID).

### FP2 - Compute Lidar-based TTC

Implemented in "computeTTCLidar", the function takes as input both previous and current lidar pointcloud and estimates the Time to Colision (TTC) for all matched points based on a relation between two consecutive frames, giving the output in seconds.

### FP3 - Associate Keypoint Correspondences with Bounding Boxes

Implemented in "clusterKptMatchesWithROI", it takes as input keypoints of previous and current frames, as well as matched points and the bounding boxes. The objective of this function is to create a startup for Time to Colision (TTC) using camera. So, to make it happen, the keypoints will be compared to bounding boxes, creating a correspondence.

### FP4 - Compute Camera-based TTC

Implemented in "computeTTCCamera", it will compute and estimate the Time to Colision (TTC) using the matched keypoints from the bounding boxes, relating always two frames: previous and current.

### FP5 - Performance Evaluation 1

Well, it's hard to say exactly the reason, but I believe there is a major reason that could be bigger than anyone. Lidars are really noisy sensors, and due to that, can create some not desirable outliers.

![img1](./images/img/part1.png?raw=true)
![img2](./img/part1.png?raw=true)
![img3](./img/part1.png?raw=true)

Looking from a top-view, it's possible to see above that, in some situations, Lidar presents noisy results, and if our program runs to each point, looking for the closest one to define time, it will create the false idea of distance and TTC.
But this is not the unique problem. The other one is that we implemented a bounding box, considering proximity of points, and due to that, we cannot see the real outliers in the situation. To solve it, a technique like RANSAC using least squares can solve the problem.



### FP6 - Performance Evaluation 2
