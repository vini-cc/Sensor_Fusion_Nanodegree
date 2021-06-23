# 3D_Object_Tracking

## Prerequisites/Dependencies

- cmake >= 2.8
- make >= 4.1 (Linux, Mac), 3.81 (Windows)
- Git LFS
- OpenCV >= 4.1
  - This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
- gcc/g++ >= 5.4
  - Linux: gcc / g++ is installed by default on most Linux distros


## Setup

- Clone
```
git clone https://github.com/vini-cc/Sensor_Fusion_Nanodegree.git
```
- Go to the project 3D_Object_Tracking
```
cd Sensor_Fusion_Nanodegree/3D_Object_Tracking
```
- create build folder
```
mkdir build && cd build
```
- Compile
```
cmake .. && make
```
- Run '3D_object_tracking'
```
./3D_object_tracking
```

- Download `yolov3.weights`. Due to github space, the real one is not properly installed on the package.
```
wget https://pjreddie.com/media/files/yolov3.weights
```




## FP0 - Writeup

### FP1 - Match 3D Obstacles

The method is implemented in "matchBoundingBoxes", taking as input previous and current data frames, and as output, return the ROI Identification (boxID).

### FP2 - Compute Lidar-based TTC

Implemented in "computeTTCLidar", the function takes as input both previous and current lidar pointcloud and estimates the Time to Colision (TTC) for all matched points based on a relation between two consecutive frames, giving the output in seconds.

### FP3 - Associate Keypoint Correspondences with Bounding Boxes

Implemented in "clusterKptMatchesWithROI", it takes as input keypoints of previous and current frames, as well as matched points and the bounding boxes. The objective of this function is to create a startup for Time to Colision (TTC) using camera. So, to make it happen, the keypoints will be compared to bounding boxes, creating a correspondence. As a plus, an Euclidean function was implemented to avoid outliers and avoid not-corresponding points.

### FP4 - Compute Camera-based TTC

Implemented in "computeTTCCamera", it will compute and estimate the Time to Colision (TTC) using the matched keypoints from the bounding boxes, relating always two frames: previous and current.

### FP5 - Performance Evaluation 1

Well, it's hard to say exactly the reason, but I believe there is a major reason that could be bigger than anyone. Lidars are really noisy sensors, and due to that, they can create some not desirable outliers.

![img1](./images/img/part1.png?raw=true)
![img2](./images/img/part6.png?raw=true)
![img3](./images/img/part4.png?raw=true)

Looking from a top-view, it's possible to see on images above that in some situations, Lidar presents noisy results with aleatory uncertainty, being not so desirable to define small distances considering a single frame, and if our program runs looking to each point to find the closest one to define time, it will create the false idea of distance and TTC.
But this is not the unique problem. The other one is that we implemented a bounding box, considering proximity of points, and due to that, we cannot see the real outliers in the situation. To solve it, an Euclidean technique to eliminate outliers were implemented and solve the problem, but could be RANSAC or just a low-pass filter with a good calibration as well.



### FP6 - Performance Evaluation 2

![img3](./images/img/part2.png?raw=true)

To start FP6, please take a look at the doc "combinations.csv".

Considering the data extracted from the code, a few considerations:
- Some combinations of the code resulted in some problems to run (core dumped), and due to that, just FAST will be used as detector, and just SURF and SIFT descriptors were discarded due to lack of informations, resulting in Core dump!

After these considerations, the top 3 detector are:
- FAST + BRISK
- FAST + FREAK
- FAST + ORB

The points considered to the conclusion are simple:
- The reason why just FAST detector were utilized is because he is the best one in relation speed x keypoints. Other detectors were implemented, but it resulted in segmentation fault (core dump again).
- These combinations showed not so robust TTC results, with few NaN and crashed results. But comparing with other detectors, it's the best deal.
- There are other options to run the same code, just changing Euclidean function to another one you like. Fork the code and try it by yourself. I would be glad to share with you my opinion and analysis :)

Hope you enjoyed the ride. Please feel free to contact me if you have any doubt.
