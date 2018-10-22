# Extended Kalman Filter

The goals / steps of this project are the following:
* Follow the taugh Extended Kalman Filter algorithm to build a C++ based project
* Test the created Extended Kalman Filter with simulated Lidar and Radar dataset.
* The position & verlosity vector output coordinates [px, py, vx, vy ] must have an root-mean-square error, RMSE <= [.11, .11, 0.52, 0.52] 



## Algorithm Flow
![alt text](https://github.com/wincle626/Udacity_Term2_Projects/blob/master/project01/img/flowchat.png)

### 1. Initialize the state and covariance matrices.

### 2. Predict the state.

### 3. Update the state and covariance matrices.



## Simulation

### 1. Files.
        ./CmakeLists.txt
          |__./cmake/FindEigen.cmake
          |__.src/FusionEKF.cpp
          |__.src/FusionEKF.hpp
          |__.src/json.hpp
          |__.src/kalman_filter.cpp
          |__.src/kalman_filter.hpp
          |__.src/measurement_package.hpp
          |__.src/tools.cpp
          |__.src/tools.hpp
          |__.src/main.cpp

### 2. Build the project
        cd build
        cmake ..
        make -j2

### 3. Connect the simulator.
        Dataset1:      
![alt text](https://github.com/wincle626/Udacity_Term2_Projects/blob/master/project01/img/dataset1.png)
![alt text](https://github.com/wincle626/Udacity_Term2_Projects/blob/master/project01/img/video.gif)

        Dataset2:      
![alt text](https://github.com/wincle626/Udacity_Term2_Projects/blob/master/project01/img/dataset2.png)
![alt text](https://github.com/wincle626/Udacity_Term2_Projects/blob/master/project01/img/video2.gif)
