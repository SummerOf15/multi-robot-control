# multiple drones and robots control

We implement a project using multiple drones to detect persons and sending the robots to identify them.

## Result

In best performance, the drone can detect **7/8** persons and the husky robot can identify **6/8** persons.

In test session, we achieved **3/8** identification and **7/8** detection.

## Example

[youtube](https://youtu.be/oOq0Vl2AG2k)

![image-20210310204020292](ReadMe.assets/image-20210310204020292.png)

## Structure

![image-20210310211001261](ReadMe.assets/image-20210310211001261.png)

### Drone navigation map

![image-20210310204321341](ReadMe.assets/image-20210310204321341.png)

### Husky robot predefined map

![image-20210310212810709](ReadMe.assets/image-20210310212810709.png)

### **Control of husky**

![img](https://lh5.googleusercontent.com/dwkex556HbyEIyJUQsBmDwJNUM6NDegZGRDeOstAFG-BH5hqyaZkzgmfN2oLIwhXp7JmrWNYs5CN_auD2LJHUyMKRYUyikMrRpB7Zpd5YkAp5EQ5XAq-z6b6b7jphnd34zHp049b)

When the drones detect the persons, they will send the person’s position to the nearest husky robot. The robots plan a general path to goal with A* algorithm. As the robots travel along the path, there will inevitably be some obstacles. With an obstacle avoidance algorithm, the robot will avoid the obstacles. 

### **Avoidance of obstacle:**

![img](https://lh6.googleusercontent.com/T3j6JALjm2wzvVF5Gv6bjKNU4fyb-1U_Me4wrtP1WFFdr0OBN2avwDqX8pCI_jpcEKnhToZapeDXMJ8jYCZQuYOSsLq0cd15I_oFw7W4bRLtL-Tsk3kEqtpt6f1uSqztFtwsPUgr)

With the Kinect (RGB-D camera), we can have the depth image. In this image, there are two parts: the white one is the ground and the black one is its perspective if there is no obstacle. We focus on the junction of these two parts. When the middle area of this junction is all black, the robot steps forward, otherwise it turns left or right according to black area in the junction. If the whole depth image is black (too near to obstacle), the robot goes back. This is the local avoidance of obstacles.

## Usage

1. build the simulated environment
2. copy the folder `multi-robot` to replace the original one
3. launch the environment
4. launch the control algorithms
```bash
roslaunch multi_robot multi_robot_control.launch
```

## Problems

1. Sometimes the husky robot is blocked by the telephone poles. 
2. The kill zones are not well considered for drone navigation.

## Author

- Kai ZHANG
- Mengyu PAN
- Yan CHEN
- Yukun LIU

