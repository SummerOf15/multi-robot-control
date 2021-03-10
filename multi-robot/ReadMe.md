# multiple drones and robots control

We implement a project using multiple drones to detect persons and sending the robots to identify them.

## Result

In best performance, the drone can detect **7/8** persons and the husky robot can identify **6/8** persons.

## Example

[youtube](https://youtu.be/J6GOlmmIfBc)

![image-20210310204020292](ReadMe.assets/image-20210310204020292.png)

## Structure

![image-20210310211001261](ReadMe.assets/image-20210310211001261.png)

### Drone navigation map

![image-20210310204321341](ReadMe.assets/image-20210310204321341.png)

### Husky robot predefined map

![image-20210310212810709](ReadMe.assets/image-20210310212810709.png)



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

