# Mobile-Robotics-Final-Project

## CPE 412/MAE 412 Mobile Robotics

> Department of Mechanical and Aerospace Engineering<br/>
> Benjamin M.Statler College of Engineering and Mineral Resources<br/>
> West Virginia University

### Final Submission Date : December 15, 2019

### Teammates : Mathew Bergenstock, Adam Hensley, Austin Moore, Hayden Nichols, Hyejun Park, William Smith

## Project

The overall project will be developing a fully autonomous robot that explores an area without colliding with obstacles or other robots that will perform simultaneously. The performance was held in AER 222 with many obstacles, competing with other robots to determine the most efficient robot when it comes to avoid obstacles and explore the area in given time.

## Result

Demo for 10 minutes each

### First Demo

- traveled total of `69 meters`
- total of `29 collisions`.

This collision total would be the highest out of all three tests for our robot due to the unexpected behavior at the start of the test.

### Second Demo

- traveled `72 meters`
- total of `18 collisions`

Testing area was increased in size while also reducing the stationary objects within the region.

### Final Demo

- traveled `73 meters`
- total of `25 collisions`.

For the third and final test area, a dramatic increase in test area obstacles was added while still holding the area from test two constant. The largest obstacle in the new area was several crates that divided the large area into two smaller sections while leaving a small corner open connecting them.

## What I engaged with

Initially, a bug in the code prevented the Vector Field Histogram (VFH) planner from controlling the wheels. As such, the reactive layer was the only functioning avoidance measure for the preliminary demo. The robot became trapped in small regions of the competition area as the reactive algorithm was triggered when the robot would try to move through small corridors and favored turning left abruptly. Improvements were made to the algorithm by fixing bugs in the VFH algorithm and removing the rigid logic behind the reactive layer.
