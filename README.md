MMAN3200, Term 1 2021 Subhan Khan, Jose Guivant

## Project 4 MPC part

# 1 Model Predictive Control

Consider the linearized model of an aircraft at altitude of 5000mand speed 128. 2 m/s:

```
x ̇=
```
### 

### 

### 

### 

### − 1. 28 0 0. 98 0

### 0 0 1 0

### − 5. 43 0 − 1 .84 0

### − 128. 2 − 128. 2 0 0

### 

### 

### 

### 

```
x+
```
### 

### 

### 

### 

### − 0. 3

### 0

### − 17

### 0

### 

### 

### 

### 

```
u, y=
```
### 

### 

### 

### 

### 1 0 0 0

### 0 1 0 0

### 0 0 1 0

### 0 0 0 1

### 

### 

### 

### 

```
x
```
wherex= [x 1 x 2 x 3 x 4 ] withx 1 [rad] is the angle of attack,x 2 [rad] is the pitch angle,x 3 [rad/s] is the
pitch rate, andx 4 [m] is the altitude. In addition,u[rad] is the elevator angle as shown in the Figure 8.
Perform the following tasks:

```
Figure 1: Aircraft control problem
```
Part A: Unconstrained MPC
For the aircraft control problem, you will first design an unconstrained MPC for a simulation time
of 10swith a sampling time of 0. 25 s. Consider the initial conditions to bex(0) = [0, 10 , 0 ,100],
y(0) = [0 0 0 5000] and the state and input penalty matrices areQ=diag(0, 1 , 0 ,1) andR= 10,
respectively. Comment on the results based on the following conditions:

1. What happens to the states and control action when the prediction horizonN= 10 andN= 5?
2. What happens to the states and control action if the initial conditions arex(0) = [10 0 100 0]

Part B: Constrained MPC
For the aircraft control problem, you will now design a constrained MPC for a simulation time
of 30swith a sampling time of 0. 25 s. Consider the initial conditions to bex(0) = [0, 10 , 0 ,100],
y(0) = [0, 0 , 0 ,5000] output input constraints are− 0. 262 rad≤u≤ 0. 262 rad, and the state and input
penalty matrices areQ=diag(0, 1 , 0 ,1) andR= 10, respectively. Comment on the results based on
the following conditions:

1. What happens to the states and control action when the prediction horizonN= 10 andN= 5?

### 1


MMAN3200, Term 1 2021 Subhan Khan, Jose Guivant

2. What happens to the states and control action if the initial conditions arex(0) = [10 0 100 0]

NOTE: You can set the values ofQandRto get the better response. Therefore, for this project,
you are allowed to use any value ofQandR. For both the parts, you are required to write a single page
report or a section in the code to comment your understanding from this project.

### 2


```
MTRN4010.2021 – Project # 4
```
```
Problem 2: Applying an optimizer for estimating position by triangulation
```
Given a set of 5 landmarks (which are always visible, and whose positions, in a given
coordinate frame, are known), and a sensor which provides range measurements
respect to all those landmarks, simultaneously, you are required to estimate the position
of the sensor by minimizing a cost function.
You will implement a Matlab function, named GetMySolutionXY, which will process the
5 range measurements associated to the 5 landmarks, in order to estimate the position
of the sensor. The position of the sensor must be expressed in the same coordinate
frame in which the positions of the landmarks are expressed.

The positions of the map’s landmarks and the measured ranges can be read, at any
time, from the global variable Data5.
Data5 is a structure that has three (3) fields:
Data5.ranges: The measured ranges (array of 5 values, in meters)
Data5.Lx: coordinates X of landmarks’ positions (array of 5 values, in meters)
Data5.Ly: coordinates Y of landmarks’ positions (array of 5 values, in meters)
The physical meaning of these variables is according to the following range
equation:

# ( ) ( )

```
22
```
## rxxyyjjj= − + −

```
In which
```
##### Data5.ranges( j )

##### Data5.Lx( j )

##### Data5.Ly( j )

```
j
j
j
```
##### r

##### x

##### y

##### =

##### =

##### =

Your function does not have arguments, since all the necessary data in contained in the
global variable Data5 (so that your program simply reads, from that global variable, the
necessary data). Your function must return the estimated 2D position, [x;y], as a 2x
vector.

function xy =GetMySolutionXY()
% This example is an empty function
% Do your calculations here.


xy=[0;0]; % you must calculate xy, the estimated _sensor’s_ position
end
Inside that function you will use an optimizer such as fminsearch, or a PSO solver.
Your function must return the estimated sensor’s position, by minimizing a cost function
of the following class:

####  

### ( ( ))

## ( ) ( ) ( )

```
*
```
(^522)
1
argmin
T
j jjj
xy
C
Crxxyy

=
=
=

### = − − + −

```
x x
```
```
x
xx
```
```
x
```
For testing your function, you may use the following program TestYourFunction

Which simulates the measurements, and then it calls your function providing to it the
simulated measurements and the map, via the global variable Data5. Then the program
uses the estimated position, which is returned by your function, for plotting it jointly with
the real position.
TestYourFunction always calls the function named GetMySolutionXY when it requires
the estimated position ( so your function must have that name, GetMySolutionXY.m)

TestYourFunction accepts one argument, for specifying the standard deviation of noise
being added to the simulated range measurements, to simulate polluted
measurements.
For instance, TestYourFunction(0.1) will result in the program adding Gaussian noise
of standard deviation =0.1 meter. If you want to simulate perfect measurements, you
must use TestYourFunction(0)

Inside your function GetMySolutionXY() you implement your code. You may use the
optimizer that you prefer, e.g. fminsearch() or the PSO optimizer.

This problem gives you 50% of Project4’s overall mark. The rest of the marks are given
by Problem 1, in which you implement an MPC controller (that problem was released on
week 8).

You will submit both programs, via Moodle. No report is required for this project. Your
solution for Problem 2 will be tested using TestYourFunction, in the same way you tested


it. We will verify that for the case TestYourFunction(0), the solution is almost at the real
position. The lecturer will show his solution in class.
Submission of this project Friday week 10, 11PM.
There is an automatic extension of one day. for the submission of this project (i.e. on
Saturday 24/April, 11PM). There is no penalty for using this extension.


