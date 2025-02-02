Quadcopters are underactuated systems, meaning they can't be controlled using a single controller. Manual flight only requires an attitude controller but autonomous flight requires a position controller as well. 

In this document we define GUDs control architecture

![[Screenshot 2025-02-02 at 16.29.19.png]]
# Position control framework

Desired position information is inputted from a user with a remote controller or from a path planning algorithm. The actual position information is inputted from the EKF. The difference between these is the position error. The position controller produces the desired yaw, pitch and roll angles according to the position error which are treated as inputs to the attitude controller. 

# Attitude control framework

Actual attitude information is extracted from the EKF, and desired attitude information is provided from the position error outputted by the position controller. The error between these two is considered as attitude error and is fed into the attitude PID controller. 

The PID controller then generates the thrust which should be produced by each of the quadcopters motors. 

# Derivation

## Yaw, pitch and roll angle control

If we have $x_{d}, y_{d}$ (desired horizontal position) and $x_{a}, y_{a}$ (actual horizontal position), then position errors in $x$ and $y$ are:
$$
\begin{align*}
x_{e} = x_{d} - x_{a} \\
y_{e} = y_{d} - y_{a}
\end{align*}
$$
If position control gains for both $x$ and $y$ axes are $P_{xp}$ and $P_{yp}$ then the desired velocities $\dot{x}_{d}, \dot{y}_{d}$ can be written as
$$
\begin{align*}
\dot{x}_{d} = P_{xp} \ x_{e} \\
\dot{y}_{d} = P_{yp} \ y_{e}
\end{align*}
$$
By position control gain, we mean a tunable factor that determines how aggressively the controller responds to position error.

Thus velocity errors become:
$$
\begin{align*}
\dot{x}_{e} &= \dot{x}_{d} - \dot{x}_{a} \\
&= P_{xp} \ x_{e} - \dot{x}_{a} \\
\\
\dot{y}_{e} &= \dot{y}_{d} - \dot{y}_{a} \\
&= P_{yp} \ y_{e} - \dot{y}_{a}
\end{align*}
$$

Now, if velocity proportional gains are $P_{xv}$ and $P_{yv}$ and integral gains are $I_{xv}$ and $I_{yv}$ then control errors for the $x$, $y$ axes can be written as:
$$
\begin{align*}
P_{error \ x} &= P_{xv} \ \dot{x}_{e} \\
\\
I_{error \ x} &= I_{xv}  \int \dot{x}_{e} \\
\\
e_{x} &= P_{error \ x} + I_{error \ x}
\end{align*}
$$
i.e. velocity proportional gains scale the velocity error to generate a control signals thus determining how aggressively the controller reacts to velocity errors. 

integral gains account for the accumulated velocity error over time. They help reduce long-term drift and corrects small persistent errors that the proportional term alone might not fix

The control error is thus a sum of these, i.e. a proportional gain term to provide immediate correction based on the current velocity error and an integral gain to correct long-term accumulated errors and ensure stability. 

The same holds for the $y$ axis:
$$
\begin{align*}
P_{error \ y} &= P_{yv} \ \dot{y}_{e} \\
\\
I_{error \ y} &= I_{yv}  \int \dot{y}_{e} \\
\\
e_{y} &= P_{error \ y} + I_{error \ y}
\end{align*}
$$

If the initial heading angle of the drone is $\theta$ then the earth frame control errors $e_{x}$ , $e_{y}$ can be converted to the body frame desired roll and pitch angles as follows:
$$
\begin{align*}
d_{p} &= e_{x} \cos(\theta) + e_{y} \sin(\theta) \\
\\
d_{r} &= - e_{x} \sin(\theta) + e_{y} \cos(\theta)
\end{align*}
$$
i.e.
$$
\begin{bmatrix}
d_{p} \\
d_{r}
\end{bmatrix}
= 
\begin{bmatrix}
\cos \theta & \sin \theta \\
-\sin \theta & \cos \theta
\end{bmatrix}
\begin{bmatrix}
e_{x} \\
e_{y}
\end{bmatrix}
$$
since we are converting from the global frame to the local frame

We define a desired yaw angle $d_{y}$ based on user input or a path planning algorithm 

Using the horizontal position controller desired outputs roll, pitch and yaw, the desired rotation matrix can be obtained:
$$
R_{d_{1}} = \begin{bmatrix}
1 & 0 & 0  \\
0  & \cos(d_{p})  &  \sin(d_{p}) \\
0  & \sin(d_{p})  & \cos(d_{p})
\end{bmatrix}
$$
$$
R_{d_{2}} = \begin{bmatrix}
\cos(d_{r}) & 0 & \sin(d_{r})  \\
0  & 1  &  0 \\
- \sin(d_{r})  & 0  & \cos(d_{r})
\end{bmatrix}
$$
$$
R_{d_{3}} = \begin{bmatrix}
\cos(d_{y}) & -\sin(d_{y}) & 0  \\
\sin(d_{y})  & \cos(d_{y})  &  0 \\
0  & 0  & 1
\end{bmatrix}
$$
$$
R_{d} = (R_{d_{1}},R_{d_{2}},R_{d_{3}} )
$$

We thus define the attitude error matrix ($R_{e}$) and actual rotation matrix $R_{a}$ (describes the drones real orientation) and the body frame reference attitude error $e_{R}$:
$$
\begin{align*}
R_{e} &= R_{d}^T R_{a} - (R_{d}^T R_{a}) ^T \\
\\
e_{R} &= \begin{bmatrix}
R_{e}(2, 3) \\
R_{e}(1, 3) \\
R_{e}(1, 2)
\end{bmatrix}
\end{align*}
$$
The subtraction in $R_{e}$ results in a skew-symmetric matrix which is a common way of representing rotational errors. This is why in $e_{R}$ we index $R_{e}$. Each indexing accesses the elements proportional to the rotational errors about the $x, y \text{ and }z$ axes

These errors are then fed into the attitude controller to generate corrective torques. 
$$
\text{Torque output} = P \ e_{R} + I \int e_{R} + D \ \frac{d}{dt} e_{R}
$$

## Throttle control

Now, if we have $z_{d}, z_{a}$ (desired and actual vertical position), then position error in $z$ is:
$$
z_{e} = z_{d} - z_{a}
$$
If we have position control gain for the $z$ axis as $P_{zp}$ then the desired velocity is 
$$
\dot{z}_{d} = P_{zp} \ z_{e}
$$
And thus we have the vertical velocity error as:
$$
\dot{z}_{e} = \dot{z}_{d} - \dot{z}_{a}
$$

With velocity control gain for the $z$ axis as $P_{zv}$ then the desired acceleration is:
$$
\ddot{z}_{d} = P_{zv} \ \ddot{z}_{e}
$$
and if acceleration proportional gain is $P_{za}$ and integral gain is $I_{za}$ then control error for the $z$ axis $e_{z}$ is:
$$
\begin{align*}
P_{error z} &= P_{za} \ \ddot{z}_{e} \\
\\
I_{error z} &= I_{za}  \int \ddot{z}_{e} \\
\\
e_{z} &= P_{error \ z} + I_{error \ z}
\end{align*}
$$

Using the control error and a user defined hover throttle value $d_{H}$ (baseline thrust needed to stay in place), a desired throttle $d_{T}$ is defined
$$
\begin{align*}
d_{T} &= d_{H} + e_{z} \\
\text{Throttle ouput}&= d_{H} + P_{za} \ \ddot{z}_{e} + I_{za} \int \ddot{z}_{e} 
\end{align*}
$$


Thus, roll and pitch corrections align the drone with the desired $x$ and $y$ positions. The desired yaw angle is included in the attitude control and automatically corrected. Throttle control adjusts the drone vertically to align the drone with the desired $z$ position. 

Thus, we have a PI for throttle control, increases or decreasing all motor speeds equally. This doesn't result in an orientation change but in an altitude change. 

Then we have a PID for torque control. This adjusts individual motor speeds differently to create rotational forces (torques). e.g. to roll right we would want to increase the speeds of the motors on the left and decrease the speed of those on the right. Pitching forward or backward means increasing or decreasing the speed of the front and rear motors. Yaw control works differently. In standard quadcopters, two motors spin clockwise and two spin counterclockwise. Yaw control involves changing the speed of the clockwise and counterclockwise motors. 


# Motor mixing equations

Once we have total thrust and torque outputs, we need to map these to the motors as described above. 

Consider the following X-quadrotor configuration:

Then:
$$
\begin{align*}
\text{Motor 1 PWM} &= \text{thrust - roll + pitch + yaw} \\
\text{Motor 2 PWM} &= \text{thrust + roll - pitch + yaw} \\
\text{Motor 3 PWM} &= \text{thrust + roll + pitch - yaw} \\
\text{Motor 4 PWM} &= \text{thrust - roll - pitch - yaw} \\
\\
\begin{bmatrix}
\text{Motor 1 PWM} \\
\text{Motor 2 PWM} \\
\text{Motor 3 PWM} \\
\text{Motor 4 PWM}
\end{bmatrix} &= 
\begin{bmatrix}
1 & -1 & 1 & 1 \\
1 & 1 & -1 & 1 \\
1 & 1 & 1 & -1 \\
1 & -1 & -1 & -1
\end{bmatrix}
\begin{bmatrix}
\text{thrust} \\
\text{roll} \\
\text{pitch} \\
\text{yaw}
\end{bmatrix}
\end{align*}
$$

The matrix in the middle is called a mixer table. It is an $N$ by 4 matrix where $N$ is the number of motors. 

We can derive it as follows:
![[Screenshot 2025-02-02 at 15.25.13.png]]

By the principle of superposition we sum the forces and torques:
$$
\begin{align*}
T &= F_{1} + F_{2} + F_{3} + F_{4} \\
\tau_{x} &= - LF_{1} + LF_{2} + LF_{3} - LF_{4} \\
\tau_{y} &= LF_{1} - LF_{2} + LF_{3} - LF_{4} \\
\tau_{z} &= \gamma F_{1} + \gamma F_{2} - \gamma F_{3} - \gamma F_{4}
\end{align*}
$$
Where $\gamma$ is the rotor drag to thrust coefficient

In matrix form
$$
\begin{align*}
\begin{bmatrix}
T \\
\tau_{x} \\
\tau_{y} \\
\tau_{z}
\end{bmatrix} &= 
\begin{bmatrix}
1 & 1 & 1 & 1 \\
-L & L & L & -L \\
L & -L & L & -L \\
\gamma & \gamma & -\gamma & -\gamma
\end{bmatrix} 
\begin{bmatrix}
F_{1} \\
F_{2} \\
F_{3} \\
F_{4}
\end{bmatrix}  \\
\\
\begin{bmatrix}
F_{1} \\
F_{2} \\
F_{3} \\
F_{4}
\end{bmatrix} &= 
\begin{bmatrix}
1 & 1 & 1 & 1 \\
-L & L & L & -L \\
L & -L & L & -L \\
\gamma & \gamma & -\gamma & -\gamma
\end{bmatrix} ^{-1}
\begin{bmatrix}
T \\
\tau_{x} \\
\tau_{y} \\
\tau_{z}
\end{bmatrix} \\
\\
\begin{bmatrix}
F_{1} \\
F_{2} \\
F_{3} \\
F_{4}
\end{bmatrix} &= 
\left( 
\begin{bmatrix}
1  & 0  & 0 & 0 \\
0 & L & 0 & 0 \\
0 & 0 & L & 0 \\
0 & 0 & 0 & \gamma
\end{bmatrix}
\begin{bmatrix}
1 & 1 & 1 & 1 \\
-1 & 1 & 1 & -1 \\
1 & -1 & 1 & -1 \\
1 & 1 & -1 & -1
\end{bmatrix} ^{-1}  \right)
\begin{bmatrix}
T \\
\tau_{x} \\
\tau_{y} \\
\tau_{z}
\end{bmatrix} \\
\\
\begin{bmatrix}
F_{1} \\
F_{2} \\
F_{3} \\
F_{4}
\end{bmatrix} &= 
\begin{bmatrix}
1 & -1 & 1 & 1 \\
1 & 1 & -1 & 1 \\
1 & 1 & 1 & -1 \\
1 & -1 & -1 & -1
\end{bmatrix}
\begin{bmatrix}
\frac{1}{4}  & 0  & 0 & 0 \\
0 & \frac{1}{4} & 0 & 0 \\
0 & 0 & \frac{1}{4} & 0 \\
0 & 0 & 0 & \frac{1}{4}
\end{bmatrix}
\begin{bmatrix}
1  & 0  & 0 & 0 \\
0 & \frac{1}{L} & 0 & 0 \\
0 & 0 & \frac{1}{L} & 0 \\
0 & 0 & 0 & \frac{1}{\gamma}
\end{bmatrix}
\begin{bmatrix}
T \\
\tau_{x} \\
\tau_{y} \\
\tau_{z}
\end{bmatrix} \\
\\
\begin{bmatrix}
F_{1} \\
F_{2} \\
F_{3} \\
F_{4}
\end{bmatrix} &= 
\begin{bmatrix}
1 & -1 & 1 & 1 \\
1 & 1 & -1 & 1 \\
1 & 1 & 1 & -1 \\
1 & -1 & -1 & -1
\end{bmatrix}
\begin{bmatrix}
\frac{1}{4}  & 0  & 0 & 0 \\
0 & \frac{1}{4L} & 0 & 0 \\
0 & 0 & \frac{1}{4L} & 0 \\
0 & 0 & 0 & \frac{1}{4\gamma}
\end{bmatrix}
\begin{bmatrix}
T \\
\tau_{x} \\
\tau_{y} \\
\tau_{z}
\end{bmatrix} \\
\end{align*} 
$$

If we introduce scaled torques and thrust:
$$
\begin{align*}
\hat{T} &= \frac{T}{4} \\
\hat{\tau}_{x} &= \frac{\tau_{x}}{4L} \\
\hat{\tau}_{y} &= \frac{\tau_{y}}{4L} \\
\hat{\tau}_{z} &= \frac{\tau_{z}}{4\gamma} \\
\end{align*}
$$
We then have the normalized mixer table:
$$
\begin{align*}
\begin{bmatrix}
F_{1} \\
F_{2} \\
F_{3} \\
F_{4}
\end{bmatrix} &= 
\begin{bmatrix}
1 & 1 & 1 & 1 \\
-1 & 1 & 1 & -1 \\
1 & -1 & 1 & -1 \\
1 & 1 & -1 & -1
\end{bmatrix} ^{-1}
\begin{bmatrix}
\hat{T} \\
\hat{\tau}_{x} \\
\hat{\tau}_{y} \\
\hat{\tau}_{z}
\end{bmatrix} \\
\\
\begin{bmatrix}
F_{1} \\
F_{2} \\
F_{3} \\
F_{4}
\end{bmatrix} &= 
\begin{bmatrix}
1 & -1 & 1 & 1 \\
1 & 1 & -1 & 1 \\
1 & 1 & 1 & -1 \\
1 & -1 & -1 & -1
\end{bmatrix}
\begin{bmatrix}
\hat{T} \\
\hat{\tau}_{x} \\
\hat{\tau}_{y} \\
\hat{\tau}_{z}
\end{bmatrix} \\
\end{align*}
$$


The output of the mixer is fed to the ESC as PWM duty. In our calculations the output of the mixer is force, whereas the ESC accepts rotational speed of the rotor. The force needs to be divided by the thrust coefficient and square rooted. 

The thrust force generated by a rotor is given by:
$$
F = C_{T} \cdot \omega ^2
$$
where $C_{T}$ is the thrust coefficient which depends on air density, blade shape, etc. and $\omega$ is the rotor speed in radians per second. 

Solving for rotor speed:
$$
\omega_{i} = \sqrt{ \frac{F_{i}}{C_{T}} }
$$

Thus each ESC input is:
$$
\begin{align*}
\begin{bmatrix}
F_{1} \\
F_{2} \\
F_{3} \\
F_{4}
\end{bmatrix} &= 
\begin{bmatrix}
1 & -1 & 1 & 1 \\
1 & 1 & -1 & 1 \\
1 & 1 & 1 & -1 \\
1 & -1 & -1 & -1
\end{bmatrix}
\begin{bmatrix}
\hat{T} \\
\hat{\tau}_{x} \\
\hat{\tau}_{y} \\
\hat{\tau}_{z}
\end{bmatrix} \\
\\
\omega_{i} &= \sqrt{ \frac{F_{i}}{C_{T}} } \ \  \forall \ i \in \{ 1,2,3,4 \}
\end{align*}
$$

# Considerations when using this framework

- $C_{T}$ must be experimentally determined or provided by the manufacturer
- 
