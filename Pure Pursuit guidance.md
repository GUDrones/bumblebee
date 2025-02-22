
# 2D pure pursuit

Consider a planar UAV-target engagement geometry: 

![[Screenshot 2025-02-20 at 18.09.26.png]]

where:
- $xy$ and $z$: horizontal plane and vertical axis 
- $u$ : UAV 
- $t$ :  target
- $V_{u}$ : UAV speed (m/s)
- $X_{u}$ : course angle
- $\gamma$ :  flight path angle
- $V_{t}$ : target speed (m/s)
- $X_{t}$ :  course angle
- $V_{u}^0$   : initial UAV  
- $V_{t}^0$ : target speed (m/s)
- $R_{xy}, \dot{R}_{xy}$ : line of sight (LOS) distance in $xy$ plane
- $R_{xy}^0$ : initial LOS distance in $xy$ plane
- $R_{z}, \dot{R}_{z}$ : vertical distance between UAV and target
- $R$ : LOS distance between UAV and target
- $\theta, \dot{\theta}$ : LOS angle
- $\phi, \dot{\phi}$ : projection of LOS angle on $xy$ plane
- $a_{xy},  \ a_{z}$ : guidance commands generated in 3D space

All distances are in meters and angles in radians. First order derivatives represent the rate of change w.r.t time

So, we have that the UAV (U) has velocity $V_{u}$ with course angle $X_{u}$ and the target (T) has velocity $V_{t}$ with a course angle $X_{t}$. 

Pure Pursuit works on the principle, that if the tracking vehicle points towards the target $(X_{u} \rightarrow \phi, \ \dot{X}_{u} = \dot{\phi})$ then it will ultimately intercept it. The pursuit condition ensures that the UAV continuously adjusts its heading toward the target. 

The target is assumed to be non-manoeuvring ($V_{t} \ \text{ and } \ X_{t}$ are constant).

The projection of the target velocity $V_{t}$ along the LOS is:
$$
V_{t} \cos(X_{t} - \phi)
$$
while the projection of the UAV velocity $V_{u}$ along the LOS is;
$$
V_{u} \cos(X_{u} - \phi)
$$
If the difference between these projections is below 0, then the UAV is closing in on the target. Thus, we can define this difference as the rate of change of $R_{xy}$ along the LOS:
$$
\dot{R}_{xy} = V_{t}\cos(X_{t} - \phi) - V_{u} \cos(X_{u} - \phi)
$$

We then define the component of the target's velocity perpendicular to the LOS:
$$
V_{t} \sin(X_{t} - \phi)
$$
The component of the UAV's velocity perpendicular to the LOS:
$$
V_{u}\sin(X_{u} - \phi)
$$
Consider an object moving perpendicularly to a radial line at velocity $v_{\perp}$, we have that:
$$
\omega  = \frac{v_{\perp}}{r}
$$
$v_{\perp}$ is the difference in perpendicular velocity components and $r$ is $R_{xy}$

Thus, it follows that:
$$
\dot{\phi} = \frac{V_{t}\sin(X_{t} - \phi) - V_{u} \sin(X_{u} - \phi)}{R_{xy}}
$$

The guidance command $(a_{xy})$ is the lateral acceleration applied normal to the velocity vector and is related to $\dot{X}_{u}$ as,
$$
\dot{X}_{u} = \frac{a_{xy}}{V_{u}}
$$
This makes sense, for a 2D object in motion, a force applied along the velocity vector, changes only the speed, not the direction while a force applied perpendicular to the velocity vector changes only the direction, keeping the speed constant. 

The mathematical relationship also makes sense. The rate of change of the UAV's course angle is proportional to the applied lateral acceleration.

Pure pursuit says that the UAV should turn toward the target by matching the LOS angular rate:
$$
\begin{align*}
\dot{X}_{u} &= \dot{\phi} \\
\frac{a_{xy}}{V_{u}} &= \dot{\phi}\\
a_{xy} &= V_{u}  \dot{\phi} - K_{a} (X_{u} - \phi) \\
\end{align*}
$$
where:
- $K_{a} > 0$  is the gain


# 3D pure pursuit

Consider now a 3D engagement of UAV-target:

![[Screenshot 2025-02-22 at 16.21.02.png]]

We assume that the target is non-maneuvering and that it is in 3D space

The UAV must now not only adjust its course angle $X_{u}$ but also its path flight angle $\gamma$ to control altitude changes. 

So now the Line-of-Sight (LOS) has both:
- a horizontal projection $\phi$
- a vertical projection $\theta$

The expressions for the UAV-target distance are given as:
$$
\begin{align*}
R &= \sqrt{ (x_{t} - x_{u})^2 + (y_{t} - y_{u})^2 + (z_{t}-z_{u})^2 } \\
R_{xy} &= \sqrt{ (x_{t} - x_{u})^2 + (y_{t} - y_{u})^2 } \\
R_{z} &= \sqrt{  (z_{t}-z_{u})^2 } \\
\end{align*}
$$
The flight path angles are given as:
$$
\begin{align*}
\phi &= \tan^{-1} \left( \frac{y_{t} - y_{u}}{x_{t} - x_{u}} \right) \\

\theta &= \tan^{-1} \left( \frac{z_{t} - z_{u}}{R_{xy}} \right)
\end{align*}
$$

Expressions of $\dot{R}_{xy}$ are derived similar to the 2D case:

The change in $R_{xy}$ is the difference in the projections of $V_{t}$ and $V_{u}$ on the XY-plane component of the LOS. If this difference is smaller than zero, we are approaching the target on the xy plane. i.e. if the UAVs velocity vector projection on the XY-plane component of the LOS is larger than the targets then we are approaching the target on the XY-plane.
$$
\begin{align*}
\dot{R}_{xy} &= V_{t} \cos(\gamma_{t}) \cos(X_{t} - \phi) - V_{u} \cos(\gamma_{u}) \cos(X_{u} - \phi) \\

\end{align*}
$$


Consider the case where the UAV is above the target and needs to descend. Then:

The change in $R_{z}$ is the difference in the perpendicular components of $V_{t}$ and $V_{u}$ on the XY-plane component of the LOS. If the UAV's velocity is more perpendicular than the target's velocity then 
$$
\dot{R}_{z} = V_{u} \sin(\gamma_{u}) - V_{t} \sin(\gamma_{t})
$$

Now consider now the case where the target is above the UAV. Then, the opposite is true:
$$
\dot{R}_{z} = V_{t} \sin(\gamma_{t}) -  V_{u} \sin(\gamma_{u}) 
$$

We can generalize this as:
$$
\begin{align*}
\dot{R}_{z} &= sgn(z_{t} - z_{u}) \ (V_{t} \sin(\gamma_{t}) -  V_{u} \sin(\gamma_{u}) ) \\
\\
sgn(z_{t} - z_{u}) &= \begin{cases}
1, & \text{if target is above UAV} \\
-1, & \text{if target is below UAV} 
\end{cases}
\end{align*}
$$

Consider now this diagram:

Consider the variation of the LOS Azimuth angle, the angle between the LOS projection on the XY plane and the X-azis. It describes how the LOS rotates in the horizontal plane. 

We can imagine varying it by moving the target perpendicularly to the LOS in the XY-plane or moving the UAV perpendicularly to the LOS in the XY plane. 

How do we describe a movement perpendicular to the LOS in the XY-plane from either the UAV or target?

$$
V_{u} \cos(\gamma_{u})
$$
extracts the projection of $V_{u}$ on the XY plane. Then we need to find the component of this velocity that is perpendicular to the XY-component of the LOS:
$$
V_{u} \cos(\gamma_{u})\sin(X_{u} - \phi)
$$

Imagine the target and UAV moving perpendicularly simultaneously. If the target moves faster, the angle increases. If the UAV moves faster, the angle decreases. If both move perpendicularly at the same time, then the angle doesn't change.

Thus we can see that:
$$
\begin{align*}
\dot{\phi} &= \frac{V_{t} \cos(\gamma_{t}) \sin(X_{t}-\phi) - V_{u} \cos(\gamma_{u}) \sin(X_{u} - \phi)}{R_{xy}} \\
\end{align*}
$$
The emergence of the denominator makes sense. The further the UAV is from the target, the slower the LOS rotates. Imagine holding a string which is held in place and moving perpendicularly. The further away you are from the point where it is held, the slower it rotates. 

Now consider the variation of the elevation angle. 

The target and UAV moving up and the horizontal rate of closure $\dot{R}_{xy}$ can result in changes to the elevation angle. 

To consider vertical target and UAV movement we need to extract the vertical components of the velocities. 
$$
V_{u} \sin(\gamma_{u})
$$
extracts the vertical component of $V_{u}$

Consider the case where $\theta = \frac{\pi}{2}$, meaning the UAV and target are on top of each other. In this case, vertical movement will not affect the elevation angle. To model this we use
$$
V_{u}\sin(\gamma_{u})\cos(\theta) 
$$
to extract how much the vertical component of the velocity impacts the LOS.

To consider the effect of horizontal motion on $\theta$ we consider the component of the horizontal closure rate that contributes to vertical LOS change:
$$
\dot{R}_{xy} \sin(\theta)
$$
Thus, we arrive at a formula for the elevation angle rate:
$$
\dot{\theta} = \frac{(V_{t}\sin(\gamma_{t}) \cos(\theta)-V_{u} \sin(\gamma_{u})\cos(\theta) - \dot{R}_{xy} \sin(\theta))}{R}
$$
However, again, this will depend on whether the UAV is above the target or not, so we have:
$$
\dot{\theta} = \frac{ sgn(z_{t} - z_{u}) (V_{t}\sin(\gamma_{t}) \cos(\theta)-V_{u} \sin(\gamma_{u})\cos(\theta)) - \dot{R}_{xy} \sin(\theta)}{R}
$$
The emergence of the denominator makes sense. The further the UAV is from the target, the slower the LOS steepens given as a result of vertical velocity. 


The guidance commands naturally emerge from the 2D version. The guidance command $a_{xy}$ is the lateral acceleration applied normal to the projection of the velocity vector on the XY plane, and is related to $\dot{X}_{u}$ as:
$$
\dot{X}_{u} = \frac{a_{xy}}{V_{u}  \cos(\gamma_{u})}
$$

The guidance command $a_{z}$ is the vertical acceleration applied normal to the projection of the velocity vector on the XY plane and is related to $\dot{\gamma_{u}}$ as:
$$
\dot{\gamma}_{u} = \frac{a_{z}}{V_{u}}
$$

Pure pursuit says that the UAV should turn toward the target by matching the LOS angular rate. 

First we consider the horizontal LOS angular rate. Similarly to the 2D case, we say that:
$$
\begin{align*}
\dot{X}_{u} &= \dot{\phi} \\
\frac{a_{xy}}{V_{u} \cos(\gamma_{u})} &= \dot{\phi} \\
a_{xy} &= V_{u} \cos(\gamma_{u}) \dot{\phi}
 - K_{a} (X_{u} - \phi)
\end{align*}
$$

For the vertical LOS angular rate:
$$
\begin{align*}
\dot{\gamma_{u}} &= \dot{\theta} \\
\frac{a_{z}}{V_{u}} &= \dot{\theta} \\
a_{z} &= V_{u} \dot{\theta} - K_{a}(\gamma_{u} - \theta)
\end{align*}
$$



## 3D pure pursuit for landing

Consider now the following 3D engagement geometry for landing

![[Screenshot 2025-02-20 at 23.00.38.png]]

In this geometry, we have the target moving on the ground in XY plane and the UAV in 3D space above it. This is a simplification of the problem we described above. 

The formulas for $R, R_{xy}, R_{z}, \phi, \theta$ remain the same.

The formula for $\dot{R}_{xy}$ is simplified since we don't have to project the target velocity onto the XY-plane as from the problem statement this is already defined to be on the XY-plane- 

So: 
$$
\begin{align*}
\dot{R}_{xy} &= V_{t}\cos(X_{t} - \phi) - V_{u} \cos(\gamma_{u}) \cos(X_{u} - \phi) \\
\end{align*}
$$
The formula for $\dot{R}_{z}$ is also simplified. Since the target's movement is restricted to the XY plane on the ground, it can't impact $R_{z}$. Moreover we know that the UAV is strictly above the target thus:
$$
\dot{R}_{z} = V_{u} \sin(\gamma_{u})
$$

The formula for the horizontal LOS projection angle rate of change is also simplified since the target's velocity is already on the XY plane and doesn't have to be projected on:
$$
\begin{align*}
\dot{\phi} &= \frac{V_{t}  \sin(X_{t}-\phi) - V_{u} \cos(\gamma_{u}) \sin(X_{u} - \phi)}{R_{xy}} \\
\end{align*}
$$
The formula for the vertical LOS projection angle rate of change is also simplified since the target's movement can't impact $\phi$ and we know for sure that:
$$
\dot{\theta} = \frac{-(V_{u} \sin(\gamma_{u})\cos(\theta)) + \dot{R}_{xy} \sin(\theta))}{R}
$$

Then for the guidance commands, the formulas for $a_{xy}, \ a_z$ stay the same
