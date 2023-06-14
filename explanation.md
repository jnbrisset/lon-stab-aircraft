# Longitudinal aircraft control

## State system representation

The goal of state-space representation is to represent the system in a way that is convenient for analysis and design. The state-space representation is a set of first-order differential equations, which can be written in matrix form as follows:

$$
\dot{x} = Ax + Bu
$$

$$
y = Cx + Du
$$

Graphicaly, it looks like this:

![State space representation](https://upload.wikimedia.org/wikipedia/commons/thumb/e/eb/Typical_State_Space_model.svg/1200px-Typical_State_Space_model.svg.png)



Let's ignore $D$, as it's not significant in aircraft control.

$$
\dot{x} = A\vec x + B\vec u
$$

$$
y = C \vec x
$$

where $x$ is called the state vector. For longitudinal aircraft control, the state vector will have 4 state variables, yielding a system of 4 equations (1st order differential equations, to be precised):
- $u$: the horizontal speed along the longitudinal axis
- $w$: the vertical speed along the vertical axis pointing down
- $q$: the pitch rate
- $\theta$: the pitch angle

$Bu$ is the control input variable or vector (sorry for redundancy with the horiontal speed). We have two control inputs: elevator deflection and thrust. Initially, we are going to ignore thrust, and we will only consider elevator deflection. Therefore, $u$ is the elevator deflection, $\delta_e$.

$y$ is the output vector. For longitudinal aircraft control, classicly, the lower order output would be the pitch angle. As we briefly discussed in the answer on Facebook, it is the C* value. More generally, in control engineering, that's the value you want to control.

Pilots, we aren't used to think with longitudinal and vertical axis, in other words in the body frame of reference of the aircraft. $u$ and $w$ would be the velocities used by the IRS. There are simple relations in between the TAS and angle of attack, both of which are more intuitive to us, which are: 


$$
TAS = \sqrt{u^2 + w^2}
$$
$$
\alpha = \arctan{\frac{w}{u}}
$$
For small angle of attack, let's say 10°, both equations can be simplified to:
$$
TAS = u
$$
$$
\alpha = \frac{w}{u}
$$

The above will be important in building our understanding.

If you are not familiar with state-space representation, we take the derivative of each state variable, and we write it as a combination of the state variables. A derivative of a velocity ($u$ or $w$) is a linear acceleration, the derivative on an angular velocity ($q$) is an angular acceleration, and the derivative of the pitch angle is the pitch rate ($q)(assuming no bank, for now). So, our state equations are:

$$
\begin{bmatrix}
\dot u \\
\dot w \\
\dot q \\
\dot \theta
\end{bmatrix}=
\begin{bmatrix}
a_{11} & a_{12} & a_{13} & a_{14} \\
a_{21} & a_{22} & a_{23} & a_{24} \\
a_{31} & a_{32} & a_{33} & a_{34} \\
a_{41} & a_{42} & a_{43} & a_{44}
\end{bmatrix} \begin{bmatrix}
u \\
w \\
q \\
\theta
\end{bmatrix} + \begin{bmatrix}
b_{1}\\
b_{2}\\
b_{3}\\
b_{4}
\end{bmatrix} \delta_e
$$

Given that the $\dot \theta$ is the pitch rate, we can rewrite the equation this way:

$$
\begin{bmatrix}
\dot u \\
\dot w \\
\dot q \\
\dot \theta
\end{bmatrix}=
\begin{bmatrix}
a_{11} & a_{12} & a_{13} & a_{14} \\
a_{21} & a_{22} & a_{23} & a_{24} \\
a_{31} & a_{32} & a_{33} & a_{34} \\
0 & 0 & 1 & 0
\end{bmatrix} \begin{bmatrix}
u \\
w \\
q \\
\theta
\end{bmatrix} + \begin{bmatrix}
b_{1}\\
b_{2}\\
b_{3}\\
b_{4}
\end{bmatrix} \delta_e
$$

The longitudinal dynamic response of the aircraft to an input or disturbance will be exclusively determined by the 4x4 matrix. Now, if we look at the first three equations:
- $\dot u$ will be related to the drag equation
- $\dot w$ will be related to the lift equation
- $\dot q$ will be related to the pitch moment equation

To achieve a meaninful analysis, we linearize the problem, meaning that we are looking at **small deviation or disturbance around a point of equilibrium**. What is the effect of this linearization? Let's take an example for the lift equation, where $m$ is the mass of the aircraft:

$$
\dot w = \frac{1}{m} (a_{21}u + a_{22}w + a_{23}q + a_{24}\theta) + b_1 \delta_e
$$

We can simplify by saying that, recalling our simplification for small angle of attack:
- The coefficient $a_{21}$ is the variation of the lift when we vary the TAS. 
- The coefficient $a_{22}$ is the variation of the lift when we vary the angle of attack.
- The coefficient $a_{23}$ is the variation of the lift when we vary the pitch rate.
- The coefficient $a_{24}$ is the variation of the lift when we vary the pitch angle, which relates to gravity in that case and not aerodynamic forces.
- The coefficient $b_1$ is the variation of the lift when we make an elevator input.

We can do the same for $\dot u$ and $\dot q$. We can relate our 4x4 matrix to actual aerodynamic and physical forces. That's one step they do when they start working on control laws on a new aircraft. They start for calculations with data from wind tunnels and simulator, as typically it's not flying yet.

## Stability

I have stated that the whole longitudinal dynamics of the aircraft is included in the $A$ 4x4 matrix. If you had done linear algebra, eigenvalues would probably ring a bell. If you calculate the eigenvalues ($\lambda$) of that matrix, you will find two sets of complex numbers. Those two sets can written as 2 polynomials, it can be expressed as:

$$
\lambda^2 + 2 \zeta \omega_n \lambda + \omega_n^2
$$

Where $\zeta$ is the damping ratio, how quickly oscillation will dissipate, and $\omega_n$ is the natural frequency, how quickly it will oscillate. Given a system with 4 equations, we have 4 complex numbers, 2 sets of complex numbers, therefore 2 polynomials, which means **two modes of oscillation for the longitudinal dynamics of an aircraft**. There are called phugoid and short period oscillations. 

### Phugoid

The phugoid is a long period oscillation, with a period of 30 to 60 seconds. It's a slow oscillation, and it's a mode of oscillation that is not damped or very slightly damped. Given the period of an oscillation, it's easy for the pilot, the flight control computer or the autopilot to correct it. From a state of equilibrium, if we do a small pitch input, we will have a phugoid oscillation. The speed will decrease, the aircraft will pitch down and accelerate, pitching up its own. It's a wave or oscillation which continuously trade potential energy with kinetic energy.

### Short period

The short period oscillation has, as its name implies, a small oscillation period of a few seconds. It needs to be heavily damped, or else it will be very difficult for the pilot to control the aircraft.

Both of those oscillations will have their damping ratio ($\zeta$) and their natural frequency ($\omega_n$). Design a controller, we want to make sure that those values are within a prescribed range of acceptable values. Here's an [old reference](https://apps.dtic.mil/sti/pdfs/ADA119421.pdf) discussing this very topic (it's an old reference but still being used, at least the same values are still being used). The values will vary depending on the aircraft type and the regime of flights. Damping ratios and natural frequencies are two of the main variables which we want to set when designing a controller. 

So now, we want to control the output $y$. We then need to add a feedback loop which will take that input and compare it to the desired output, and generate elevator deflection commands.

## Feedback control

Here's the added loop for feedback control:

![Feedback control](https://upload.wikimedia.org/wikipedia/commons/6/66/Typical_State_Space_model_with_feedback_and_input.png?20141206181707)

Ignore D, again as it's not significant in aircraft control. Our controller will be $K$. It can be a simple proportional gain, it can be a PID controller. Airbus uses Eigenvector/Eigenstructure assignment method. Let's recall, we discuss Eigenvalues in the A matrix. The goal is to use the gains in matrix K to modify the Eigenvalues as desired. Eigenstructure assignment can be done using the state variables or the output variables. 

## C* value

Thru early fighter jets development, C* value has been found to be a reliable output to use in output feedback controllers. The Space Shuttle was using it. The A320 was the first large civil transport aircraft to use it. Using a C* value, without any input from the pilot, the aircraft will maintain its current equilibrium. At lower speeds, the sidestick would command pitch rate, and at higher speeds would be commanding a load factor. Boeing developped a similar concept called C*U. The objective was to make the aircraft behaves more like a conventional aircraft, requiring trimming from the pilot. Thus, the C*U value will generate a pitch rate if not trimmed out.

The equation for C* is:

$$
C^* = n_z + \frac{U_c}{g} q 
$$

where $U_c$ is the crossover speed.

Pitch rate is one of our state variable, and the load factor being $\frac{Lift}{mg}$, we can write an output vector that calculates C* value:

$$
\vec y
\begin{bmatrix}
0 & \frac{1}{mg} & \frac{U_c}{g} & 0
\end{bmatrix} \vec x
$$

The Eigenstructure assignment can either use the state variables as feedback, or the output variable as feedback, the former being more easier to implement. The two approaches are either using the state feedback as an inner loop, and implement a C* value control loop as an outer loop; or use the C* value as an output feedback. I cannot say which is being used by Airbus. The output feedback might be a bit harder to get a uniform response across the flight envelope, but that's just an educated guess.

References

- [E. Field - The Application of a C* flight control law to large civil transport aircraft](https://dspace.lib.cranfield.ac.uk/bitstream/handle/1826/186/coareport9303.pdf?sequence=2&isAllowed=y)
- [Blog post on C* value](https://www.engineeringpilot.com/post/2018/10/15/c-an-unknown-star)
- [Search results to references to C* use on Airbus](https://scholar.google.com/scholar?hl=en&as_sdt=0%2C5&q=c-star+value+airbus+control+engineering&btnG=)

## Vertical speed control loop

With basic instrumentation, vertical speed is typically measured with the static port. With an aircraft equipped with IRS, the vertical speed can also be measured with the IRS. Looking in the alpha call-up table, the source of the vertical speed is the ADR, it uses air data sensors. I have flown aircrafts in the past where the vertical speed was measured with the IRS.

In our state-space representation above, we can calculate the vertical speed from the state variables, assuming small variation of the angle of attack and pitch angle:

$$
\dot h = TAS \times (\theta - \alpha)
$$

Recall that we can express $\alpha$ with the state variables $u$ and $w$. Now, we can create a new output vector:

$$
\vec y=
TAS \times \begin{bmatrix}
0 & \frac{1}{TAS} & 0 & 1
\end{bmatrix} \vec x
$$

With that output, we can use some form of output feedback to control the vertical speed, and it will be an outer loop to the C* value control loop. The time order of the loops is important, the vertical speed control loop will be slightly slower than the C* value control loop. If it is tuned to be faster than the inner loop, that inner loop will be rendered useless, and the stability of the system will only be dependent on the vertical speed loop, which is not desired. 

![Change with K_i, an integral of the signal](https://upload.wikimedia.org/wikipedia/commons/c/c0/Change_with_Ki.png)

## Altitude capture
