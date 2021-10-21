# PID Controller
Lateral trajectory tracking using a Proportional-Integral-Derivative (PID) controller to 
regulate the steering angle.

This project is part of Udacity's [Self-Driving-Car Nanodegree][Course]. The project 
description and build instructions can be found [here][Project], the simulator 
[here][Simulator].

## PID Control overview
The PID controller is one of the most frequently used control loop feedback mechanisms in 
practice. Let us have a quick overview about what a control loop actually is.

![][ControlLoop]

The **plant** in a control loop is the system that we want to control, the system whose 
behaviour we want to affect. In this project the system is the vehicle in the simulator, and
we want it to drive in the center of the lane. The input into the system is the **actuating 
signal**, in our case a specific position of the steering wheel. The vehicle will take the 
provided steering command and change its lateral position according to its specific system 
dynamics. The lateral position of the vehicle is the output of the system, the **controlled 
variable**. The basic idea of a control system is to figure out how to generate the 
approriate actuating signal, so that the system produces the desired output. In our case this 
means, how to move the steering wheel, so that the vehicle drives in the center of the lane. 
The system output is fed back and compared to the **reference variable**: the center of the 
lane. This shows us how far off the vehicle is from where it should actually be. The 
difference is measured by the **error term**, which in our case is the *cross track error*, 
the distance of the vehicle from the centerline. If the vehicle's position is near the lane 
center, the cross track error is close to zero, and that is exactly what we want, a zero 
error. And here is where the **contoller** comes into play. The controller's task is to 
convert the error term into a suitable actuator command: an appropriate position of the 
steering wheel. This shall be done in a way, that over time the error converges to zero. So
how does the PID controller accomplish this task? 

Let's consider that the vehicle is too far to the left. Then it needs to turn to the right 
and vice versa. But how much?

![][PID]

### Proportional controller term
One way to set the steering wheel angle is to use the so-called **proportional control**. It 
determines the steering command by multipliying the **present cross track error** ***e(t)*** 
with a scaling factor called the **proportional gain** ***K<sub>P</sub>***. The proportional 
controller steers all the more, the further away the vehicle is from the centerline, or in other
words the higher the cross track error *e(t)* is. When the vehicle approaches the center of the 
lane, the steering angle gets smaller and smaller, being zero at the moment the vehicle reaches 
the centerline. However, the vehicle may still overshoot, since its orientation typically is not
aligned with the centerline. If the proportional gain is set too high, the system can become 
unstable with increasing oscillation magnitudes (left video below). In contrast, if the gain is 
set to low, the steering commands will not be large enough to keep the vehicle inside the lane 
(center video). The gain must have a certain magnitude to provide steering commands that are 
large enough to navigate the vehicle around the curve. However, the proportional controller 
alone is not sufficient to do this safely, as the abruptly increasing cross track error in the 
curves leads to overshooting tendencies resulting in strong oscillations (right video). We need
to consider an additional control term.

| high K<sub>P</sub> (unstable) | low K<sub>P</sub> (to slow) | medium K<sub>P</sub> (still oscillating)|	
| ----------------------------- | ----------------- | --------------------------- |
| ![][KpUnstable]               | ![][KpLow]        | ![KpInit]                   |

### Derivative controller term
A good candidate for an extra measurement is the  **cross track error rate**, the time 
derivative of the error. By multiplying it with its own gain ***K<sub>D</sub>*** we get 
the **derivative controller** term, which measures how fast the error evolves over time. In our
case this means, how fast the vehicle is moving to or away from the center of the lane. The 
derivative gain *K<sub>D</sub>* needs to be tuned simultaneously to the proportional gain 
*K<sub>P</sub>*. When the vehicle approaches the centerline, the cross track error *e(t)* and 
its derivative *de/dt* have opposite signs and therefore act in opposite directions. 

> Conceptually we can think that increasing the proportional gain *K<sub>P</sub>* will increase 
> the pull that the vehicle feels towards the center of the lane, and increasing the derivative
> gain *K<sub>D</sub>* increases the resistance the car will feel against moving too quickly 
> towards it.  

The derivative term can be interpreted as a prediction of the future error, and it is used to 
reduce overshooting and oscillation tendencies. If it is too low (for a given proportional gain),
the system is called under damped, and it will still overshoot and oscillate. If it is too high, 
the system will be over damped, and it will take a long time to settle. Properly choosing the 
derivative gain allows the car to approach the desired trajectory quickly with a cross track 
error rate close to zero. This is being called critically damped. By adding the derivative 
control term, the intense oscillations of the plain P Controller could be reduced, and the 
vehicle now is able to take the curves.

![][PDControl]

Although a plain PD controller is sufficient to safely navigate the vehicle on this track, for
reasons of completeness, let's take a look on the last term of the PID controller.

### Integral controller term
Environmental factors or mechanical defects can change the vehicle's nominal behavior and 
thus the performance of the controller. For example if there's a heavy crosswind the vehicle 
may drift sidewards unless the driver counteracts the wind force with a corrective steering 
command. The vehicle may experience a residual lane offset, called the **steady-state error**. 
One way to address this problem is to add yet another term: the integral term. This third 
measurement sums up the cross track error over the time, by taking the integral &int;e(t)dt. It 
gives an indication of whether the vehicle spends more time on one side of the trajectory than 
on the other. The **integral term** is multiplied by its own gain ***K<sub>I</sub>***. If the
gain is too large, normal controller fluctuations could get exaggerated and the controller can 
become unstable. However, if the gain is too low, the response to the dynamic changes could
take to long. If the gain is just right, the controller can quickly reduce the steady-state 
error and return to its nominal performance. 

### PID control tuning
The combination of these three terms results in the PID control: a versatile controller 
that uses the present, the past, and a prediction of the future error to calculate the 
appropriate actuator commands. Each of the errors contributes some amount to the overall 
output of the controller, whereby the contributions are weighted by the corresponding PID gains. 
By properly tuning each of the gains, different characteristics of the dynamic system can be
addressed, as shown in the subsequent table. However, note that the PID gains are dependent 
of each other, and changing one of them can effect the other two. The table shall only give 
a reference about the general effects of each of the gains on the system's behaviour.

![][Tuning]

## Result
The PID gains were tuned manually, by observing their effects on the driving behaviour in the 
simulator and considering their influence on the system's behaviour, as shown in the table 
above. The finally chosen gains are

> ``Kp = 0.12``  
> ``Ki = 0.00``  
> ``Kd = 3.50``

Since there was no systematic bias in the simulator causing a steady-state error, the integral 
term was neglected, and a plain PD controller was used to complete this project. The 
implementation of the controller can be found in the [`src`][Src] folder.

![][Final]


[Course]: https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
[Project]: https://github.com/udacity/CarND-PID-Control-Project
[Simulator]: https://github.com/udacity/self-driving-car-sim/releases/tag/v1.45

[ControlLoop]: images/contol_loop.png "Feedback control loop"
[PID]: images/PID.png "PID control loop"
[Tuning]: images/Tuning.png "PID control tuning"

[KpLow]: videos/kp_low_2x_480.gif "Low Kp"
[KpUnstable]: videos/kp_unstable_480.gif "High Kp (unstable)"
[KpInit]: videos/kp_initial_1_5x_480.gif "Initial Kp"
[PDControl]: videos/pd_control_1_5x_480.gif "PD controller"
[Final]: videos/pd_control_2x_480.gif "Full lap"

[Src]: https://github.com/Harlequln/C2M13X-PID_Controller/tree/main/src
