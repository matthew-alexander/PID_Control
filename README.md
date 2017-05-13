# PID_Control
The goal of this project is to design a PID controller to move a car around a track simulation in C++. 


## The effect of parameters
The main parameters involved in the design of the controller are the "proportional" (P), the "differential" (D), and the "integral" (I) components of the general equation that determines the steering angle. This equation is described below. Kp, Kd, and Ki are the proportional, differential and integral paramters, respectively: 

![alt text](https://wikimedia.org/api/rest_v1/media/math/render/svg/cd581e5c8539ce46453574d1188bd9d52a610fe0 "controller output. source wikimedia")

The proportial parameter in effect controls how rapidly the outout adjusts to the error. The higher the value for this parameter created large ocillations back and forth accorss the lane of the track. If this was the only parameter used, the car would ocilate constantly , overshooting the middle of the track each time. The way to stop the overshooting is to use a parameter that ajusts teh controller output to take into consideration the change in error over time, which is the differential parameter. 

The differential parameter does the job of decreasing the output as the cross check error becomes smaller through time. Thus, the effect is that as the car adjust back to the center of the road, the steering angle of the car decreases through time to "ease" into  a center lane position. There is still over overshooting, and there is still visible ocillations in the simultion. I found that that the parameter most successful was a Kd of 0.9. 

The integral parameter attempts to compensate for sytematic bias in the vehicle. If there is a alighnment issue that is consistenly placing the car off of center, this parameter uses the total sum of cross check error to compensate. The parameter I found to be most successful through hand tuning experiments, was 0.004. 

## Final parameters
The final parameters were chosen by manual tuning through many iterations of watching the car and studying the python example of Sebastian Thrun. The final parameters that I have for a throttle of 0.4 are Kp = 0.09; Kd = 0.9; Ki = 0.004. 

A video of the result can be seen below: 

<a href="http://www.youtube.com/watch?feature=player_embedded&v=https://www.youtube.com/watch?v=ykg7dqplO6E" target="_blank"><img src="http://img.youtube.com/vi/https://youtu.be/ykg7dqplO6E/0.jpg" 
alt="Simulation Result" width="480" height="360" border="1" /></a>

or here: https://youtu.be/ykg7dqplO6E