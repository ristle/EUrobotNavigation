import matplotlib.pyplot as plt
import numpy as np

''' This is just plots '''

def mjtg(current, setpoint,current_y, setpoint_y, frequency, move_time):
    trajectory = []
    trajectory_y = []
    trajectory_derivative = []
    trajectory_acceleration = []
    trajectory_ang_vel = []
    timefreq = int(move_time * frequency)

    for time in range(1, timefreq):
        trajectory.append(
            current + (setpoint - current) *
            (10.0 * (time/timefreq)**3
             - 15.0 * (time/timefreq)**4
             + 6.0 * (time/timefreq)**5))
        trajectory_y.append(
            current_y + (setpoint_y - current_y) *
            (10.0 * (time/timefreq)**3
             - 15.0 * (time/timefreq)**4
             + 6.0 * (time/timefreq)**5))

        trajectory_derivative.append(
            frequency * (1.0/timefreq) * (setpoint - current) *
            (30.0 * (time/timefreq)**2.0
             - 60.0 * (time/timefreq)**3.0
             + 30.0 * (time/timefreq)**4.0))# velocity x'
        trajectory_acceleration.append(
             frequency * (1.0/timefreq**2) * (setpoint - current) *
            (60.0 * (time/timefreq)
             - 180.0 * (time/timefreq)**2.0
             + 120.0 * (time/timefreq)**3.0))
        
        # here r going to be x'' and x'''  -  ускорение и угловное ускорение
        

    return trajectory, trajectory_y, trajectory_derivative, trajectory_acceleration

# Set up and calculate trajectory.
average_velocity = 90.0
current = 13.0
current_y = 23.0
setpoint = 60.0
setpoint_y = 54.
frequency = 45
time = (setpoint - current) / average_velocity

traj, traj_y, traj_vel, tr_acc = mjtg(current, setpoint,current_y, setpoint_y, frequency, time)

# Create plot.
xaxis = [i / frequency for i in range(1, int(time * frequency))]

plt.plot(xaxis, traj)
plt.plot(xaxis, traj_y)
plt.plot(xaxis, traj_vel)
plt.plot(xaxis, tr_acc)
plt.title("Minimum jerk trajectory")
plt.xlabel("Time [s]")
plt.ylabel("Angle [deg] and angular velocity [deg/s]")
plt.legend(['pos','pos_y', 'vel', 'acc'])
plt.show()