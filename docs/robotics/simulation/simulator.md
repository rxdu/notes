# Robot Simulator

## Simulation Process

The process of robot simulation is essentially calculating the system state using it's mathematic model with given initial conditions and inputs. Generally we use ordinary differential equations (ODEs) to describe our robots. By solving for the system states from the set of ODEs at discrete time $0, t_s, 2t_s, ...$, we can get the simulated behavior of the robot during the simulated period.

Similarly a physics engine can calculate the state of a system using numerical methods. (Read about ODE/Bullet for more information). We can set a step size (t_phy_s) for the physics engine so that we can get the calculated system state at time $t_{phy_s}, 2t_{phy_s}, 3t_{phy_s} ...$ consecutively when we call the physics engine to do the calculation. If we invoke the calculation at $f_{phy}$ Hz with the step size of t_phy_s, it means we will have the system state simulated from time 0 to $f_{phy} \cdot t_{phy_s}$ in 1 second. For example $f_{phy} = 1kHz$, and $t_{phy_s} = 1ms$, after the last calculation iteration we will get the system state at $t = 1k \cdot 1ms = 1s$. Since the wall time of the real world elapses for 1s and the simulated system state also advances for 1s, we can say that the real-time factor is 1.

In an extreme condition, if we call the physics engine to calculate very fast and we use a big step size, then by the end of the 1s simulation period, we will get the calculated system state at time $t_s > 1s$. This means time in the simulator elapses faster than time in the real world, thus real-time factor is greater than 1.

Of course, limited by the computational power of the computer, the update rate of the physics engine cannot be infinitely high. If a system is extremely complex and the the physics engine cannot get a solution within 1s, then even if we set the step size to be 1s (which is pretty large when simulating a process), we still cannot get a real time factor to be equal or greater than 1.

As to the simulation step, it involves sensing and rendering. The physics engine can run at a much higher rate while we don't necessarily update the result to the user at each step. Hence we may have the physics engine run at 1kHz in the background and only refresh the simulation scene for the user at 50Hz.

## Reference:

- [Gazebo Document](https://gazebosim.org/docs/latest/getstarted/)
- [V-REP Document - Simulation](https://manual.coppeliarobotics.com/en/simulation.htm)
