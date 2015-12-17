# Robot identification using genetic algorithms 

This is the code to identify the dynamic parameters of a humanoid robot model. It was developed during my [PhD thesis](http://miguelgfierro.com/docs/gonzalez-fierro2014thesis.pdf). The model is a triple inverted pendulum, with three links and three masses, and the robot is the humanoid HOAP-3.

To identify the system we optimized the triple pendulum parameters minimizing the difference between the theoretical Zero Moment Point (ZMP) and the real ZMP. The [ZMP](https://en.wikipedia.org/wiki/Zero_moment_point) is a criteria for dynamic stability commonly used on humanoids. 

The optimization is constrained to respect the torque limits of the robot. The torques were computed using an inverse dynamics method based on the [Lagrange formulation](https://en.wikipedia.org/wiki/Lagrangian_mechanics).

The optimization method is a genetic algorithm called [Differential Evolution](https://en.wikipedia.org/wiki/Differential_evolution). This algorithm is a good method for multiobjective optimization with constraints. 

For more information please visit my [blog](http://miguelgfierro.com) or my [youtube channel](https://www.youtube.com/user/ciruselvirus)
