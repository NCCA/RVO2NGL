#RVO2 NGL Demos

A series of demos showing how to use the RV02 and RV02-3D [lib](http://gamma.cs.unc.edu/RVO2/) and NGL.

These demos are all based on the examples shiped with the library and have the same format for each.

```
	/// a pointer to the sim
    std::unique_ptr<RVO::RVOSimulator > m_sim;
    /// goals for the sim to reach
    std::vector <int> m_goals;
    // called to setup the simulation
    void setupSim();
    // this sets the agent velocities for the frame
    void setPreferredVelocities();
    // sees if all agents have reached the goal
    bool reachedGoal() ;
    // This will update the sim
    void timerEvent(QTimerEvent *);
```


