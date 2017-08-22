# CarND-Controls-MPC

--


![](v1.png)

For the cost function, I defined a number of coefficients to control what elements were taken into account and by how much in relation to each other. They are:

```
// How bad is it to be away from the centre of the
// road? Pretty bad.
#define COST_CROSSTRACK_ERROR 100 

// How bad is it to be angled differently than the
// ideal path? Not as bad.
#define COST_ANGLE_ERROR      10 

// How bad is it to be going slower than we want?
// Okay.
#define COST_SPEED_ERROR      5   

// How bad is it to speed up or turn left or right 
// quickly? Bad.
#define COST_MOVEMENT_ERROR   10  

// How bad is it to speed up, slow down, turn left 
// and right jittery? Real bad.
#define COST_JERK_ERROR       20  

// How bad is it to have velocity while taking a
// corner? Just don't do it!
#define COST_SPEED_CORNERING  50  
```

For N, the number of timesteps calculated each frame, I first tried 20, but it couldn't solve it fast enough and ran off the road. I also tried 5, and it drove smoothly but with a large crosstrack error, at the side of the road. So I settled with 10, which plots far enough out to drive smoothly and in the centre of the road.

![](v1.gif)
