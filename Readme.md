Anton Khabbaz  HW9
akhabbaz


Features:

Added a GUI that allows one to change the parameters of feedback for the rotational motion.  The parameters are the physical ones, the frequency and the damping ratio.  One sets the values and then clicks on the button at the bottom to update them.  One can see that with a damping ratio of 1 the motion is stable, but when the ratio is less than one, the angle oscillates.

I set the frequency to 16 Hz and the damping ratio to be 1.  I also set the time constant for the force as required.


Got all the behaviors working up.  I implemented Wander in both ways and one can uncomment out the #define RADIALDECELERATION line
to try the second way.  The first method worked better. I needed to turn down TAvoid to get it to work.

I wrote a class FlockBehaviors that figured out the next neighbor. It returned an iterator to the next neighbor and returned last if there
were no more neighbors.  It was the base class for all the collective behaviors.  This allowed implementing all the collective behaviors with a 
simple while loop that was easy to understand.

Cohesion Alignment, Separation all work.  I found no issues.

There are constants in the top of Behaviors.cpp that set the flocking and leader behaviors

The flocking and leading all work as suggested. To tune them, I put the coefficients in the GUI and adjusted them.
The leader is pretty perfect and works with the parameters set.  Flocking works well too.  The leader will always end up near the target,
but the others may not if they get separated from the leader.  They may find themselves paired up but with no leader and so those two will 
stay together but won't joint the flock.  I think this is alright.

I added one more behavior called Guided Avoid.  It currently has a weight of avoid and separation.  This way the actors avoid obstacles but 
don't clump all up in the end.  One can see how this works very well.


