The "Simplistic" model comes [here](https://www.ee.usyd.edu.au/tutorials_online/matlab/examples/pend/invpen.html).

It does not account for inertia of wheels, and it models both wheels as a single mass.

The readings it wants from encoders are *X POSITION*, so in balancer.ino, when updating estimator in the main loop, make sure to send this (and not phi)
