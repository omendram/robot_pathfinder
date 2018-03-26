
This is the solution for the ASSIGNMENT 2 from Group No. 9. 

In order to run the python script (named as assignment2.py), the OS would need the following dependencies resolved:
	
	1. Python3
	2. Numpy
	3. PyGame
	4. Shapely

The Video Report contains a number simulations under different noise and speed values including a simulation without having the Kalman Filter applied. We gather noisy data about the robot's position by the sensor's values, and it helps us better estimate the Position and correct the belief / local data about the position. The visual traces of correction can be seen behind the actual position of the robot, which is constantly being corrected at each step.

The noise / speed value of simulations are given as follows:

Case I: High Noise - High Speed
	Noise Values: 
		noise1 = 0.4 # noise for rot1 and 2
		noise2 = 0.2 # trans noise for rot1 and 2
		noise3 = 0.4 # trans noise for trans
		noise4 = 0.2 # rot noise for trans
	Speed Value: 
		speed = 5

Case II: Medium Noise - High Speed
	Noise Values: 
		noise1 = 0.2 # noise for rot1 and 2
		noise2 = 0.1 # trans noise for rot1 and 2
		noise3 = 0.2 # trans noise for trans
		noise4 = 0.1 # rot noise for trans
	Speed Value: 
		speed = 5

Case III: No Noise - High Speed
	Noise Values:
		noise1 = 0.0 # noise for rot1 and 2
		noise2 = 0.0 # trans noise for rot1 and 2
		noise3 = 0.0 # trans noise for trans
		noise4 = 0.0 # rot noise for trans
	Speed Value:
		speed = 5

Case IV: Medium Noise - Low Speed
	Noise Values:
		noise1 = 0.2 # noise for rot1 and 2
		noise2 = 0.1 # trans noise for rot1 and 2
		noise3 = 0.2 # trans noise for trans
		noise4 = 0.1 # rot noise for trans
	Speed Value:
		speed = 2

Case V: Very High Speed - Medium Noise 
	Noise Values:
		noise1 = 0.3 # noise for rot1 and 2
		noise2 = 0.1 # trans noise for rot1 and 2
		noise3 = 0.2 # trans noise for trans
		noise4 = 0.2 # rot noise for trans
	Speed Value:
		speed = 10


The values of the variables indicated above can be assigned on the script at the given line no. ranges. 

Thank you!

Regards
Group 9
