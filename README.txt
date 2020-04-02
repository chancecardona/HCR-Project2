Code by Chance Cardona
Uses python3
To launch in train mode:
	roslaunch wall_follower train.launch
To launch in test mode (NOTE! Requires valid ccardona_Q_table.npy in your ~/.ros/ directory):
	roslaunch wall_follower test.launch

Make sure that you copy and pase the ccardona_Q_table.npy from this directory into your ~/.ros/ directory
before you run the test.launch file! Else you'll run into an error. This is the result of my best trained
set, but ultimately the robot just went in a circle.

I tried my best to get this code working, but ultimately I couldn't get the robot to train correctly.
In my code I have fullfilled all the other requirements as far as I know. This code correctly updates
the Q table, uses a decayed epsilon for its policy, randomizes robot position each time, and can load
and save the Q table. I fully beleive that if this problem is possible a few tweaks in the right
direction would make it work, however I'm just out of time. This project was really hard. I played with
different lidar scan ranges, different speeds, I created more turning actions possible, and a ton of 
different reward functions. I even tried to get it to work with LinearRegression to estimate the walls
orientation, however due to the way the lidar is not continuous I was not able to get this working.
I thus have no video link.
