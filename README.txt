Code by Chance Cardona
Uses python
To launch in train mode:
	roslaunch wall_follower train.launch
To launch in test mode (NOTE! If no ccardona_Q_table.npy file in node's directory it will run pre initialized Q table):
	roslaunch wall_follower test.launch

More information about this code is included inside of the pdf in this directory. 

This is the best trained result, from about 4 hours of training 150 episodes. You shouldn't have to 
worry about moving any files around, this one should all work out of the box.

This version now actually works to solve all corners. Improvements were changing my acion space to be smaller,
adding a linear regression to calculate an Orientation state, and tweaking my LIDAR sweep domains. In addition
I updated the launch files so the code works out of the box now, with ccardona_Q_table.npy being included in 
the wall_follower/src/ directory, which is where the code looks for it now. Thank you for allowing corrections!
video at: https://youtube.com/video/QO0UvDqAUf8/
