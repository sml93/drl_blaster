This is a work-in-progress (wip) repository at an attempt to write a **DRL network for UAV attitude control**.  

The prior/ background work comes from this repo: https://github.com/PX4-Gazebo-Simulation  
However more work needs to be done since most of the stuff involving mavros has not been updated since four years ago.  

23 May 2022
Currently, in drl_blaster repo:  
-> blasterDRL.py doesnt work as intended because:  
--> Att_running msg is abit unstable. It requires the topic "att_running_msg" to be published which is now handled by an external offb_ctrl.py.   
--> Action does not set/send anything.  

TODO/TO FIX:  
1) Try to incorporate an offboard function/node in the blasterDRL.py or create a separate script for putting the UAV in offboard.  
2) To relook into actions, make sure to be able to send the required info (i.e. thrust etc.).  
3) If attempting to redo, prepare for attitude/position (x,y,z) instead of just z.  
4) ~~How to get thrust states from mavros (FIXED with /setpoint_raw/target_attitude)~~.  
5) To check on env_ip and its publisher.   
