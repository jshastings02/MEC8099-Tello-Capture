This Github repository contains code used and referenced in the MEC8099 Group Project titled 
"Development of Swarm Robotics in Autonomous Drones for Aerial Object Detection & Net Capture: Design, Manufacture, and Testing"

'final_detection_and_autonomous_capture_code' requires 'best.pt' to be downloaded along with it as this file is necessary to run the trained YOLOv11 model. 
On line 64 of the code you will need to update the path to wherever you have saved best.pt on your device.

'Enclosing_method' provides an independent coding protocol for shortening the distance between drones, to encapsulate the captured object. This can be tested directly without updating any lines.

'Leader_switch' is an independant code with the algorithm for a failsafe mechanism which allows a follower to take over as the leader in case of loss of control of the starting leader. 

'Vertical_Estimation' is code used to estimate the vertical distance between the leader drone and net centre 
