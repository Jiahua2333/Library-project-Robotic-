BreezyCreate2 provides a simple abstraction layer on top of the 
<a href="https://github.com/pomeroyb/Create2Control">Create2API</a>
library by Brandon Pomeroy, suitable for use by beginning Python programmers.
BreezyCreate2 uses the standard Python distutils
to install the Python module breezycreate2 and the JSON file required by
Create2API.  I have tested it with Python 2.7 and 3.5.

Once you've installed BreezyCreate2, you can access its sole
class, the <tt>Robot</tt> class, which has easy methods for interacting
with the robot: <tt>setForwardSpeed</tt>, 
<tt>playNote</tt>,  <tt>getBumpers</tt>, etc. (See the <tt>robotest.py</tt>
script for an example.)

The <tt>roboserver.py</tt> script can be run on a Raspberry Pi or other
single-board computer, to control your Create2 over a wireless ad-hoc
network.  The corresponding <tt>robotclient.py</tt> script uses a joystick or
game controller to send commands to the server over the network.   The <tt>playsong.py</tt>
script will use the Create2 to play a familiar melody.



Files explaination:

1. IRobotProcessing.py

  This program simplifies some robot's functions from the "_init_.py" file and adds some news functions for 
  the for the iRobot Create2, such as getDistanceAngle(), getEncoder(), lineFollowing().

2. LibraryProject.py

  This class is about some openCV image processing. Also, The main function is in this class.
  Run it for the whole project.

3. Colltrolling.py

  This program uses keyboard to control the robot move.
  
  Attention: when the panel appears on the screen, it must click inside
              one time first(that is nothing), then, it can use the keyboard 
              control the robot.
