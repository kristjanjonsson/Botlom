    #!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Vector3

x, y = None, None
roll, pitch, yaw = None, None, None

relative_positions = None

class SimulationDriver:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        rospy.Service('simulation_driver', CanPosition, self.brake)
        rospy.Service('simulation_driver', CanPosition, self.circle)
        rospy.Service('simulation_driver', CanPosition, self.tank)
        rospy.Service('simulation_driver', CanPosition, self.turn)
        self.circle_service_handle = rospy.ServiceProxy('get_can_position', CanPosition)

    # Takes in a single boolean. When the boolean is true (e.g. 1), stops the robot.
    def brake(self, breakNow):
        pass

    # Takes in three parameters: clear (a boolean), speed, and radius. speed sets the forward speed in
    #    mm/s from -500 to 500 mm/s (negative speeds move the robot backwards).
    # Radius is used to set the radius of the circle in mm that the robot will move along.
    def circle(self, clear, speed, radius):
        pass

    # Takes in three parameters: clear (a boolean), left, and right.
    # left and right are used to set the left and right wheel velocities respectively.
    # They can vary between -500 and 500 and are interpreted as mm/s with negative velocities moving backward.
    def tank(self, boolean, left, right):
        pass

    # Takes in two parameters: clear (a boolean), and turn. turn is a velocity in mm/s.
    # Positive velocities turn the robot in place clockwise.
    # Negative velocities turn the robot in place counter-clockwise.
    def turn(self, clear, turn):


if __name__ == "__main__":
    driver = SimulationDriver('simulation_driver')
