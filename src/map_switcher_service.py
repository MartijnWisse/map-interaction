#!/usr/bin/env python

import rospy
import roslaunch
import rospkg
from map_interaction.srv import SwitchMap, SwitchMapResponse
import os

class MapSwitcher:
    def __init__(self):
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)
        self.launch = None
        self.map_path = None

    def handle_switch_map(self, req):
        try:
            # Store the new map path to be used by the main thread
            self.map_path = req.map_path

            # Shutdown existing launch if it exists
            if self.launch is not None:
                self.launch.shutdown()

            # Indicate success of the service call
            return SwitchMapResponse(success=True)
        except Exception as e:
            rospy.logerr(f"Failed to switch map: {e}")
            return SwitchMapResponse(success(False))

    def run(self):
        # Initialize node
        rospy.init_node('map_switcher_service')

        # Create service
        s = rospy.Service('switch_map', SwitchMap, self.handle_switch_map)
        rospack = rospkg.RosPack()

        # Main loop to handle the actual launch in the main thread
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            if self.map_path:
                if self.map_path.endswith('newmap.yaml'):
                    try:
                        # Find the path to the map_interaction package (i.e., own package)
                        package_path = rospack.get_path('map_interaction')
                        launch_file = package_path + "/launch/slam_gmapping.launch"
                        
                        # Create a new launch instance with the updated arguments
                        self.launch = roslaunch.parent.ROSLaunchParent(
                            self.uuid,
                            [launch_file]
                        )
                        self.launch.start()

                        # Reset map_path to avoid re-launching
                        self.map_path = None
                    except Exception as e:
                        rospy.logerr(f"Failed to launch new map: {e}")

                else:    
                    try:
                        # Find the path to the ridgeback_navigation package
                        package_path = rospack.get_path('ridgeback_navigation')
                        launch_file = package_path + "/launch/amcl_demo.launch"

                        # Command to launch the map server and amcl with the new map file
                        map_arg = ['map_file:=' + self.map_path]
                        roslaunch_args = roslaunch.rlutil.resolve_launch_arguments([launch_file])
                        launch_files = [(roslaunch_args[0], map_arg)]
                        
                        # Create a new launch instance with the updated arguments
                        self.launch = roslaunch.parent.ROSLaunchParent(
                            self.uuid,
                            launch_files
                        )
                        self.launch.start()

                        # Reset map_path to avoid re-launching
                        self.map_path = None
                    except Exception as e:
                        rospy.logerr(f"Failed to launch new map: {e}")
            rate.sleep()

if __name__ == "__main__":
    switcher = MapSwitcher()
    switcher.run()