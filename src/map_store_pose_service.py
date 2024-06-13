#!/usr/bin/env python

import rospy
import os
import yaml
import tkinter as tk
from tkinter import simpledialog
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Trigger, TriggerResponse
import threading
import queue

current_pose = None

def pose_callback(msg):
    global current_pose
    current_pose = msg.pose

class CustomDialog(simpledialog.Dialog):
    def __init__(self, parent, prompt, existing_labels):
        self.prompt = prompt
        self.existing_labels = existing_labels
        self.user_input = None
        super().__init__(parent, title="Input")

    def body(self, master):
        tk.Label(master, text=self.prompt, font=("Helvetica", 20)).pack()
        tk.Label(master, text="Existing labels:", font=("Helvetica", 20)).pack()
        for label in self.existing_labels:
            tk.Label(master, text=label, font=("Helvetica", 20)).pack()
        self.entry = tk.Entry(master, font=("Helvetica", 20))
        self.entry.pack(padx=5, pady=5)
        return self.entry

    def apply(self):
        self.user_input = self.entry.get()

def handle_store_map_pose(req, result_queue):
    if not rospy.has_param('/map_file'):
        rospy.logwarn("It is unknown which map is open, so no pose is stored")
        result_queue.put(TriggerResponse(success=False, message="Map file parameter not set"))
        return

    map_file_path = rospy.get_param('/map_file')
    map_file_name = os.path.basename(map_file_path).replace('.yaml', '')
    locations_dir = os.path.join(os.path.dirname(map_file_path), 'locations')
    locations_file_path = os.path.join(locations_dir, f'{map_file_name}_locations.yaml')

    if not os.path.exists(locations_dir):
        os.makedirs(locations_dir)

    locations_data = []
    if os.path.exists(locations_file_path):
        with open(locations_file_path, 'r') as file:
            locations_data = yaml.safe_load(file) or []

    if current_pose is None:
        result_queue.put(TriggerResponse(success=False, message="Current pose is not available"))
        return

    root = tk.Tk()
    root.withdraw()  # Hide the root window
    labels = [point['label'] for point in locations_data]
    prompt = "Enter label for new point.\nIf the label equals an existing label,\nit will be overwritten.\n"
    dialog = CustomDialog(root, prompt, labels)
    label = dialog.user_input
    root.destroy()

    if label is None:
        result_queue.put(TriggerResponse(success=False, message="No label entered"))
        return

    existing_point = next((point for point in locations_data if point['label'] == label), None)
    covariance = current_pose.covariance
    covariance_matrix = {
        'xx': covariance[0],
        'yy': covariance[7],
        'yawyaw': covariance[35]
    }

    new_point = {
        'number': len(locations_data) + 1 if existing_point is None else existing_point['number'],
        'label': label,
        'position': {
            'x': current_pose.pose.position.x,
            'y': current_pose.pose.position.y,
            'z': current_pose.pose.position.z
        },
        'orientation': {
            'x': current_pose.pose.orientation.x,
            'y': current_pose.pose.orientation.y,
            'z': current_pose.pose.orientation.z,
            'w': current_pose.pose.orientation.w
        },
        'covariance': covariance_matrix
    }

    if existing_point:
        existing_point.update(new_point)
    else:
        locations_data.append(new_point)

    with open(locations_file_path, 'w') as file:
        yaml.dump(locations_data, file, default_flow_style=False)

    result_queue.put(TriggerResponse(success=True, message="Pose stored successfully"))

def service_callback(req):
    result_queue = queue.Queue()
    thread = threading.Thread(target=handle_store_map_pose, args=(req, result_queue))
    thread.start()
    thread.join()
    return result_queue.get()

def main():
    rospy.init_node('store_map_pose_service')

    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, pose_callback)
    rospy.Service('/store_map_pose', Trigger, service_callback)

    rospy.loginfo("Ready to store map poses.")
    rospy.spin()

if __name__ == '__main__':
    main()


# #!/usr/bin/env python

# import rospy
# import os
# import yaml
# import tkinter as tk
# from tkinter import simpledialog
# from geometry_msgs.msg import PoseWithCovarianceStamped
# from std_srvs.srv import Trigger, TriggerResponse

# current_pose = None

# def pose_callback(msg):
#     global current_pose
#     current_pose = msg.pose

# class CustomDialog(simpledialog.Dialog):
#     def __init__(self, parent, prompt, existing_labels):
#         self.prompt = prompt
#         self.existing_labels = existing_labels
#         self.user_input = None
#         super().__init__(parent, title="Input")

#     def body(self, master):
#         tk.Label(master, text=self.prompt, font=("Helvetica", 20)).pack()
#         tk.Label(master, text="Existing labels:", font=("Helvetica", 20)).pack()
#         for label in self.existing_labels:
#             tk.Label(master, text=label, font=("Helvetica", 20)).pack()
#         self.entry = tk.Entry(master, font=("Helvetica", 20))
#         self.entry.pack(padx=5, pady=5)
#         return self.entry

#     def apply(self):
#         self.user_input = self.entry.get()

# def handle_store_map_pose(req):
#     if not rospy.has_param('/map_file'):
#         rospy.logwarn("It is unknown which map is open, so no pose is stored")
#         return TriggerResponse(success=False, message="Map file parameter not set")

#     map_file_path = rospy.get_param('/map_file')
#     map_file_name = os.path.basename(map_file_path).replace('.yaml', '')
#     locations_dir = os.path.join(os.path.dirname(map_file_path), 'locations')
#     locations_file_path = os.path.join(locations_dir, f'{map_file_name}_locations.yaml')

#     if not os.path.exists(locations_dir):
#         os.makedirs(locations_dir)

#     locations_data = []
#     if os.path.exists(locations_file_path):
#         with open(locations_file_path, 'r') as file:
#             locations_data = yaml.safe_load(file) or []

#     if current_pose is None:
#         return TriggerResponse(success=False, message="Current pose is not available")

#     root = tk.Tk()
#     root.withdraw()  # Hide the root window
#     labels = [point['label'] for point in locations_data]
#     label_str = "\n".join(labels)
#     prompt = f"Enter label for new point.\nIf the label equals an existing label,\nit will be overwritten.\n"
#     dialog = CustomDialog(root, prompt, labels)
#     label = dialog.user_input
#     root.destroy()

#     if label is None:
#         return TriggerResponse(success=False, message="No label entered")

#     existing_point = next((point for point in locations_data if point['label'] == label), None)
#     covariance = current_pose.covariance
#     covariance_matrix = {
#         'xx': covariance[0],
#         'yy': covariance[7],
#         'yawyaw': covariance[35]
#     }

#     new_point = {
#         'number': len(locations_data) + 1 if existing_point is None else existing_point['number'],
#         'label': label,
#         'position': {
#             'x': current_pose.pose.position.x,
#             'y': current_pose.pose.position.y,
#             'z': current_pose.pose.position.z
#         },
#         'orientation': {
#             'x': current_pose.pose.orientation.x,
#             'y': current_pose.pose.orientation.y,
#             'z': current_pose.pose.orientation.z,
#             'w': current_pose.pose.orientation.w
#         },
#         'covariance': covariance_matrix
#     }

#     if existing_point:
#         existing_point.update(new_point)
#     else:
#         locations_data.append(new_point)

#     with open(locations_file_path, 'w') as file:
#         yaml.dump(locations_data, file, default_flow_style=False)

#     return TriggerResponse(success=True, message="Pose stored successfully")

# def main():
#     rospy.init_node('store_map_pose_service')

#     rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, pose_callback)
#     rospy.Service('/store_map_pose', Trigger, handle_store_map_pose)

#     rospy.loginfo("Ready to store map poses.")
#     rospy.spin()

# if __name__ == '__main__':
#     main()
