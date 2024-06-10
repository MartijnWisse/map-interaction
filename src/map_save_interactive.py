#!/usr/bin/env python

import rospy
import rospkg
import os
import tkinter as tk
from tkinter import simpledialog
from std_srvs.srv import Trigger, TriggerResponse

# Find the path to the map_interaction package
rospack = rospkg.RosPack()
package_path = rospack.get_path('map_interaction')
MAPS_DIR = os.path.join(package_path, "maps")

def get_available_maps():
    maps = []
    for file in os.listdir(MAPS_DIR):
        if file.endswith(".yaml"):
            maps.append(os.path.splitext(file)[0])
    return maps

def save_map(map_name):
    map_file_path = os.path.join(MAPS_DIR, map_name)
    os.system(f"rosrun map_server map_saver -f {map_file_path}")

class MapSaveDialog(simpledialog.Dialog):
    def __init__(self, parent, maps):
        self.maps = maps
        self.map_name = None
        super().__init__(parent, title="Save Map")

    def body(self, master):
        tk.Label(master, text="Provide map name (without extension). If this map already exists, it will be overwritten.").pack()
        tk.Label(master, text="Existing maps:").pack()
        for map_name in self.maps:
            tk.Label(master, text=map_name).pack()
        self.entry = tk.Entry(master)
        self.entry.pack(padx=5, pady=5)
        return self.entry

    def apply(self):
        self.map_name = self.entry.get()

def show_save_dialog(maps):
    root = tk.Tk()
    root.withdraw()  # Hide the root window
    dialog = MapSaveDialog(root, maps)
    new_map_name = dialog.map_name
    root.quit()
    root.destroy()
    return new_map_name

def handle_save_map_interactively(req):
    maps = get_available_maps()
    map_name = show_save_dialog(maps)
    if map_name:
        save_map(map_name)
        return TriggerResponse(success=True, message="Map saved successfully.")
    else:
        rospy.logwarn("No map name provided")
        return TriggerResponse(success=False, message="No map name provided.")

def save_map_interactive_server():
    rospy.init_node('save_map_interactive_server')
    s = rospy.Service('save_map_interactively', Trigger, handle_save_map_interactively)
    rospy.loginfo("Ready to save maps interactively.")
    rospy.spin()

if __name__ == "__main__":
    save_map_interactive_server()
