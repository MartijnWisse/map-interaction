#!/usr/bin/env python

import rospy
from map_interaction.srv import SwitchMap, SwitchMapRequest, SwitchMapResponse
from map_interaction.srv import SwitchMapInteractively, SwitchMapInteractivelyResponse
import os
import tkinter as tk
from tkinter import simpledialog

MAPS_DIR = "/home/mwisse/mirte_ws/src/mirte-ros-packages/ridgeback/ridgeback_navigation/maps"

def get_available_maps():
    maps = []
    for file in os.listdir(MAPS_DIR):
        if file.endswith(".yaml"):
            maps.append(file)
    return maps

def switch_map(map_file):
    rospy.wait_for_service('switch_map')
    try:
        switch_map_service = rospy.ServiceProxy('switch_map', SwitchMap)
        response = switch_map_service(map_file)
        return response.success
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False

class MapSelectionDialog(simpledialog.Dialog):
    def __init__(self, parent, maps):
        self.maps = maps
        self.selected_map = None
        super().__init__(parent, title="Switch Map")

    def body(self, master):
        tk.Label(master, text="Switch map to:").pack()

        self.listbox = tk.Listbox(master)
        self.listbox.pack(padx=5, pady=5)

        # Add the options to the listbox
        self.listbox.insert(tk.END, "new map")
        for map_name in self.maps:
            self.listbox.insert(tk.END, map_name)

        self.listbox.selection_set(0)  # Set the default selection to the first item
        return self.listbox

    def apply(self):
        selected_index = self.listbox.curselection()[0]
        self.selected_map = self.listbox.get(selected_index)

def show_popup(maps):
    root = tk.Tk()
    root.withdraw()  # Hide the root window
    dialog = MapSelectionDialog(root, maps)
    selected_map = dialog.selected_map
    root.quit()
    return selected_map

def handle_switch_map_interactively(req):
    maps = get_available_maps()
    selected_map = show_popup(maps)
    if selected_map == "new map":
        rospy.loginfo("new map being created")
        return SwitchMapInteractivelyResponse(success=True)
    elif selected_map in maps:
        map_file = os.path.join(MAPS_DIR, selected_map)
        rospy.set_param('map_file', map_file)
        success = switch_map(map_file)
        return SwitchMapInteractivelyResponse(success=success)
    else:
        rospy.logwarn("No valid map selected")
        return SwitchMapInteractivelyResponse(success=False)

def switch_map_interactive_server():
    rospy.init_node('switch_map_interactive_server')
    s = rospy.Service('switch_map_interactively', SwitchMapInteractively, handle_switch_map_interactively)
    rospy.loginfo("Ready to switch maps interactively.")
    rospy.spin()

if __name__ == "__main__":
    switch_map_interactive_server()
