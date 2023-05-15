######################## Description #######################
# Python Class of GUI that helps displaying the images stored in the waypoint_info.json


####################### Version Info #######################
# Last mod: 2023-May-05
# Version: 2

import tkinter as tk
from json_handler_class_2 import JSON_Handler
from PIL import ImageTk, Image
import os

class ImageGallery:
    def __init__(self, object_name, json_path='/home/hello-robot/catkin_ws/src/blue_stretch/scripts/waypoint_info.json'):
        # self.images = images
        # initialize JSON Handler --> (max list amt, name of json, change to false)
        self.handler = JSON_Handler(5, json_path, new_json=False)
        self.object_name = object_name
        
        self.raw_image_data = self.handler.get_images_with_object(self.object_name)
        self.num_images = len(self.raw_image_data)
        if self.num_images == 0:
            print("Sorry, I have not seen it")
            return
        self.current_image_index = 0
        
        # Create a Tkinter window
        self.root = tk.Tk()
        self.root.title('Image Gallery')
        self.root.geometry("950x400")
        # Create a label for the image
        self.image_label = tk.Label(self.root)
        self.image_label.pack(side='left')

        # Create labels for the title and timestamp
        self.title_label = tk.Label(self.root, font=('Arial', 24, 'bold'), justify=tk.CENTER)
        self.title_label.pack()
        self.timestamp_label = tk.Label(self.root, font=('Arial', 18), justify=tk.CENTER)
        self.timestamp_label.pack()
        self.location_label = tk.Label(self.root, font=('Arial', 18), justify=tk.CENTER)
        self.location_label.pack()

        # Create buttons to navigate the images
        self.prev_button = tk.Button(self.root, text='<<',font=("Arial",18),width=6, heigh=2,bg="red",command=self.prev_image)
        self.prev_button.pack(side='left')
        self.select_button = tk.Button(self.root, text='Select Image',font=("Arial",18),width=15,heigh=2,bg="light blue",command=self.select_image)
        self.select_button.pack(side='left')
        self.next_button = tk.Button(self.root, text='>>',font=("Arial",18),width=6,heigh=2,bg="green",command=self.next_image)
        self.next_button.pack(side='left')

        # Display the first image
        self.show_image(self.raw_image_data[self.current_image_index])

        #info to return to main program
        self.object_location = None
        self.object_pose = None

    def show_image(self, image_data):
        title_text_label = f"Is this your {self.object_name}?"
        self.title_label.config(text=title_text_label)
        
        timestamp = image_data['timestamp']
        h, m, s = timestamp[:2], timestamp[3:5], timestamp[6:]
        timestamp_text_label = f"When I saw this: {h}:{m}:{s}"
        self.timestamp_label.config(text=timestamp_text_label)
        
        location_text_label = f"Where I saw this: {image_data['location']}"
        self.location_label.config(text=location_text_label)
        
        image_path = image_data['path']
        image = Image.open(image_path).resize((400, 400), Image.ANTIALIAS)
        photo = ImageTk.PhotoImage(image)
        self.image_label.config(image=photo)
        self.image_label.image = photo

    def prev_image(self):
        self.current_image_index = (self.current_image_index - 1) % self.num_images
        self.show_image(self.raw_image_data[self.current_image_index])

    def next_image(self):
        self.current_image_index = (self.current_image_index + 1) % self.num_images
        self.show_image(self.raw_image_data[self.current_image_index])
    
    def select_image(self):
        image_data = self.raw_image_data[self.current_image_index]
        image_path = image_data['path']
        self.object_location = image_data['location']
        self.object_pose = image_data['object_pose']
        #print(image_path, self.object_location, self.object_pose)
        #Add code here to save the image to a file or perform any other desired action
        #return image_location, image_pose
        #print(type(image_location), type(image_pose))
        self.root.destroy()

    def run(self):
        # Start the Tkinter event loop
        if self.num_images == 0:
            return
        self.root.mainloop()
        print('I was killed')
        return self.object_location, self.object_pose
# ----------------
# TEST CODE   
# ----------------

test = ImageGallery('cell phone', json_path='/home/hello-robot/catkin_ws/src/blue_stretch/scripts/waypoint_info.json')
test.run()