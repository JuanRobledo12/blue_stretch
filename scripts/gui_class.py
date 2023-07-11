'''
Python Class of GUI that helps displaying the images stored in the waypoint_info.json

Last mod: 2023-Jun-30
Version: 2

'''

import tkinter as tk
from json_handler_class import JSON_Handler
from PIL import Image, ImageTk, ImageOps
import playsound
from gtts import gTTS
import os


class ImageGallery:
    def __init__(self, object_name, json_path='./waypoint_info.json'):
        
        self.handler = JSON_Handler(5, json_path, new_json=False)
        self.object_name = object_name
        
        self.raw_image_data = self.handler.get_images_with_object(self.object_name)
        self.num_images = len(self.raw_image_data)
        if self.num_images == 0:
            tts = gTTS(text='I apologize, but I couldn\'t find any matches for the {}.'.format(self.object_name), lang='en')
            tts.save('./stretch_audio_files/no_object.mp3')
            playsound.playsound('./stretch_audio_files/no_object.mp3', True)
            return
        else:
            object_in_gallery_msg = 'I\'ve found some photos of the {}. To help you locate it, I\'ll display the images on my screen'.format(self.object_name)
            tts = gTTS(text=object_in_gallery_msg, lang='en')
            tts.save('./stretch_audio_files/object_ingallery.mp3')
            playsound.playsound('./stretch_audio_files/object_ingallery.mp3', True)
        
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
        self.next_button = tk.Button(self.root, text='Next Image',font=("Arial",18),width=15,heigh=2,bg="medium purple",command=self.next_image)
        self.next_button.pack(side='top', pady=20)
        self.prev_button = tk.Button(self.root, text='Prev Image',font=("Arial",18),width=15, heigh=2,bg="medium purple",command=self.prev_image)
        self.prev_button.pack(side='top')
        self.select_button = tk.Button(self.root, text='Select Image',font=("Arial",18),width=15,heigh=2,bg="green yellow",command=self.select_image)
        self.select_button.pack(side='top', pady=20)
        

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
        image = Image.open(image_path).resize((600, 600), resample=Image.LANCZOS)
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

# test = ImageGallery('book', json_path='./waypoint_info.json')
# test.run()