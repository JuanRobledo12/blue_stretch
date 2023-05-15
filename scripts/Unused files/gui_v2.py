import tkinter as tk
from PIL import ImageTk, Image
import os

# Define a list of images with their titles and timestamps
images = [
    {'title': 'Image 1', 'timestamp': '2023-03-31 11:30:00', 'file': 'keys.jpg'},
    {'title': 'Image 2', 'timestamp': '2023-03-31 10:35:00', 'file': 'mouse.jpg'},
    {'title': 'Image 3', 'timestamp': '2023-03-31 10:23:00', 'file': 'person_phones.jpg'},
    #{'title': 'Image 4', 'timestamp': '2022-01-04 12:00:00', 'file': 'image4.jpg'},
]
# Define functions to navigate to the previous and next images
current_image_index = 0
def prev_image():
    global current_image_index
    current_image_index -= 1
    if current_image_index < 0:
        current_image_index = len(images) - 1
    show_image(images[current_image_index])

def next_image():
    global current_image_index
    current_image_index += 1
    if current_image_index >= len(images):
        current_image_index = 0
    show_image(images[current_image_index])
# Define a function to display the image and its details
def show_image(image_data):
    title_label.config(text=image_data['title'])
    timestamp_label.config(text=image_data['timestamp'])
    image_path = os.path.join('images', image_data['file'])
    image = Image.open(image_path)
    image = image.resize((400, 400), Image.ANTIALIAS)
    photo = ImageTk.PhotoImage(image)
    image_label.config(image=photo)
    image_label.image = photo

# Define a function to save the current image and close the window
def select_image():
    global current_image_index
    image_data = images[current_image_index]
    image_path = os.path.join('images', image_data['file'])
    # Add code here to save the image to a file or perform any other desired action
    root.destroy()

# Create a Tkinter window
root = tk.Tk()
root.title('Image Gallery')

# Create a label for the image
image_label = tk.Label(root)
image_label.pack()

# Create labels for the title and timestamp
title_label = tk.Label(root, font=('Arial', 20, 'bold'))
title_label.pack()
timestamp_label = tk.Label(root, font=('Arial', 16))
timestamp_label.pack()

# Create buttons to navigate the images
prev_button = tk.Button(root, text='<<', bg = 'green',command=prev_image)
prev_button.pack(side='left')
next_button = tk.Button(root, text='>>', bg = 'green',command=next_image)
next_button.pack(side='right')

# Create a button to select the current image
select_button = tk.Button(root, text='This is my object!', bg = 'red',command=select_image)
select_button.pack()

# Define functions to navigate to the previous and next images
current_image_index = 0
def prev_image():
    global current_image_index
    current_image_index -= 1
    if current_image_index < 0:
        current_image_index = len(images) - 1
    show_image(images[current_image_index])

def next_image():
    global current_image_index
    current_image_index += 1
    if current_image_index >= len(images):
        current_image_index = 0
    show_image(images[current_image_index])

# Display the first image
show_image(images[current_image_index])

# Start the Tkinter event loop
root.mainloop()