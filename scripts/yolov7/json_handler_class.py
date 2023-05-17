######################## Description #######################
# Python Class to handle the waipoint_info.json file


####################### Version Info #######################
# Last mod: 2023-May-05
# Version: 2

import os
import json

MAX_IMAGE_STORAGE = 5
JSON_FILE_NAME = '/home/hello-robot/catkin_ws/src/blue_stretch/scripts/waypoint_info.json'

class JSON_Handler:
    
    def __init__(self, max_image_storage, json_file_name, new_json=False):
        self.max_image_storage = max_image_storage
        self.json_file_name = json_file_name
        self.waypoint_list = ["waypoint_1", "waypoint_2", "waypoint_3", "waypoint_4", "waypoint_5"]
        self.waypoint_names = ["table1", "couch", "table2","kitchen_counter","stove"]
        self.waypoint_positions = [[1.0441372394561768, 3.246427536010742,0],[0.7050193548202515, 2.021536350250244, 0],[0.1476496160030365, 1.8214967250823975, 0], [3.6145787239074707, 2.8259506225585938, 0], [4.306626796722412, 0.9125393033027649, 0]]
        self.waypoint_orientations = [[0, 0, 0.975745967055294, 0.2189059336229358],[0, 0, -0.3955405787212978, 0.918448501868679],[0, 0, -0.6137154103723945, 0.7895273238282785], [0, 0, 0.98866042236501, -0.15016846955682814], [0, 0, 0.19079321277213537, 0.9816302511435182]]
        if new_json:
            self.build_new_json()
        self.update_data()

    def build_new_json(self):
        #Clear old file
        if os.path.exists(self.json_file_name):
            os.remove(self.json_file_name)

        #Make and setup new json file
        with open(self.json_file_name, "w") as outfile:
            data = {}
            for i, waypoint in enumerate(self.waypoint_list):
                data[waypoint] = {
                    'name': self.waypoint_names[i],
                    'pose':{
                        'position': self.waypoint_positions[i],
                        'orientation': self.waypoint_orientations[i]},
                    'images':{}
                }
            
            # print(data)
            json.dump(data, outfile)
        print("json built")
    
    def update_data(self):
        self.data = self.json_to_dictionary()
        return self.data
    
    def display(self):
        for key, value in self.data.items():
            print(key, 'name:', value['name'], ' || pose:', value['pose'])
            for img_key, img_value in value['images'].items():
                # print('here')
                print(img_key,":", img_value)
    
    # Convert a json file to a dictionary
    def json_to_dictionary(self):
        with open(self.json_file_name) as json_file:
            dictionary = json.load(json_file)
        return dictionary

    # Convert a dictionary to a json file
    def dictionary_to_json(self, file_name, dictionary):
        with open(file_name, "w") as outfile:
            json.dump(dictionary, outfile)
        
    # Get image index
    def get_img_number(self, waypoint):
        num_stored = len(self.data[waypoint]['images'])
        if num_stored < MAX_IMAGE_STORAGE:
            return num_stored
        else:
            print('removed:', self.data[waypoint]['images']['0'])
            #pop queue and shift up
            for i in range(1, self.max_image_storage):
                self.data[waypoint]['images'][str(i-1)] = self.data[waypoint]['images'][str(i)]
            idx = self.max_image_storage - 1
            
            # print('should change', self.data[waypoint][str(idx)])

            return self.max_image_storage - 1
    
    # Add image data to a dictionary
    def add_image_data(self, waypoint, img_name, timestamp, objects, confidences):
        self.update_data()

        img_number = str(self.get_img_number(waypoint))
    
        self.data[waypoint]['images'][img_number] = {
            'name': img_name,
            'time': timestamp,
            'objects': objects,
            'confidences': confidences,
        }
            
        self.dictionary_to_json(self.json_file_name, self.data)
        print('Data Added')
    
    # Find all images that have a specific object in them
    def get_images_with_object(self, object_name, data=[], layer=0):
        if data == []:
            data = self.data
        
        output = []
        for waypoint in self.waypoint_list:
            images = data[waypoint]['images']
            for key, value in images.items():
                print('key:', key)
                print('value:', value)
                if object_name in value['objects']:
                    index = value['objects'].index(object_name) 
                    img_name = value['name']
                    timestamp = value['time']
                    object_pose = data[waypoint]['pose']
                    img_location = data[waypoint]['name']
                    confidence = value['confidences'][index]
                    output.append({
                        'path': img_name, 
                        'timestamp': timestamp, 
                        'location': img_location, 
                        'confidence': confidence,
                        'object_pose':object_pose,
                        })
        
        sorted_output = sorted(output, key=lambda x: int(x['timestamp'].replace("_", "")))
        sorted_output.reverse()
        return sorted_output
    
        
# ---------------
# TEST CODE
# ---------------

# handler = JSON_Handler(MAX_IMAGE_STORAGE, 'waypoint_info.json', new_json=True)
# handler.add_image_data('waypoint_1', 'img_0', '000000', ['dog', 'b', 'c'], [.9, .8, .7])
# handler.add_image_data('waypoint_2', 'img_0.jpg', '000000', ['dog', 'b', 'c'], [.9, .8, .7])
# handler.add_image_data('waypoint_2', 'img_1.jpg', '000001', ['e', 'b', 'c'], [.9, .8, .7])
# handler.add_image_data('waypoint_2', 'img_2.jpg', '000002', ['dog', 'b', 'c'], [.9, .8, .7])
# handler.add_image_data('waypoint_2', 'img_3.jpg', '000003', ['g', 'b', 'dog'], [.9, .8, .7])
# handler.add_image_data('waypoint_2', 'img_4.jpg', '000004', ['dog', 'b'], [.9, .8, .7])
# handler.add_image_data('waypoint_3', 'img_5.jpg', '000005', ['f', 'b', 'c'], [.9, .8, .7])

# handler.display()
# print('imgs w/ dog:', handler.get_images_with_object('dog'))

