import json

f = open("waypoints.json")
location_data = json.load(f)

locations = []
locations_name = []

for loc in location_data.keys():
    print(type(location_data[loc]["Pose"]), location_data[loc]["Pose"])
    print(type(location_data[loc]["Orientation"]), location_data[loc]["Orientation"])
    locations.append([Pose(Point(location_data[loc]["Pose"]), Quaternion(location_data[loc]["Orientation"]))])
    locations_name.append(location_data[loc]["Name"])
