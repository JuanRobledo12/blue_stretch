import json

f = open("waypoint_info.json")

location_data = json.load(f)

locations = []
locations_name = []

# saving the locations in a list
for loc in location_data.keys():
    locations.append([location_data[loc]["pose"]["position"],location_data[loc]["pose"]["orientation"]])
    locations_name.append(location_data[loc]["name"])

print(locations)
print(locations_name)
