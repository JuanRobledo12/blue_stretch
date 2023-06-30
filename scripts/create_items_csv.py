import csv

items = ['book', 'wallet', 'pencil']
directory = './stretch_misc_files/'  # Update this with the desired directory path
filename = 'items.csv'
filepath = directory + filename

# Open the CSV file in write mode
with open(filepath, 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)

    # Write the header row
    writer.writerow(['Item'])

    # Write the items
    for item in items:
        writer.writerow([item])

print(f"{filename} created successfully in {directory}!")


