# This is a script to convert the pick and place file to a coordinates map
# This code is very much not optimized, but it does its job.

import math

inputFile = "../pick-and-place.csv"
outputFile = "../pick-and-place-coordinates.csv"

lines = []

with open(inputFile, 'r') as file:
    for line in file:
        if line.strip():  # Check if the line is not empty
            lines.append(line.strip())

x0 = float(lines[0].split(',')[3])
y0 = float(lines[0].split(',')[4])

coordinates = []
for line in lines:
    parts = line.split(',')
    if len(parts) >= 2:
        x = float(parts[3]) - x0
        y = float(parts[4]) - y0
        coordinates.append((x, y))

# Convert to spherical coordinates
spherical_coordinates = []
for x, y in coordinates:
    r = (x**2 + y**2)**0.5
    phi = math.atan2(y, x)  # angle in radians
    if phi < 0:
        phi += 2 * math.pi  # Normalize angle to [0, 2π)
    spherical_coordinates.append((r, phi))

# scale r from 0 to pi
scaled_coordinates = []
for r, phi in spherical_coordinates:
    scaled_r = r / max(r for r, _ in spherical_coordinates) * math.pi
    scaled_coordinates.append((round(scaled_r, 2), round(phi, 2)))

# Write to output file
with open(outputFile, 'w') as file:
    for theta, phi in scaled_coordinates:
        file.write(f"Coordinates({theta}, {phi}),\n")
print(f"Coordinates written to {outputFile}")
