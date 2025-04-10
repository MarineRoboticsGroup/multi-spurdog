import csv
import numpy as np

def handle_surface_gap(depths, speeds):
    ssp_list = []
    for depth, speed in zip(depths, speeds):
        ssp_list.append([depth, speed])
    # Make sure that the list is sorted by depth
    ssp_list = sorted(ssp_list, key=lambda r: r[0])

    # Make sure that there is an entry for the surface (depth 0)
    if ssp_list[0][0] != 0:
        # just copy the shallowest measurement as depth 0.
        ssp_list.insert(0, [0, ssp_list[0][1]])
    return np.asarray(ssp_list)

def get_ssp_from_csv_file(filename: str) -> np.array:
    ''' Get a sound speed profile from a CSV file.
    This tries to do the right thing with a variety of files.  If passed a CSV with two columns and no header rows, it
    will assume that the first column is depth and the second is sound speed.
    If there are header rows, it skips through them until it finds a row that contains labels like 'depth' and
    'speed' or 'velocity'.  It then uses those columns.
    It does require that depth is in meters and speed is in meters per second.
    '''
    # Initialize an empty list to store the data
    ssp_list = []

    # Open the file in read mode
    with open(filename, 'r') as file:
        # Use csv reader to read the file
        reader = csv.reader(file)

        # Peek at the first row
        first_row = next(reader)

        if len(first_row) == 2 and all(s.replace('.', '', 1).isdigit() for s in first_row):
            # This is a data row, so this file doesn't have headers
            # Assume that the two columns are depth and speed, respectively
            depth_index, speed_index = 0, 1

            # Start processing rows immediately
            ssp_list.append([float(first_row[0]), float(first_row[1])])
        else:
            # This file has a header, so find it first
            for row in reader:
                # Skip the line if it's empty
                if not row:
                    continue

                # Check if this row contains the column names
                if any("depth" in s.lower() for s in row) and any("velocity" in s.lower() or "speed" in s.lower() for s in row):
                    # This is the header row! Store the column indices
                    depth_index = next((i for i, s in enumerate(row) if "depth" in s.lower()), None)
                    speed_index = next((i for i, s in enumerate(row) if "velocity" in s.lower() or "speed" in s.lower()), None)

                    # Break the loop to stop skipping rows
                    break

        # Now that we have the correct indices, process the remaining rows
        for row in reader:
            # Extract depth and sound speed values
            depth = abs(float(row[depth_index]))
            speed = float(row[speed_index])

            # Append values to the list as a new sublist
            ssp_list.append([depth, speed])

    # Make sure that the list is sorted by depth
    ssp_list = sorted(ssp_list, key=lambda r: r[0])

    # Make sure that there is an entry for the surface (depth 0)
    if ssp_list[0][0] != 0:
        # just copy the shallowest measurement as depth 0.
        ssp_list.insert(0, [0, ssp_list[0][1]])

    # Return the final list
    return np.asarray(ssp_list)
