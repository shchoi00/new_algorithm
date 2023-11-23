def read_pcd_file(filepath):
    with open(filepath, 'r') as file:
        lines = file.readlines()

    # Extract the header and point data
    header = []
    points = []
    read_header = True
    for line in lines:
        if read_header:
            if line == 'FIELDS z y x':
                line = 'FIELDS x y z'
            elif line.startswith('SIZE') and len(line) == 12:
                line = line[:-1]+'\n'
            elif line.startswith('TYPE') and len(line) == 12:
                line = line[:-1]+'\n'
            elif line.startswith('COUNT') and len(line) == 12:
                line = line[:-1]+'\n'
            header.append(line)
            if line.startswith('DATA'):
                read_header = False
        else:
            points.append(line.strip().split())

    return header, points

def write_pcd_file(filepath, header, points):
    with open(filepath, 'w') as file:
        for line in header:
            file.write(line)
        for point in points:
            file.write(' '.join(point) + '\n')

def convert_pcd_axes(input_filepath, output_filepath):
    # Read the PCD file
    header, points = read_pcd_file(input_filepath)

    # # Change the order of the FIELDS in the header
    # for i, line in enumerate(header):
    #     if line.startswith('FIELDS'):
    #         fields = line.strip().split()
    #         # New order: x to z, y to x, z to y
    #         fields[1], fields[2], fields[3] = fields[3], fields[1], fields[2]
    #         header[i] = ' '.join(fields) + '\n'
    #         break

    # Change the XYZ to ZXY for each point
    converted_points = []
    for point in points:
        if len(point) == 3:
            x, y, z = point
        elif len(point) == 4:
            x, y, z, _ = point
        # Reorder to ZXY (x to z, y to x, z to y)
        converted_points.append([z, x, y])

    # Write the updated PCD file
    write_pcd_file(output_filepath, header, converted_points)
    print(f"Converted PCD file saved to '{output_filepath}'")

# Use the function to convert a PCD file
input_pcd_file = './vision_tower_B3_05.pcd'  # Replace with the path to your input PCD file
output_pcd_file = f'./converted_{input_pcd_file[2:]}'  # Replace with the desired path for your output PCD file
convert_pcd_axes(input_pcd_file, output_pcd_file)
