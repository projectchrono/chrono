class shapes_index():
    SPHERE = 0
    ELLIPSOID = 1
    BOX = 2
    CYLINDER = 3
    CONVEXHULL = 4
    TRIANGLEMESH = 5
    BARREL = 6
    CAPSULE = 7
    CONE = 8
    ROUNDEDBOX = 9
    ROUNDEDCYL = 10
    BEZIER = 11


def import_shapes(filepath):

    shapes_data = []
    
    with open(filepath, 'r') as FH:
        lines = FH.readlines()

        # Read number of bodies, visual assets, joints, and TSDA elements (first line)
        line = lines[0].strip().split(',')
        num_bodies = int(line[0])
        num_assets = int(line[1])
        ##print ("File: ", filepath, "  Num bodies: ", num_bodies, "  Num assets: ", num_assets)

        # Read only visual assets
        for il in range(1 + num_bodies, 1 + num_bodies + num_assets):
            line = lines[il].strip().split(',')
            
            # Extract information common to all assets
            body_id, active,  x, y, z, e0, e1, e2, e3, r, g, b, shape_type = line[:13:]
            data = [int(shape_type), int(body_id), active,  float(x), float(y), float(z), float(e0), float(e1), float(e2), float(e3), float(r), float(g), float(b)]
            
            # Read asset-specific data
            if int(shape_type) == shapes_index.TRIANGLEMESH:
                try:
                    path = line[13]
                    data.extend([path.strip('"')])
                except:
                    print("Record: ", lines[il])
                    raise Exception("Failed while trying to parse trimesh data.")
            elif int(shape_type) == shapes_index.SPHERE:
                try:
                    rad = line[13]
                    data.extend([float(rad)])
                except:
                    print("Record: ", lines[il])
                    raise Exception("Failed while trying to parse sphere data.")
            elif int(shape_type) == shapes_index.BOX:
                try:
                    size_x, size_y, size_z = line[13:16:]
                    data.extend([float(size_x), float(size_y), float(size_z)])
                except:
                    print("Record: ", lines[il])
                    raise Exception("Failed while trying to parse box data.")
            elif int(shape_type) == shapes_index.ELLIPSOID:
                try:
                    size_x, size_y, size_z = line[13:16:]
                    data.extend([float(size_x), float(size_y), float(size_z)])
                except:
                    print("Record: ", lines[il])
                    raise Exception("Failed while trying to parse ellipsoid data.")
            elif int(shape_type) == shapes_index.CYLINDER:
                try:
                    rad, p1x, p1y, p1z, p2x, p2y, p2z = line[13:20:]
                    data.extend([float(rad), float(p1x), float(p1y), float(p1z), float(p2x), float(p2y), float(p2z)])
                except:
                    print("Record: ", lines[il])
                    raise Exception("Failed while trying to parse cyilinder data.")    
            else:
                print('Unsupported shape type')
                continue
            
            # Append to list of shape data
            shapes_data.append(data)


    return shapes_data

if __name__ == "__main__":
    data = import_shapes('data1.dat')
    print(data)