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
    ROUNDEDCONE = 11
    BEZIER = 12


def import_shapes(filepath):

    shapes_data = []
    
    with open(filepath, 'r') as FH:
        for file in FH.readlines():
            line = file.strip()
            line_data = line.split(',')
            # Check that the POVRayShapeType is a number
            if len(line_data) < 13 or not line_data[12].isdigit():
                continue
            if int(line_data[12]) == shapes_index.TRIANGLEMESH:
                try:
                    body_ind, active,  x, y, z, e0, e1, e2, e3, r, g, b, shape_tpye, path, _  = line_data
                    shapes_data.append([int(shape_tpye), int(body_ind), active,  float(x), float(y), float(z), float(e0), float(e1), float(e2), float(e3), 
                                        float(r), float(g), float(b), path.strip('"')])
                except:
                    print ("Record: ", line)
                    raise Exception("Failed while trying to parse trimesh data.")
            
            elif int(line_data[12]) == shapes_index.SPHERE:
                try:
                    body_ind, active,  x, y, z, e0, e1, e2, e3, r, g, b, shape_tpye, rad, _  = line_data
                    shapes_data.append([int(shape_tpye), int(body_ind), active,  float(x), float(y), float(z), float(e0), float(e1), float(e2), float(e3), float(r), float(g), float(b), float(rad)])
                except:
                    print ("Record: ", line)
                    raise Exception("Failed while trying to parse sphere data.")
                    
            elif int(line_data[12]) == shapes_index.BOX:
                try:
                    body_ind, active,  x, y, z, e0, e1, e2, e3, r, g, b, shape_tpye, size_x, size_y, size_z, _  = line_data
                    shapes_data.append([int(shape_tpye), int(body_ind), active,  float(x), float(y), float(z), float(e0), float(e1), float(e2), float(e3), 
                                        float(r), float(g), float(b), float(size_x), float(size_y), float(size_z)])
                except:
                    print ("Record: ", line)
                    raise Exception("Failed while trying to parse box data.")
                    
            elif int(line_data[12]) == shapes_index.ELLIPSOID:
                try:
                    body_ind, active,  x, y, z, e0, e1, e2, e3, r, g, b, shape_tpye, size_x, size_y, size_z, _  = line_data
                    shapes_data.append([int(shape_tpye), int(body_ind), active,  float(x), float(y), float(z), float(e0), float(e1), float(e2), float(e3), 
                                        float(r), float(g), float(b), float(size_x), float(size_y), float(size_z)])
                except:
                    print ("Record: ", line)
                    raise Exception("Failed while trying to parse Ellipsoid data.")
                    
            elif int(line_data[12]) == shapes_index.CYLINDER:
                try:
                    body_ind, active,  x, y, z, e0, e1, e2, e3, r, g, b, shape_tpye, rad, p1x, p1y, p1z, p2x, p2y, p2z, _  = line_data
                    shapes_data.append([int(shape_tpye), int(body_ind), active,  float(x), float(y), float(z), float(e0), float(e1), float(e2), float(e3), 
                                        float(r), float(g), float(b), float(rad), float(p1x), float(p1y), float(p1z), float(p2x), float(p2y), float(p2z)])
                except:
                    print ("Record: ", line)
                    raise Exception("Failed while trying to parse cyilinder data.")    
                    
                    
            else:
                print('Unsupported shape type')
            
    
    return shapes_data

if __name__ == "__main__":
    data = import_shapes('data1.dat')
    print(data)