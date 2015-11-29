
def get_blocks():
    f = open("/home/dan/catkin_ws/src/Botlom/map/tutorial.world", 'r')
    blocks = []

    for line in f:
        line = line.rstrip()
        if line[0:5] == "block" and line <> "":
            split =  line.split(" ")
            blocks.append([float(split[3]), float(split[4]), float(split[5]), split[-1][1:-2]])

    f.close()
    return  blocks

