import xml.etree.ElementTree as et


def modify_light(world_file, intensity=0.4):
    tree = et.parse(world_file)
    root = tree.getroot()
    world_elem = root.find("world")
    scene_elem = world_elem.find("scene")
    ambient_elem = scene_elem.find("ambient")

    print(ambient_elem.text)
    ambient_elem.text = f"{intensity} {intensity} {intensity} 1"
    print(ambient_elem.text)
    tree.write(world_file, xml_declaration=True, encoding='utf-8')


# modify_light("resources/simulation/worlds/simple_obstacle_scenario1.world", 0.1)
