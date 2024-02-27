import xml.etree.ElementTree as et


def add_wind(world_file, wind_speed):
    tree = et.parse(world_file)
    root = tree.getroot()
    plugin_elem = et.Element('plugin')
    plugin_elem.set('name', 'wind_plugin')
    plugin_elem.set('filename', 'libgazebo_wind_plugin.so')

    elements_to_add = [
        ('frameId', 'base_link'),
        ('robotNamespace', ''),
        ('windVelocityMean', str(wind_speed)),
        ('windVelocityMax',  str(wind_speed)),
        ('windVelocityVariance', '0'),
        ('windDirectionMean', '0 1 0'),
        ('windDirectionVariance', '0'),
        ('windGustStart', '0'),
        ('windGustDuration', '0'),
        ('windGustVelocityMean', '0'),
        ('windGustVelocityMax',  str(wind_speed)),
        ('windGustVelocityVariance', '0'),
        ('windGustDirectionMean', '1 0 0'),
        ('windGustDirectionVariance', '0'),
        ('windPubTopic', 'world_wind')
    ]

    for elem_name, elem_text in elements_to_add:
        elem = et.SubElement(plugin_elem, elem_name)
        elem.text = elem_text

    world_elem = root.find("world")
    world_elem.append(plugin_elem)

    tree.write(world_file, xml_declaration=True, encoding='utf-8')


def remove_wind(world_file):
    tree = et.parse(world_file)
    root = tree.getroot()
    world_elem = root.find("world")
    plugin_elem = world_elem.find("plugin")
    if plugin_elem is None:
        return
    world_elem.remove(plugin_elem)
    tree.write(world_file, xml_declaration=True, encoding='utf-8')


# add_wind("resources/simulation/worlds/simple_obstacle_scenario1.world", 10.0)
# remove_wind("resources/simulation/worlds/simple_obstacle_scenario1.world")
