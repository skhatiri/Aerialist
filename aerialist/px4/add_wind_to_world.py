import xml.etree.ElementTree as ET
import logging

logger = logging.getLogger(__name__)


def insert_wind_plugin(world_path, wind, at_top=True):
    tree = ET.parse(world_path)
    root = tree.getroot()

    # Find <world name="default">
    world_tag = None
    for elem in root.iter("world"):
        if elem.attrib.get("name") == "default":
            world_tag = elem
            break

    if world_tag is None:
        raise Exception("No <world name='default'> found.")

    # Remove existing wind plugin if it exists
    existing_plugin = None
    for plugin in world_tag.findall("plugin"):
        if plugin.attrib.get("name") == "wind_plugin":
            existing_plugin = plugin
            break

    if existing_plugin is not None:
        world_tag.remove(existing_plugin)

    if wind is None:
        if existing_plugin is not None:
            logger.debug(f"❌ Removed wind plugin from {world_path}")
        return  # Done: plugin was removed (if present), and no new one will be added

    # Create new wind plugin
    plugin = ET.Element("plugin", attrib={
        "name": "wind_plugin",
        "filename": "libgazebo_wind_plugin.so"
    })

    def add_tag(tag, text):
        el = ET.SubElement(plugin, tag)
        el.text = str(text)

    add_tag("frameId", "base_link")
    add_tag("robotNamespace", "")
    add_tag("windVelocityMean", wind.velocity_mean)
    add_tag("windVelocityMax", wind.velocity_max)
    add_tag("windVelocityVariance", wind.velocity_variance)
    add_tag("windDirectionMean", " ".join(map(str, wind.direction)))
    add_tag("windDirectionVariance", wind.direction_variance)
    add_tag("windGustStart", wind.gust_start)
    add_tag("windGustDuration", wind.gust_duration)
    add_tag("windGustVelocityMean", wind.gust_velocity_mean)
    add_tag("windGustVelocityMax", wind.gust_velocity_max)
    add_tag("windGustVelocityVariance", wind.gust_velocity_variance)
    add_tag("windGustDirectionMean", " ".join(map(str, wind.gust_direction)))
    add_tag("windGustDirectionVariance", wind.gust_direction_variance)
    add_tag("windPubTopic", "world_wind")

    # Insert plugin
    if at_top and len(world_tag):
        world_tag.insert(0, plugin)
    else:
        world_tag.append(plugin)

    tree.write(world_path, xml_declaration=True, encoding='utf-8')

    logger.debug(f"✅ Wind plugin inserted into {world_path} (position: {'top' if at_top else 'bottom'})")
