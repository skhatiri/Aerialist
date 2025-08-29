from typing import Tuple
import xml.etree.ElementTree as ET
import logging

logger = logging.getLogger(__name__)


class Wind:
    def __init__(self,
                 velocity_mean: float,
                 velocity_max: float,
                 direction: Tuple[float, float, float],
                 velocity_variance: float = 0,
                 direction_variance: float = 0,
                 gust_start: float = 0,
                 gust_duration: float = 0,
                 gust_velocity_mean: float = 0,
                 gust_velocity_max: float = 0,
                 gust_velocity_variance: float = 0,
                 gust_direction: Tuple[float, float, float] = (0, 0, 0),
                 gust_direction_variance: float = 0):
        
        self.velocity_mean = velocity_mean
        self.velocity_max = velocity_max
        self.direction = direction
        self.velocity_variance = velocity_variance
        self.direction_variance = direction_variance
        self.gust_start = gust_start
        self.gust_duration = gust_duration
        self.gust_velocity_mean = gust_velocity_mean
        self.gust_velocity_max = gust_velocity_max
        self.gust_velocity_variance = gust_velocity_variance
        self.gust_direction = gust_direction
        self.gust_direction_variance = gust_direction_variance

    def to_dict(self):
        return {
            "velocity_mean": self.velocity_mean,
            "velocity_max": self.velocity_max,
            "velocity_variance": self.velocity_variance,
            "direction": list(self.direction),
            "direction_variance": self.direction_variance,
            "gust_start": self.gust_start,
            "gust_duration": self.gust_duration,
            "gust_velocity_mean": self.gust_velocity_mean,
            "gust_velocity_max": self.gust_velocity_max,
            "gust_velocity_variance": self.gust_velocity_variance,
            "gust_direction": list(self.gust_direction),
            "gust_direction_variance": self.gust_direction_variance,
        }

    def insert_wind_plugin(self, world_path, at_top=True):
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

        if self is None:
            if existing_plugin is not None:
                logger.info(f"❌ Removed wind plugin from {world_path}")
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
        add_tag("windVelocityMean", self.velocity_mean)
        add_tag("windVelocityMax", self.velocity_max)
        add_tag("windVelocityVariance", self.velocity_variance)
        add_tag("windDirectionMean", " ".join(map(str, self.direction)))
        add_tag("windDirectionVariance", self.direction_variance)
        add_tag("windGustStart", self.gust_start)
        add_tag("windGustDuration", self.gust_duration)
        add_tag("windGustVelocityMean", self.gust_velocity_mean)
        add_tag("windGustVelocityMax", self.gust_velocity_max)
        add_tag("windGustVelocityVariance", self.gust_velocity_variance)
        add_tag("windGustDirectionMean", " ".join(map(str, self.gust_direction)))
        add_tag("windGustDirectionVariance", self.gust_direction_variance)
        add_tag("windPubTopic", "world_wind")

        # Insert plugin
        if at_top and len(world_tag):
            world_tag.insert(0, plugin)
        else:
            world_tag.append(plugin)

        tree.write(world_path, xml_declaration=True, encoding='utf-8')

        logger.info(f"✅ Wind plugin inserted into {world_path} (position: {'top' if at_top else 'bottom'})")
