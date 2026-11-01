"""Generate world-frame RViz displays for every configured aircraft."""

import copy
import yaml

COLORS = ["255; 230; 100", "70; 190; 255", "255; 150; 180"]


def prepare_rviz(root, output, specs):
    config = yaml.safe_load(
        (root / "Simulation/px4_forest/config/forest.rviz").read_text()
    )
    manager = config["Visualization Manager"]
    template = manager["Displays"]
    displays = copy.deepcopy(template[:2])  # Shared grid and display-only forest.
    for index, vehicle in enumerate(specs):
        color = COLORS[index % len(COLORS)]
        children = []
        displays.append({
            "Class": "rviz/Group", "Name": vehicle.name,
            "Enabled": True, "Displays": children,
        })
        cloud = copy.deepcopy(template[2])
        cloud.update(
            Name=vehicle.name + " world cloud",
            Topic=vehicle.namespace + "/world/cloud",
            Color=color,
            Enabled=False,
        )
        children.append(cloud)
        children.append(
            {
                "Class": "rviz/Odometry",
                "Name": vehicle.name + " world odometry",
                "Enabled": True,
                "Topic": vehicle.namespace + "/world/odom",
                "Keep": 1,
                "Shape": "Arrow",
                "Color": color,
                "Position Tolerance": 0.05,
                "Angle Tolerance": 0.1,
            }
        )
        for suffix, label in (
            ("optimal_list", "planned trajectory"),
            ("global_list", "global reference"),
        ):
            children.append(
                {
                    "Class": "rviz/Marker",
                    "Name": vehicle.name + " " + label,
                    "Enabled": True,
                    "Marker Topic": "/uav_%d_ego_planner_node/%s"
                    % (vehicle.uav_id, suffix),
                    "Queue Size": 10,
                }
            )
        children.append({
            "Class": "rviz/MarkerArray", "Name": vehicle.name + " requested waypoints",
            "Enabled": True, "Marker Topic": "/sim/view/" + vehicle.name + "/goals",
            "Queue Size": 10,
        })
        children.append({
            "Class": "rviz/MarkerArray", "Name": vehicle.name + " label",
            "Enabled": True, "Marker Topic": "/sim/view/" + vehicle.name + "/vehicle",
            "Queue Size": 10,
        })
    manager["Displays"] = displays
    content = yaml.safe_dump(config, sort_keys=False)
    (root / "Simulation/px4_forest/config/fleet.rviz").write_text(content)
    # Keep each running fleet isolated from later config regeneration.
    path = output / "fleet.rviz"
    path.write_text(content)
    return path


def route_markers(goals, uav_id):
    """Display-only requested world waypoints; never feed navigation."""
    from visualization_msgs.msg import Marker, MarkerArray

    color = [int(v.strip()) / 255.0 for v in COLORS[(uav_id - 1) % len(COLORS)].split(";")]
    array = MarkerArray()
    for index, position in enumerate(goals):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.ns = "requested_waypoints"
        marker.id = index
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = position
        marker.scale.x = marker.scale.y = marker.scale.z = 0.22
        marker.color.r, marker.color.g, marker.color.b = color
        marker.color.a = 1
        array.markers.append(marker)
    return array


def vehicle_label(name, position, uav_id, stamp):
    """Display-only name above the aircraft's world-frame position."""
    from visualization_msgs.msg import Marker, MarkerArray

    marker = Marker()
    marker.header.frame_id = "world"
    marker.header.stamp = stamp
    marker.ns = "vehicle_label"
    marker.id = uav_id
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD
    marker.text = name
    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = position
    marker.pose.position.z += 0.6
    marker.pose.orientation.w = 1
    marker.scale.z = 0.3
    marker.color.r, marker.color.g, marker.color.b = [
        int(v.strip()) / 255.0 for v in COLORS[(uav_id - 1) % len(COLORS)].split(";")
    ]
    marker.color.a = 1
    return MarkerArray(markers=[marker])
