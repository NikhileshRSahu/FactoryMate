from __future__ import annotations

from xml.etree import ElementTree as ET


def _txt(parent: ET.Element, tag: str, text: str) -> ET.Element:
    child = ET.SubElement(parent, tag)
    child.text = text
    return child


def _pose(parent: ET.Element, xyzrpy: tuple[float, float, float, float, float, float]) -> None:
    _txt(parent, 'pose', ' '.join(f'{v:.4f}' for v in xyzrpy))


def _material(visual: ET.Element, rgba: tuple[float, float, float, float]) -> None:
    material = ET.SubElement(visual, 'material')
    rgb = ' '.join(f'{v:.3f}' for v in rgba)
    _txt(material, 'ambient', rgb)
    _txt(material, 'diffuse', rgb)
    _txt(material, 'specular', '0.12 0.12 0.12 1.0')


def _box_geometry(parent: ET.Element, size: tuple[float, float, float]) -> None:
    geometry = ET.SubElement(parent, 'geometry')
    box = ET.SubElement(geometry, 'box')
    _txt(box, 'size', ' '.join(f'{v:.4f}' for v in size))


def _box_link(
    model: ET.Element,
    name: str,
    size: tuple[float, float, float],
    pose: tuple[float, float, float, float, float, float],
    color: tuple[float, float, float, float],
    collision: bool = True,
) -> ET.Element:
    link = ET.SubElement(model, 'link', {'name': name})
    _pose(link, pose)
    if collision:
        collision_el = ET.SubElement(link, 'collision', {'name': 'collision'})
        _box_geometry(collision_el, size)
    visual = ET.SubElement(link, 'visual', {'name': 'visual'})
    _box_geometry(visual, size)
    _material(visual, color)
    return link


def _static_box_model(
    world: ET.Element,
    name: str,
    size: tuple[float, float, float],
    pose: tuple[float, float, float, float, float, float],
    color: tuple[float, float, float, float],
) -> ET.Element:
    model = ET.SubElement(world, 'model', {'name': name})
    _txt(model, 'static', 'true')
    _pose(model, pose)
    _box_link(model, 'body', size, (0, 0, 0, 0, 0, 0), color)
    return model


def _add_rack(world: ET.Element, name: str, x: float, y: float) -> None:
    model = ET.SubElement(world, 'model', {'name': name})
    _txt(model, 'static', 'true')
    _pose(model, (x, y, 0, 0, 0, 0))

    # One coarse collision volume keeps navigation/physics cheap.
    collision_link = ET.SubElement(model, 'link', {'name': 'rack_collision'})
    _pose(collision_link, (0, 0, 1.2, 0, 0, 0))
    collision = ET.SubElement(collision_link, 'collision', {'name': 'collision'})
    _box_geometry(collision, (3.6, 0.9, 2.4))

    steel = (0.16, 0.23, 0.30, 1.0)
    shelf = (0.72, 0.47, 0.16, 1.0)
    for idx, px in enumerate((-1.72, 1.72)):
        for side, py in enumerate((-0.39, 0.39)):
            _box_link(model, f'post_{idx}_{side}', (0.08, 0.08, 2.4), (px, py, 1.2, 0, 0, 0), steel, collision=False)
    for level, z in enumerate((0.18, 1.05, 1.92)):
        _box_link(model, f'shelf_{level}', (3.55, 0.85, 0.09), (0, 0, z, 0, 0, 0), shelf, collision=False)
        # Pallet-box silhouettes for visual realism without physics overhead.
        for bx in (-1.1, 0.0, 1.1):
            _box_link(model, f'box_{level}_{bx:+.1f}', (0.65, 0.62, 0.52), (bx, 0, z + 0.30, 0, 0, 0), (0.55, 0.32, 0.16, 1.0), collision=False)


def _cylinder_geometry(parent: ET.Element, radius: float, length: float) -> None:
    geometry = ET.SubElement(parent, 'geometry')
    cyl = ET.SubElement(geometry, 'cylinder')
    _txt(cyl, 'radius', f'{radius:.4f}')
    _txt(cyl, 'length', f'{length:.4f}')


def _sphere_geometry(parent: ET.Element, radius: float) -> None:
    geometry = ET.SubElement(parent, 'geometry')
    sphere = ET.SubElement(geometry, 'sphere')
    _txt(sphere, 'radius', f'{radius:.4f}')


def _worker_model(world: ET.Element, index: int, x: float, y: float, yaw: float) -> None:
    name = f'worker_{index:02d}'
    model = ET.SubElement(world, 'model', {'name': name})
    _pose(model, (x, y, 0, 0, 0, yaw))
    _txt(model, 'self_collide', 'false')

    base = ET.SubElement(model, 'link', {'name': 'base_link'})
    _pose(base, (0, 0, 0.42, 0, 0, 0))
    inertial = ET.SubElement(base, 'inertial')
    _txt(inertial, 'mass', '45.0')
    inertia = ET.SubElement(inertial, 'inertia')
    _txt(inertia, 'ixx', '3.0'); _txt(inertia, 'iyy', '3.0'); _txt(inertia, 'izz', '2.0')
    collision = ET.SubElement(base, 'collision', {'name': 'body_collision'})
    _cylinder_geometry(collision, 0.28, 1.55)
    visual = ET.SubElement(base, 'visual', {'name': 'torso'})
    _cylinder_geometry(visual, 0.25, 1.25)
    _material(visual, (0.12 + 0.05 * (index % 4), 0.36, 0.62, 1.0))

    head_visual = ET.SubElement(base, 'visual', {'name': 'head'})
    _pose(head_visual, (0, 0, 0.88, 0, 0, 0))
    _sphere_geometry(head_visual, 0.18)
    _material(head_visual, (0.80, 0.62, 0.48, 1.0))

    for side, yoff in [('left', 0.27), ('right', -0.27)]:
        wheel = ET.SubElement(model, 'link', {'name': f'{side}_wheel'})
        _pose(wheel, (0, yoff, 0.12, 1.5708, 0, 0))
        inertial = ET.SubElement(wheel, 'inertial')
        _txt(inertial, 'mass', '1.0')
        inertia = ET.SubElement(inertial, 'inertia')
        _txt(inertia, 'ixx', '0.02'); _txt(inertia, 'iyy', '0.02'); _txt(inertia, 'izz', '0.02')
        col = ET.SubElement(wheel, 'collision', {'name': 'collision'})
        _cylinder_geometry(col, 0.11, 0.05)
        vis = ET.SubElement(wheel, 'visual', {'name': 'visual'})
        _cylinder_geometry(vis, 0.11, 0.05)
        _material(vis, (0.05, 0.05, 0.05, 1.0))
        joint = ET.SubElement(model, 'joint', {'name': f'{side}_wheel_joint', 'type': 'revolute'})
        _txt(joint, 'parent', 'base_link')
        _txt(joint, 'child', f'{side}_wheel')
        axis = ET.SubElement(joint, 'axis')
        _txt(axis, 'xyz', '0 1 0')
        limit = ET.SubElement(axis, 'limit')
        _txt(limit, 'lower', '-1e16'); _txt(limit, 'upper', '1e16'); _txt(limit, 'effort', '20'); _txt(limit, 'velocity', '12')

    caster = ET.SubElement(model, 'link', {'name': 'caster'})
    _pose(caster, (-0.20, 0, 0.08, 0, 0, 0))
    caster_col = ET.SubElement(caster, 'collision', {'name': 'collision'})
    _sphere_geometry(caster_col, 0.07)
    caster_vis = ET.SubElement(caster, 'visual', {'name': 'visual'})
    _sphere_geometry(caster_vis, 0.07)
    _material(caster_vis, (0.08, 0.08, 0.08, 1.0))
    caster_joint = ET.SubElement(model, 'joint', {'name': 'caster_joint', 'type': 'ball'})
    _txt(caster_joint, 'parent', 'base_link')
    _txt(caster_joint, 'child', 'caster')

    plugin = ET.SubElement(model, 'plugin', {'filename': 'gz-sim-diff-drive-system', 'name': 'gz::sim::systems::DiffDrive'})
    _txt(plugin, 'left_joint', 'left_wheel_joint')
    _txt(plugin, 'right_joint', 'right_wheel_joint')
    _txt(plugin, 'wheel_separation', '0.54')
    _txt(plugin, 'wheel_radius', '0.11')
    _txt(plugin, 'topic', f'/model/{name}/cmd_vel')
    _txt(plugin, 'odom_topic', f'/model/{name}/odom')
    _txt(plugin, 'tf_topic', f'/model/{name}/tf')
    _txt(plugin, 'max_linear_acceleration', '1.2')
    _txt(plugin, 'max_angular_acceleration', '2.0')


def build_world(worker_count: int = 9) -> str:
    if worker_count < 0 or worker_count > 20:
        raise ValueError('worker_count must be between 0 and 20')

    sdf = ET.Element('sdf', {'version': '1.10'})
    world = ET.SubElement(sdf, 'world', {'name': 'factorymate_dynamic_warehouse'})

    _txt(world, 'gravity', '0 0 -9.81')
    physics = ET.SubElement(world, 'physics', {'name': 'default_physics', 'type': 'ignored'})
    _txt(physics, 'max_step_size', '0.004')
    _txt(physics, 'real_time_factor', '1.0')

    for filename, name in [
        ('gz-sim-physics-system', 'gz::sim::systems::Physics'),
        ('gz-sim-user-commands-system', 'gz::sim::systems::UserCommands'),
        ('gz-sim-scene-broadcaster-system', 'gz::sim::systems::SceneBroadcaster'),
    ]:
        ET.SubElement(world, 'plugin', {'filename': filename, 'name': name})
    sensors = ET.SubElement(world, 'plugin', {'filename': 'gz-sim-sensors-system', 'name': 'gz::sim::systems::Sensors'})
    _txt(sensors, 'render_engine', 'ogre2')

    scene = ET.SubElement(world, 'scene')
    _txt(scene, 'ambient', '0.55 0.58 0.62 1')
    _txt(scene, 'background', '0.12 0.14 0.18 1')
    _txt(scene, 'shadows', 'true')
    _txt(scene, 'grid', 'false')

    floor = _static_box_model(world, 'warehouse_floor', (50.0, 34.0, 0.12), (0, 0, -0.06, 0, 0, 0), (0.36, 0.38, 0.40, 1))
    # Add painted lanes as visuals only.
    for idx, y in enumerate((-12.5, -8.5, -4.5, -0.5, 3.5, 7.5, 11.5)):
        _box_link(floor, f'lane_{idx}', (46.0, 0.08, 0.01), (0, y, 0.071, 0, 0, 0), (0.95, 0.72, 0.10, 1), collision=False)

    wall_color = (0.52, 0.56, 0.60, 1)
    _static_box_model(world, 'north_wall', (50, 0.18, 4.5), (0, 17, 2.25, 0, 0, 0), wall_color)
    _static_box_model(world, 'south_wall', (50, 0.18, 4.5), (0, -17, 2.25, 0, 0, 0), wall_color)
    _static_box_model(world, 'east_wall', (0.18, 34, 4.5), (25, 0, 2.25, 0, 0, 0), wall_color)
    _static_box_model(world, 'west_wall', (0.18, 34, 4.5), (-25, 0, 2.25, 0, 0, 0), wall_color)

    # 6 x 6 grid of racks with wide cross-aisles.
    rack_xs = (-18.0, -10.8, -3.6, 3.6, 10.8, 18.0)
    rack_ys = (-10.8, -6.4, -2.0, 2.4, 6.8, 11.2)
    count = 1
    for y in rack_ys:
        for x in rack_xs:
            _add_rack(world, f'shelf_r{count:02d}', x, y)
            count += 1

    _static_box_model(world, 'pickup_pallet', (2.4, 2.4, 0.12), (-21.2, -14.2, 0.06, 0, 0, 0), (0.10, 0.55, 0.20, 1))
    _static_box_model(world, 'inspection_cell', (2.4, 2.4, 0.12), (0.0, -14.2, 0.06, 0, 0, 0), (0.15, 0.35, 0.75, 1))
    _static_box_model(world, 'delivery_cell', (2.4, 2.4, 0.12), (21.2, -14.2, 0.06, 0, 0, 0), (0.75, 0.18, 0.12, 1))
    _static_box_model(world, 'pallet_jack', (1.7, 0.65, 0.35), (-14.5, 14.2, 0.175, 0, 0, 0.35), (0.92, 0.65, 0.06, 1))
    _static_box_model(world, 'work_desk', (2.2, 0.8, 0.9), (14.5, 14.2, 0.45, 0, 0, 0), (0.28, 0.22, 0.18, 1))

    for i, (lx, ly) in enumerate(((-18, -12), (-6, -12), (6, -12), (18, -12), (-12, 12), (0, 12), (12, 12), (-20, 0), (20, 0)), start=1):
        light = ET.SubElement(world, 'light', {'name': f'ceiling_light_{i:02d}', 'type': 'point'})
        _pose(light, (lx, ly, 4.0, 0, 0, 0))
        _txt(light, 'diffuse', '0.95 0.95 0.90 1')
        _txt(light, 'specular', '0.15 0.15 0.15 1')
        _txt(light, 'cast_shadows', 'true')
        _txt(light, 'intensity', '2.3')
        attenuation = ET.SubElement(light, 'attenuation')
        _txt(attenuation, 'range', '16')
        _txt(attenuation, 'constant', '0.4')
        _txt(attenuation, 'linear', '0.02')
        _txt(attenuation, 'quadratic', '0.002')

    worker_starts = [
        (-22, -11, 0.0), (-15, -4, 1.57), (-7, 9, 3.14), (0, -9, 1.57),
        (8, 5, -1.57), (16, -8, 3.14), (21, 10, 3.14), (-20, 7, 0.0), (2, 13, -1.57),
    ]
    for i in range(worker_count):
        x, y, yaw = worker_starts[i % len(worker_starts)]
        _worker_model(world, i + 1, x, y, yaw)

    ET.indent(sdf, space='  ')
    xml_body = ET.tostring(sdf, encoding='unicode')
    return '<?xml version="1.0" ?>\n' + xml_body + '\n'


if __name__ == '__main__':
    print(build_world(), end='')
