#!/usr/bin/env python3
from __future__ import annotations

import argparse
import shutil
from pathlib import Path
from typing import NamedTuple
from xml.etree import ElementTree as ET

UPSTREAM_URL = 'https://github.com/belal-ibrahim/dynamic_logistics_warehouse'


class ImportReport(NamedTuple):
    models_copied: int
    converted_world: Path
    source_snapshot: Path


def _is_classic_plugin(plugin: ET.Element) -> bool:
    filename = plugin.attrib.get('filename', '').lower()
    name = plugin.attrib.get('name', '').lower()
    classic_tokens = ('libgazebo', 'gazebo_ros')
    return any(token in filename or token in name for token in classic_tokens)


def _strip_classic_plugins(root: ET.Element) -> int:
    removed = 0
    for parent in root.iter():
        for child in list(parent):
            if child.tag == 'plugin' and _is_classic_plugin(child):
                parent.remove(child)
                removed += 1
    return removed


def _ensure_harmonic_systems(world: ET.Element) -> None:
    existing = {p.attrib.get('filename') for p in world.findall('plugin')}
    systems = [
        ('gz-sim-physics-system', 'gz::sim::systems::Physics'),
        ('gz-sim-user-commands-system', 'gz::sim::systems::UserCommands'),
        ('gz-sim-scene-broadcaster-system', 'gz::sim::systems::SceneBroadcaster'),
    ]
    for filename, name in systems:
        if filename not in existing:
            ET.SubElement(world, 'plugin', {'filename': filename, 'name': name})
    if 'gz-sim-sensors-system' not in existing:
        sensors = ET.SubElement(world, 'plugin', {'filename': 'gz-sim-sensors-system', 'name': 'gz::sim::systems::Sensors'})
        engine = ET.SubElement(sensors, 'render_engine')
        engine.text = 'ogre2'


def convert_world(source_world: Path, output_world: Path) -> int:
    tree = ET.parse(source_world)
    root = tree.getroot()
    if root.tag != 'sdf':
        raise ValueError(f'expected an SDF root in {source_world}')
    root.set('version', '1.10')
    world = root.find('world')
    if world is None:
        raise ValueError(f'no <world> found in {source_world}')
    world.set('name', 'factorymate_dynamic_logistics_warehouse')
    removed = _strip_classic_plugins(root)
    _ensure_harmonic_systems(world)
    output_world.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root, space='  ')
    tree.write(output_world, encoding='utf-8', xml_declaration=True)
    return removed


def import_assets(source: Path, destination: Path) -> ImportReport:
    source = Path(source).expanduser().resolve()
    destination = Path(destination).expanduser().resolve()
    models = source / 'models'
    source_world = source / 'worlds' / 'warehouse.world'
    if not models.is_dir():
        raise ValueError(f'upstream source is missing models/: {source}')
    if not source_world.is_file():
        raise ValueError(f'upstream source is missing worlds/warehouse.world: {source}')

    upstream_dest = destination / 'models' / 'upstream'
    upstream_dest.mkdir(parents=True, exist_ok=True)
    for child in models.iterdir():
        target = upstream_dest / child.name
        if child.is_dir():
            shutil.copytree(child, target, dirs_exist_ok=True)
        elif child.is_file():
            shutil.copy2(child, target)

    model_count = sum(1 for p in upstream_dest.iterdir() if p.is_dir() and (p / 'model.sdf').exists())
    if model_count == 0:
        raise ValueError('no Gazebo model.sdf files were found in the upstream models directory')

    classic_snapshot = destination / 'worlds' / 'dynamic_logistics_warehouse_classic.source.world'
    classic_snapshot.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source_world, classic_snapshot)
    harmonic_world = destination / 'worlds' / 'dynamic_logistics_warehouse_harmonic.sdf'
    convert_world(source_world, harmonic_world)

    license_src = source / 'LICENSE'
    if license_src.is_file():
        shutil.copy2(license_src, upstream_dest / 'LICENSE.upstream')

    (upstream_dest / 'UPSTREAM_SOURCE.md').write_text(
        '# Upstream warehouse assets\n\n'
        f'Source: {UPSTREAM_URL}\n\n'
        'The upstream repository root advertises GPL-2.0 licensing. Its package metadata also contains '
        'an Apache-2.0 declaration, so keep the original LICENSE file with redistributed assets and '
        'review licensing requirements before commercial redistribution.\n\n'
        'The original Gazebo Classic world is preserved under `worlds/` as a source snapshot. '
        'The Harmonic SDF copy removes Classic-only `libgazebo*` / `gazebo_ros*` plugins and adds '
        'Gazebo Harmonic world systems. Model meshes and textures are copied without modification.\n'
    )
    return ImportReport(model_count, harmonic_world, classic_snapshot)


def main() -> None:
    parser = argparse.ArgumentParser(description='Import and port dynamic_logistics_warehouse assets for FactoryMate.')
    parser.add_argument('source', type=Path, help='Local clone/path of belal-ibrahim/dynamic_logistics_warehouse')
    parser.add_argument('destination', type=Path, help='factorymate_gazebo package directory')
    args = parser.parse_args()
    report = import_assets(args.source, args.destination)
    print(f'Imported {report.models_copied} models')
    print(f'Harmonic world: {report.converted_world}')
    print(f'Classic source snapshot: {report.source_snapshot}')


if __name__ == '__main__':
    main()
