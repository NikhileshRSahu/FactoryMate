from pathlib import Path
import importlib.util
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'tools/import_dynamic_logistics_warehouse.py'


def _load():
    spec = importlib.util.spec_from_file_location('asset_importer', SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _fixture(tmp_path: Path) -> Path:
    src = tmp_path / 'dynamic_logistics_warehouse'
    model = src / 'models/test_rack'
    (model / 'meshes').mkdir(parents=True)
    (model / 'materials/textures').mkdir(parents=True)
    (src / 'worlds').mkdir(parents=True)
    (model / 'model.config').write_text('<model><name>test_rack</name><sdf version="1.6">model.sdf</sdf></model>')
    (model / 'model.sdf').write_text('<sdf version="1.6"><model name="test_rack"><static>true</static></model></sdf>')
    (model / 'meshes/rack.DAE').write_bytes(b'<COLLADA>mesh</COLLADA>')
    (model / 'materials/textures/rack.png').write_bytes(b'\x89PNG\r\nfixture')
    (src / 'worlds/warehouse.world').write_text('''<?xml version="1.0"?>
<sdf version="1.6"><world name="old_warehouse">
  <plugin filename="libgazebo_ros_factory.so" name="gazebo_ros_factory"/>
  <model name="static_box"><static>true</static></model>
</world></sdf>''')
    (src / 'LICENSE').write_text('GPL-2.0 fixture')
    return src


def test_import_copies_binary_assets_and_creates_harmonic_world(tmp_path):
    mod = _load()
    src = _fixture(tmp_path)
    dest = tmp_path / 'factorymate_gazebo'
    (dest / 'models').mkdir(parents=True)
    (dest / 'worlds').mkdir(parents=True)
    report = mod.import_assets(src, dest)
    assert report.models_copied == 1
    assert (dest / 'models/upstream/test_rack/meshes/rack.DAE').read_bytes() == b'<COLLADA>mesh</COLLADA>'
    assert (dest / 'models/upstream/test_rack/materials/textures/rack.png').read_bytes() == b'\x89PNG\r\nfixture'
    converted = dest / 'worlds/dynamic_logistics_warehouse_harmonic.sdf'
    root = ET.parse(converted).getroot()
    assert root.attrib['version'] == '1.10'
    world = root.find('world')
    assert world.attrib['name'] == 'factorymate_dynamic_logistics_warehouse'
    filenames = [p.attrib.get('filename', '') for p in world.findall('.//plugin')]
    assert not any('libgazebo' in f for f in filenames)
    assert 'gz-sim-physics-system' in filenames
    assert 'gz-sim-user-commands-system' in filenames
    assert 'gz-sim-scene-broadcaster-system' in filenames


def test_import_writes_attribution_and_source_snapshot(tmp_path):
    mod = _load()
    src = _fixture(tmp_path)
    dest = tmp_path / 'factorymate_gazebo'
    (dest / 'models').mkdir(parents=True)
    (dest / 'worlds').mkdir(parents=True)
    mod.import_assets(src, dest)
    attribution = (dest / 'models/upstream/UPSTREAM_SOURCE.md').read_text()
    assert 'belal-ibrahim/dynamic_logistics_warehouse' in attribution
    assert 'GPL-2.0' in attribution
    assert (dest / 'worlds/dynamic_logistics_warehouse_classic.source.world').exists()


def test_import_rejects_incomplete_source(tmp_path):
    mod = _load()
    bad = tmp_path / 'bad'
    bad.mkdir()
    dest = tmp_path / 'dest'
    try:
        mod.import_assets(bad, dest)
    except ValueError as exc:
        assert 'models' in str(exc) or 'warehouse.world' in str(exc)
    else:
        raise AssertionError('incomplete source must fail')
