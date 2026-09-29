# Third-party sources

## belal-ibrahim/dynamic_logistics_warehouse

Source repository: `https://github.com/belal-ibrahim/dynamic_logistics_warehouse`

The project uses this repository as an **optional asset source**. Its meshes, textures, maps, and original world are **not bundled** in this distribution; `scripts/fetch_dynamic_logistics_assets.sh` downloads them directly from the upstream repository on the user's machine.

Licensing deserves explicit review: the upstream GitHub repository advertises a **GPL-2.0** root license, while its historical `package.xml` contains an **Apache-2.0** declaration. This project does not attempt to resolve that inconsistency. The importer preserves the upstream `LICENSE` alongside imported assets, and users should satisfy the upstream license terms before redistribution or commercial use.

The simulation world used here is `dynamic_logistics_warehouse_harmonic.sdf`, a Gazebo Harmonic port of the upstream warehouse. Generic primitive FactoryMate warehouse SDFs are not included.
