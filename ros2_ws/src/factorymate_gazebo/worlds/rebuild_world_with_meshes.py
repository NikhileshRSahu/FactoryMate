#!/usr/bin/env python3
"""
rebuild_world_with_meshes.py
============================
Repairs dynamic_logistics_warehouse_harmonic.sdf.

The original migration script inlined every model as a 1×1×1 box placeholder
instead of using <include> to reference the real model SDFs with actual mesh
geometry and textures.

This script:
1. Parses the broken world SDF.
2. For each box-placeholder model, extracts:
     - model type  (e.g. aws_robomaker_warehouse_ShelfE_01)
     - instance name (e.g. aws_robomaker_warehouse_ShelfE_01_001)
     - world pose
3. For each model whose type has a model.sdf in our models/upstream/ directory,
   replaces the inline box definition with a proper <include> block.
4. Leaves actors, DeskC inline compound, waybot_ground, and any unknown models
   untouched (they are either fine as-is or not in our model library).
5. Writes the result to dynamic_logistics_warehouse_harmonic_fixed.sdf.
"""

import re
import sys
import os
import xml.etree.ElementTree as ET
from pathlib import Path

# ── Paths ──────────────────────────────────────────────────────────────────
SCRIPT_DIR   = Path(__file__).parent
WORLD_IN     = SCRIPT_DIR / "dynamic_logistics_warehouse_harmonic.sdf"
WORLD_OUT    = SCRIPT_DIR / "dynamic_logistics_warehouse_harmonic_fixed.sdf"
MODELS_DIR   = SCRIPT_DIR.parent / "models" / "upstream"

# ── Which model types we manage (must have model.sdf in MODELS_DIR) ────────
MANAGED_TYPES = {d.name for d in MODELS_DIR.iterdir() if d.is_dir()}
print(f"[INFO] Managed model types ({len(MANAGED_TYPES)}):", sorted(MANAGED_TYPES))

# ── Helpers ────────────────────────────────────────────────────────────────

def model_type_from_name(instance_name):
    """
    Given an instance name like 'aws_robomaker_warehouse_ShelfE_01_003',
    return the model type 'aws_robomaker_warehouse_ShelfE_01' by stripping the
    trailing _NNN suffix.  Returns None if not recognized.
    """
    # Strip trailing _<digits> suffix (instance number)
    m = re.match(r'^(.+?)(?:_\d+)?$', instance_name)
    candidate = m.group(1) if m else instance_name
    if candidate in MANAGED_TYPES:
        return candidate
    # Some names have no suffix at all and ARE the type
    if instance_name in MANAGED_TYPES:
        return instance_name
    return None


def include_block(model_type, instance_name, pose):
    """
    Emit an SDF <include> block for a managed model.
    """
    return (
        f'    <include>\n'
        f'      <uri>model://{model_type}</uri>\n'
        f'      <name>{instance_name}</name>\n'
        f'      <pose>{pose}</pose>\n'
        f'    </include>'
    )


# ── Parse input ────────────────────────────────────────────────────────────
raw = WORLD_IN.read_text(encoding='utf-8')

lines = raw.splitlines()

# Parse with ElementTree just to extract model names and poses.
try:
    tree = ET.parse(str(WORLD_IN))
except ET.ParseError as e:
    print(f"[ERROR] XML parse error: {e}")
    sys.exit(1)

root = tree.getroot()
world_el = root.find('world') if root.tag == 'sdf' else root

# Build a mapping: instance_name -> world_pose_string
# Only for top-level <model> children (not nested models inside actors/Untitled)
model_poses = {}

def el_pose_str(model_el):
    """Get the outermost pose of a model element."""
    pose_el = model_el.find('pose')
    if pose_el is not None and pose_el.text:
        return pose_el.text.strip()
    return '0 0 0 0 0 0'

for child in world_el:
    if child.tag == 'model':
        mname = child.get('name', '')
        model_poses[mname] = el_pose_str(child)

print(f"[INFO] Found {len(model_poses)} top-level models in world SDF")

# ── Second pass: rebuild the file ──────────────────────────────────────────
# We find each top-level <model name="..."> block in the raw text and decide
# whether to keep it or replace it with an <include>.

# Top-level model blocks are indented with 4 spaces (direct child of <world>)
MODEL_START  = re.compile(r'^    <model name="([^"]+)">')

output_lines = []
replaced     = 0
kept_models  = 0

i = 0
while i < len(lines):
    line = lines[i]

    m = MODEL_START.match(line)
    if m:
        instance_name = m.group(1)
        mtype = model_type_from_name(instance_name)
        pose  = model_poses.get(instance_name, '0 0 0 0 0 0')

        if mtype and mtype in MANAGED_TYPES:
            # Replace entire block with <include>
            output_lines.append(include_block(mtype, instance_name, pose))
            # Skip until the matching closing </model> at depth 0
            depth = 1
            i += 1
            while i < len(lines) and depth > 0:
                l = lines[i]
                # Count nested model open/close tags
                opens  = l.count('<model ') + l.count('<model>')
                closes = l.count('</model>')
                depth += opens - closes
                i += 1
            replaced += 1
            continue   # skip i += 1 at bottom
        else:
            # Keep as-is (actor, Untitled compound, waybot, unknown)
            output_lines.append(line)
            kept_models += 1
    else:
        output_lines.append(line)

    i += 1

print(f"[INFO] Replaced {replaced} box-placeholder models with <include> blocks")
print(f"[INFO] Kept {kept_models} model-line occurrences unchanged")

# ── Write output ───────────────────────────────────────────────────────────
WORLD_OUT.write_text('\n'.join(output_lines) + '\n', encoding='utf-8')
print(f"[OK]  Written to: {WORLD_OUT}")

# ── Sanity check ───────────────────────────────────────────────────────────
out_text = WORLD_OUT.read_text()
box_count     = out_text.count('<box>')
include_count = out_text.count('<include>')
print(f"\n[SANITY] <box>     tags in output : {box_count}")
print(f"[SANITY] <include> tags in output : {include_count}")
if box_count == 0:
    print("[OK]  No stray box placeholders remain.")
else:
    print(f"[WARN] {box_count} <box> tags remain (may be in actors/ground — inspect if unexpected).")
