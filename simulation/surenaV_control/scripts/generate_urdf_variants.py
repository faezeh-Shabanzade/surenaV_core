#!/usr/bin/env python3
"""Generate position- and velocity-control URDF variants of the Surena V model.

The mechanical source of truth is the merged package
``surenaV_description/urdf/surenaV.urdf`` (full body + hands).  This script
NEVER edits that file.  It
produces two variants in ``surenaV_control/urdf/`` that differ from the source
in exactly one thing: the ``<hardwareInterface>`` text inside each of the 51
transmissions (both the <joint> and the <actuator> element):

  surenaV_position.urdf   hardware_interface/PositionJointInterface
  surenaV_velocity.urdf   hardware_interface/VelocityJointInterface

Why two files: Gazebo Classic's ``DefaultRobotHWSim`` reads only the FIRST
<hardwareInterface> tag of a transmission, so a single URDF cannot expose the
position and velocity command interfaces at the same time.  Two variants keep
the mechanical model single-source while giving each control mode the
interface it needs.

Usage:
    python3 generate_urdf_variants.py [--source PATH] [--outdir PATH]

Run automatically by CMakeLists.txt at build time; can also be run by hand.
"""

import argparse
import os
import re
import sys
import xml.etree.ElementTree as ET

DEFAULT_SOURCE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "..", "surenaV_description", "urdf", "surenaV.urdf",
)
DEFAULT_OUTDIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "urdf",
)

POSITION_IFACE = "hardware_interface/PositionJointInterface"
VELOCITY_IFACE = "hardware_interface/VelocityJointInterface"

# Matches the hardwareInterface element text wherever it appears inside a
# transmission block (joint or actuator sub-element).
HW_IFACE_RE = re.compile(
    r"(<hardwareInterface>)\s*[^<]*?(</hardwareInterface>)"
)


def generate_variant(source_path, out_path, interface):
    """Copy source_path to out_path replacing only hardwareInterface texts."""
    with open(source_path, "r", encoding="utf-8") as f:
        text = f.read()

    n_transmissions = text.count("<transmission ")
    n_replaced = 0

    def _sub(m):
        nonlocal n_replaced
        n_replaced += 1
        return m.group(1) + interface + m.group(2)

    new_text = HW_IFACE_RE.sub(_sub, text)

    # Sanity: every transmission must expose exactly two hardwareInterface
    # tags (joint + actuator) in the source model.
    expected = n_transmissions * 2
    if n_replaced != expected:
        raise RuntimeError(
            "Expected %d <hardwareInterface> tags (2 per transmission), "
            "found/replaced %d in %s" % (expected, n_replaced, source_path)
        )

    # Structural sanity: still valid XML with the same joints/links.
    src_root = ET.parse(source_path).getroot()
    new_root = ET.parsestring_safe = ET.fromstring(new_text)
    for tag in ("link", "joint", "transmission"):
        src_names = [e.get("name") for e in src_root.findall(tag)]
        new_names = [e.get("name") for e in new_root.findall(tag)]
        if src_names != new_names:
            raise RuntimeError(
                "%s list changed while generating %s" % (tag, out_path)
            )

    with open(out_path, "w", encoding="utf-8") as f:
        f.write(new_text)
    return n_replaced


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--source", default=DEFAULT_SOURCE,
                        help="Path to the mechanical source URDF")
    parser.add_argument("--outdir", default=DEFAULT_OUTDIR,
                        help="Directory for the generated variants")
    args = parser.parse_args()

    source = os.path.abspath(args.source)
    outdir = os.path.abspath(args.outdir)
    if not os.path.isfile(source):
        sys.exit("Source URDF not found: %s" % source)
    os.makedirs(outdir, exist_ok=True)

    for filename, interface in (
        ("surenaV_position.urdf", POSITION_IFACE),
        ("surenaV_velocity.urdf", VELOCITY_IFACE),
    ):
        out_path = os.path.join(outdir, filename)
        n = generate_variant(source, out_path, interface)
        print("Generated %s (%d hardwareInterface tags -> %s)"
              % (out_path, n, interface))


if __name__ == "__main__":
    main()
