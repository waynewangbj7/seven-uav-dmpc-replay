"""Resolve local scene assets and adapt the verified Open Robotics X3 visual links."""
import argparse
from hashlib import sha256
import os
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parent
ASSETS = ROOT / "assets"
X3_SHA256 = "008210ed18d40e59568587d780825855d728ddc0cdcbfa82d6dc336b71c93705"
X3_URL = "https://fuel.gazebosim.org/1.0/OpenRobotics/models/X3 UAV/4"


def upstream_model():
    override = os.environ.get("X3_MODEL_SDF")
    if override:
        candidates = [Path(override).expanduser()]
    else:
        cache = Path.home() / ".gz/fuel"
        candidates = sorted(cache.glob("*/openrobotics/models/x3 uav/4/model.sdf"))
    for model in candidates:
        if model.is_file() and sha256(model.read_bytes()).hexdigest() == X3_SHA256:
            return model
    raise SystemExit(
        f"Expected Open Robotics X3 UAV revision 4 in the Gazebo Fuel cache. "
        f"Run gz fuel download -u '{X3_URL}' or set X3_MODEL_SDF to its model.sdf."
    )


def write_model(stage):
    directory = stage / "models/mavic_x3"
    directory.mkdir(parents=True)
    xml = ET.parse(upstream_model())
    links = xml.findall("model/link")
    assert len(links) == 5
    for link in links:
        visual = link.find("visual")
        mesh = visual.find("geometry/mesh")
        if link.get("name") == "X3/base_link":
            filename = "mavic_style.dae"
            units = "0.0041935484 0.0062686567 0.0042592593"
        else:
            assert link.get("name") in {f"X3/rotor_{i}" for i in range(4)}
            filename = "mavic_rotor.dae"
            units = "0.0058982947 0.0058982947 0.0058982947"
        mesh.find("uri").text = (ASSETS / filename).as_uri()
        scale = mesh.find("scale")
        if scale is None:
            scale = ET.SubElement(mesh, "scale")
        scale.text = units
        material = visual.find("material")
        if material is not None:
            visual.remove(material)
    xml.write(directory / "model.sdf", encoding="utf-8", xml_declaration=True)
    (directory / "model.config").write_text(
        '<model><name>Mavic-inspired X3</name><version>1.0</version>'
        '<sdf version="1.6">model.sdf</sdf></model>\n',
        encoding="utf-8",
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scenario", choices=("baseline", "a", "b", "c", "dual"))
    parser.add_argument("stage", type=Path)
    args = parser.parse_args()
    source = ROOT / "scenes" / f"{args.scenario}.sdf"
    xml = ET.parse(source)
    for uri in xml.iter("uri"):
        if uri.text.startswith("../assets/"):
            asset = (source.parent / uri.text).resolve(strict=True)
            assert asset.is_relative_to(ASSETS)
            uri.text = asset.as_uri()
    write_model(args.stage)
    xml.write(args.stage / "world.sdf", encoding="utf-8", xml_declaration=True)


if __name__ == "__main__":
    main()
