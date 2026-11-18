"""The printed parts, each turned the way it goes on the H2S build plate (Z up, lowest face on the plate).

PETG on a Bambu Lab H2S (340 x 320 x 340 mm), 0.4 mm nozzle, 0.2 mm layers, no supports.

Check: cad model dfm "$ID" --process printing --technology fdm --material petg --build-volume 340x320x340
"""
import build123d as bd
from bd_materials import plastics
from cad import model
from bd_materials.processes import fdm

from rov import hull_cap, hull_tub, nacelle


def on_plate(shape, label):
    bb = shape.bounding_box()
    s = bd.Pos(-bb.center().X, -bb.center().Y, -bb.min.Z) * shape
    s.label = label
    s.material = plastics.petg(process=fdm(layer_height_mm=0.2))
    return s


@model
def print_hull_tub():
    """Floor down, open top up."""
    return on_plate(hull_tub().shape, "hull_tub")


@model
def print_hull_cap():
    """Roof down: the top chamfer becomes a 45 degree slope, no supports."""
    return on_plate(bd.Rot(180, 0, 0) * hull_cap().shape, "hull_cap")


@model
def print_nacelle():
    """Standing on its end: a tube printed upright needs no supports. Print two."""
    return on_plate(bd.Rot(0, -90, 0) * nacelle().shape, "nacelle")


if __name__ == "__main__":
    print_hull_tub()
    print_hull_cap()
    print_nacelle()
