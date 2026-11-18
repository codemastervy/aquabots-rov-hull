"""The printed part on the H2S build plate: the whole frame in one piece, as modelled (no supports).

Check: cad model dfm "$ID" --process printing --technology fdm --material petg --build-volume 340x320x340
"""
import build123d as bd
from bd_materials import plastics
from bd_materials.processes import fdm
from cad import model

from rov import frame


@model
def print_frame():
    f = frame().shape
    bb = f.bounding_box()
    s = bd.Pos(-bb.center().X, -bb.center().Y, -bb.min.Z) * f
    s.label = "frame"
    s.material = plastics.petg(process=fdm(layer_height_mm=0.2))
    return s


if __name__ == "__main__":
    print_frame()
