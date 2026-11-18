"""Flatten Bambu Studio's built-in BBL presets (resolve 'inherits') so the command line can use them."""
import json, sys
from pathlib import Path
R = Path("/Applications/BambuStudio.app/Contents/Resources/profiles/BBL")
def load(kind, name):
    p = R / kind / f"{name}.json"
    d = json.loads(p.read_text())
    if "inherits" in d:
        base = load(kind, d["inherits"]); base.update(d); d = base
    return d
def out(kind, name, path, **over):
    d = load(kind, name); d.pop("inherits", None); d["from"] = "User"; d.update(over)
    Path(path).write_text(json.dumps(d, indent=1))
out("machine", "Bambu Lab H2S 0.4 nozzle", "machine.json")
common = dict(sparse_infill_density="100%", sparse_infill_pattern="zig-zag", wall_loops="2", enable_support="0",
              top_shell_layers="4", bottom_shell_layers="4", brim_type="auto_brim", seam_position="back")
out("process", "0.20mm Standard @BBL H2S", "process.json", **common)
out("filament", "Bambu PETG HF @BBL H2S", "petg.json")
out("filament", "Bambu PLA Basic @BBL H2S", "pla.json")
print("ok")
