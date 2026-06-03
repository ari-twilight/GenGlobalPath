from multiprocessing import freeze_support
import os
import time

from GenGlobalPath import *
from SliceAlgo import *
from Utility import *


bead_width = None
overlap_rate = 0.5
slc_path = os.path.join(os.path.dirname(__file__), "hechai_3 at 2mm.slc")


def _worker_count(layer_count):
    env_value = os.environ.get("MAX_WORKERS")
    if env_value:
        return max(1, int(env_value))
    cpu_count = os.cpu_count() or 1
    return max(1, min(cpu_count - 1, 6, layer_count))


def _plan_layers(layers):
    use_parallel = os.environ.get("PARALLEL", "1") != "0" and len(layers) > 4
    workers = _worker_count(len(layers)) if use_parallel else 1
    print(
        f"Planning {len(layers)} layers "
        f"({'parallel' if use_parallel else 'serial'}, workers={workers})..."
    )
    return continuousLayerPathPlanner(
        layers,
        bead_width,
        overlap_rate,
        max_workers=workers,
        parallel=use_parallel,
    )


def _draw_layer(layers, global_paths, target_layer_idx):
    va = VtkAdaptor()
    for contour in layers[target_layer_idx].contours:
        if contour is not None and contour.count() > 0:
            actor = va.drawPolyline(contour)
            actor.GetProperty().SetColor(0, 0, 0)
            actor.GetProperty().SetLineWidth(1.0)

    for path in global_paths[target_layer_idx]:
        if path is not None and path.count() > 0:
            actor = va.drawPolyline(path)
            actor.GetProperty().SetColor(1, 0, 0)
            actor.GetProperty().SetLineWidth(1.5)
    va.display()


def main():
    layers = readSlcFile(slc_path)
    print(f"SLC: {slc_path}")
    print(f"Layer count: {len(layers)}")

    non_empty = next((layer for layer in layers if layer.contours), None)
    if non_empty is not None:
        actual_bead_width, actual_overlap, shell_thickness = estimateWaamParameters(
            non_empty.contours, overlap_rate
        )
        print(
            f"WAAM params: bead_width={actual_bead_width:.3f}mm, "
            f"spacing={actual_bead_width * (1 - actual_overlap):.3f}mm, "
            f"shell={shell_thickness:.3f}mm"
        )

    start = time.perf_counter()
    global_paths = _plan_layers(layers)
    elapsed = time.perf_counter() - start
    total_points = sum(path.count() for paths in global_paths for path in paths)
    print(f"GenGlobalPath finished: {elapsed:.3f}s, points={total_points}")

    if os.environ.get("NO_VTK", "0") == "1":
        return

    target_layer_idx = int(os.environ.get("TARGET_LAYER", min(40, len(layers) - 1)))
    target_layer_idx = max(0, min(target_layer_idx, len(layers) - 1))
    print(f"Rendering layer {target_layer_idx}...")
    _draw_layer(layers, global_paths, target_layer_idx)


if __name__ == "__main__":
    freeze_support()
    main()
