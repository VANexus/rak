"""Patch only a copied, pinned upstream checkout inside the developer image."""
from pathlib import Path
import sys

root = Path(sys.argv[1])
for folder in ("pointnet2", "knn"):
    for file in (root / folder).rglob("*"):
        if file.suffix in (".cpp", ".h", ".cu"):
            text = file.read_text()
            text = text.replace(".data<", ".data_ptr<").replace(".type().is_cuda()", ".is_cuda()")
            file.write_text(text)
# Docker image builds have no visible GPU; compile CUDA KNN based on toolkit availability.
setup = root / "knn/setup.py"
setup.write_text(setup.read_text().replace(
    "torch.cuda.is_available() and CUDA_HOME is not None", "CUDA_HOME is not None"))
# Offline inference only needs grasp containers. Do not import benchmark/DexNet
# evaluation or install their large dependency tree into the developer image.
api = Path(sys.argv[2])
(api / "graspnetAPI/__init__.py").write_text(
    'from .grasp import Grasp, GraspGroup, RectGrasp, RectGraspGroup\n')
