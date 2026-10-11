FROM nvcr.io/nvidia/pytorch:24.05-py3@sha256:d4030fc006cce2e56497fa07d003c5996de0eb7bde3610599fedbe7e7d03adea
# Build context is the external tools/graspnet directory, never the entire rak repo.
ENV TORCH_CUDA_ARCH_LIST="8.9" MAX_JOBS=2
RUN apt-get -o Acquire::ForceIPv4=true -o Acquire::Retries=3 update && apt-get -o Acquire::ForceIPv4=true -o Acquire::Retries=3 install -y --no-install-recommends libgl1 libglib2.0-0 && rm -rf /var/lib/apt/lists/*
# These dependencies come from PyPI; do not query the base image's unavailable
# NVIDIA extra index for every package. Keep the bundled NVIDIA PyTorch intact.
RUN PIP_CONFIG_FILE=/dev/null python -m pip install --index-url https://pypi.org/simple --no-cache-dir 'numpy==1.26.4' 'scipy==1.13.1' 'open3d==0.18.0' 'opencv-python-headless==4.10.0.84' 'transforms3d==0.4.2' 'trimesh==4.4.9' 'Pillow==10.4.0' tqdm scikit-image
COPY graspnet-baseline /opt/graspnet-baseline
COPY graspnetAPI /opt/graspnetAPI
COPY compat.py /opt/rak-grasp-compat.py
RUN test "$(git -C /opt/graspnet-baseline rev-parse HEAD)" = "280c215129f759ed8649cb4e89fc5dfee55f4f80" && test "$(git -C /opt/graspnetAPI rev-parse HEAD)" = "eb57dd2092d8dbe05312a29c3d0c22f3226efbfc"
RUN python /opt/rak-grasp-compat.py /opt/graspnet-baseline /opt/graspnetAPI && PIP_CONFIG_FILE=/dev/null python -m pip install --no-cache-dir --no-build-isolation --no-deps /opt/graspnet-baseline/pointnet2 /opt/graspnet-baseline/knn /opt/graspnetAPI
WORKDIR /work
# NVIDIA's source-built OpenCV leaves an untracked config-3.10.py/native module.
# Remove its shared package directory after uninstalling both distributions,
# then install one complete headless binding. This affects the image copy only.
RUN PIP_CONFIG_FILE=/dev/null python -m pip uninstall -y opencv opencv-python-headless && python -c 'from pathlib import Path; import shutil; p=Path("/usr/local/lib/python3.10/dist-packages/cv2"); assert p.is_absolute() and p.name == "cv2"; shutil.rmtree(p) if p.exists() else None' && PIP_CONFIG_FILE=/dev/null python -m pip install --index-url https://pypi.org/simple --no-cache-dir --no-deps opencv-python-headless==4.10.0.84 && python -c 'import cv2; assert cv2.__version__ == "4.10.0"; print(cv2.__version__)'
