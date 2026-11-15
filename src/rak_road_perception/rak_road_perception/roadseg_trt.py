"""TensorRT segmentation inference for the road-perception node.

Why not torch: JetPack does not ship PyTorch and apt has no aarch64 candidate, so a
TorchScript runtime is not a deployable dependency (see `docs/design/vision_nav_v1.md`
§2.3). TensorRT *is* shipped with JetPack, and the engine is built once by
`scripts/build_roadseg_engine.py`.

Why not onnxruntime: the aarch64 `onnxruntime-gpu` wheel ships no TensorRT EP, and its
CUDA EP measured 54.9 ms against 16.9 ms for the native engine on the same model.

Device memory is managed through `libcudart` via ctypes: neither `pycuda` nor
`cuda-python` is installed on the robot, and a 40-line ctypes shim is smaller than a
new dependency that must be rebuilt per JetPack.
"""
from __future__ import annotations

import ctypes
import ctypes.util
import glob
import os
import time

import numpy as np


class CudartError(RuntimeError):
    pass


class _Cudart:
    """Minimal libcudart surface: device buffers, copies, one stream."""

    _LIBCANDIDATES = ("libcudart.so", "libcudart.so.13", "libcudart.so.12")

    def __init__(self) -> None:
        self.lib = self._load()
        self.lib.cudaGetErrorString.restype = ctypes.c_char_p
        self.lib.cudaGetErrorString.argtypes = [ctypes.c_int]
        self.lib.cudaMalloc.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_size_t]
        self.lib.cudaFree.argtypes = [ctypes.c_void_p]
        self.lib.cudaMemcpy.argtypes = [
            ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t, ctypes.c_int,
        ]
        self.lib.cudaStreamCreate.argtypes = [ctypes.POINTER(ctypes.c_void_p)]
        self.lib.cudaStreamSynchronize.argtypes = [ctypes.c_void_p]
        self.lib.cudaStreamDestroy.argtypes = [ctypes.c_void_p]
        self.lib.cudaSetDevice.argtypes = [ctypes.c_int]
        self._check(self.lib.cudaSetDevice(0))

    @classmethod
    def _load(cls) -> ctypes.CDLL:
        for name in cls._LIBCANDIDATES:
            try:
                return ctypes.CDLL(name)
            except OSError:
                continue
        for root in ("/usr/local/cuda*/lib64", "/usr/local/cuda*/targets/*/lib"):
            for path in sorted(glob.glob(os.path.join(root, "libcudart.so*"))):
                try:
                    return ctypes.CDLL(path)
                except OSError:
                    continue
        raise CudartError("libcudart not found; install CUDA runtime or set LD_LIBRARY_PATH")

    def _check(self, result: int) -> None:
        if result != 0:
            message = self.lib.cudaGetErrorString(result).decode(errors="replace")
            raise CudartError(f"cuda error {result}: {message}")

    def malloc(self, nbytes: int) -> ctypes.c_void_p:
        ptr = ctypes.c_void_p()
        self._check(self.lib.cudaMalloc(ctypes.byref(ptr), nbytes))
        return ptr

    def free(self, ptr: ctypes.c_void_p) -> None:
        if ptr:
            self.lib.cudaFree(ptr)

    def host_to_device(self, device: ctypes.c_void_p, array: np.ndarray) -> None:
        self._check(self.lib.cudaMemcpy(device, array.ctypes.data, array.nbytes, 1))

    def device_to_host(self, array: np.ndarray, device: ctypes.c_void_p) -> None:
        self._check(self.lib.cudaMemcpy(array.ctypes.data, device, array.nbytes, 2))

    def stream(self) -> ctypes.c_void_p:
        handle = ctypes.c_void_p()
        self._check(self.lib.cudaStreamCreate(ctypes.byref(handle)))
        return handle

    def sync(self, handle: ctypes.c_void_p) -> None:
        self._check(self.lib.cudaStreamSynchronize(handle))

    def destroy_stream(self, handle: ctypes.c_void_p) -> None:
        if handle:
            self.lib.cudaStreamDestroy(handle)


class TrtSegmenter:
    """Runs a built TensorRT engine with a fixed input shape.

    `input_wh` is (width, height). The engine must have been built for exactly that
    shape; TensorRT rebuilds on shape change are 5+ minutes on this board, so a
    mismatch is a hard error instead of a silent re-profile.
    """

    def __init__(self, engine_path: str, input_wh: tuple[int, int],
                 output_name: str | None = None) -> None:
        import tensorrt as trt  # lazy: TRT is a JetPack runtime dependency, not an import-time one

        self._trt = trt
        self._cuda = _Cudart()
        self._logger = trt.Logger(trt.Logger.WARNING)
        runtime = trt.Runtime(self._logger)
        with open(engine_path, "rb") as handle:
            blob = handle.read()
        engine = runtime.deserialize_cuda_engine(blob)
        if engine is None:
            raise RuntimeError(f"failed to deserialize TensorRT engine: {engine_path}")
        self._engine = engine
        self._context = engine.create_execution_context()
        if self._context is None:
            raise RuntimeError("failed to create TensorRT execution context")

        # Every IO tensor needs a device buffer and a bound address; leaving one unset
        # makes execute_async_v3 return false with no diagnostic (measured on the
        # TwinLiteNet+ engine, which has two output heads).
        self._inputs: list[str] = []
        self._outputs: list[str] = []
        self._shapes: dict[str, tuple[int, ...]] = {}
        self._host: dict[str, np.ndarray] = {}
        self._device: dict[str, ctypes.c_void_p] = {}
        for index in range(engine.num_io_tensors):
            name = engine.get_tensor_name(index)
            shape = tuple(engine.get_tensor_shape(name))
            self._shapes[name] = shape
            host = np.zeros(shape, dtype=np.float32)
            self._host[name] = host
            self._device[name] = self._cuda.malloc(host.nbytes)
            self._context.set_tensor_address(name, self._device[name].value)
            if engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT:
                self._inputs.append(name)
            else:
                self._outputs.append(name)
        if len(self._inputs) != 1:
            raise RuntimeError(f"expected exactly one input tensor, got {self._inputs}")
        self._input_name = self._inputs[0]
        if not self._outputs:
            raise RuntimeError("engine has no output tensor")
        self._output_name = output_name if output_name in self._outputs else self._outputs[0]
        if output_name and output_name not in self._outputs:
            raise RuntimeError(
                f"output {output_name!r} not in engine outputs {self._outputs}"
            )

        shape = self._shapes[self._input_name]
        expected = (1, 3, input_wh[1], input_wh[0])
        if shape != expected:
            raise RuntimeError(
                f"engine input shape {shape} does not match configured {expected}; "
                "rebuild with scripts/build_roadseg_engine.py"
            )
        self._input_shape = shape
        self._output_shape = self._shapes[self._output_name]
        self._stream = self._cuda.stream()

    @property
    def input_wh(self) -> tuple[int, int]:
        return self._input_shape[3], self._input_shape[2]

    def class_probability(self, chw: np.ndarray, class_index: int = 1) -> np.ndarray:
        """Return the softmax probability of `class_index` as an (H, W) float32 map."""
        if chw.shape != self._input_shape:
            raise ValueError(f"input {chw.shape} != engine input {self._input_shape}")
        np.copyto(self._host[self._input_name], chw, casting="unsafe")
        self._cuda.host_to_device(self._device[self._input_name], self._host[self._input_name])
        if not self._context.execute_async_v3(self._stream.value):
            raise RuntimeError("TensorRT execution failed (check that every IO tensor is bound)")
        self._cuda.sync(self._stream)
        self._cuda.device_to_host(self._host[self._output_name], self._device[self._output_name])

        logits = self._host[self._output_name]  # (1, C, H, W)
        channels = logits.shape[1]
        if channels == 1:  # already a probability map
            return logits[0, 0].astype(np.float32, copy=False)
        if channels == 2:
            # Binary head: one sigmoid instead of a 2-way softmax. Same result, one
            # exp pass over H*W instead of two plus a max-reduce (measured ~10 ms saved
            # per frame on the Orin -- that is a quarter of the frame budget).
            other = logits[0, 1 - class_index]
            return 1.0 / (1.0 + np.exp(other - logits[0, class_index]))
        top = logits.max(axis=1, keepdims=True)
        exp = np.exp(logits - top, dtype=np.float32)
        return (exp / exp.sum(axis=1, keepdims=True))[0, class_index].astype(np.float32, copy=False)

    def close(self) -> None:
        for pointer in self._device.values():
            self._cuda.free(pointer)
        self._cuda.destroy_stream(self._stream)

    def __del__(self) -> None:  # best effort; explicit close() is preferred
        try:
            self.close()
        except Exception:
            pass


def _bench(engine_path: str, input_wh: tuple[int, int], iterations: int = 50) -> None:
    """Self-check: load an engine, run it, report latency. Needs a built engine."""
    segmenter = TrtSegmenter(engine_path, input_wh)
    chw = np.random.rand(*segmenter._input_shape).astype(np.float32)
    segmenter.class_probability(chw)  # warm-up
    started = time.perf_counter()
    for _ in range(iterations):
        prob = segmenter.class_probability(chw)
    elapsed = (time.perf_counter() - started) / iterations
    print(f"{engine_path}: {elapsed*1000:.2f} ms/frame -> {1/elapsed:.1f} FPS, "
          f"prob range [{prob.min():.3f}, {prob.max():.3f}]")
    segmenter.close()


if __name__ == "__main__":
    import sys

    if len(sys.argv) < 4:
        raise SystemExit("usage: python -m rak_road_perception.roadseg_trt <engine> <width> <height>")
    _bench(sys.argv[1], (int(sys.argv[2]), int(sys.argv[3])))
