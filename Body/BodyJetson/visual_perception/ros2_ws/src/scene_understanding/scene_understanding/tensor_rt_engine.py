from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict

import numpy as np
import tensorrt as trt
from cuda import cudart


TRT_LOGGER = trt.Logger(trt.Logger.WARNING)


def trt_dtype_to_numpy(dtype: trt.DataType) -> np.dtype:
    """Map TensorRT dtypes to numpy dtypes."""
    mapping = {
        trt.DataType.FLOAT: np.float32,
        trt.DataType.HALF: np.float16,
        trt.DataType.INT8: np.int8,
        trt.DataType.INT32: np.int32,
        trt.DataType.BOOL: np.bool_,
    }
    if dtype not in mapping:
        raise TypeError(f"Unsupported TensorRT dtype: {dtype}")
    return mapping[dtype]


def check_cuda(status, msg: str) -> None:
    """Raise if a CUDA Runtime API call failed."""
    if isinstance(status, tuple):
        status = status[0]

    if status != cudart.cudaError_t.cudaSuccess:
        raise RuntimeError(f"{msg} failed with CUDA error: {status}")


@dataclass
class TensorBinding:
    name: str
    shape: tuple[int, ...]
    dtype: np.dtype
    is_input: bool
    host: np.ndarray
    device_ptr: int


class TensorRTEngine:
    """
    Thin TensorRT runtime wrapper.

    Responsibilities:
    - deserialize a .engine file
    - create an execution context
    - allocate host/device buffers
    - expose one infer() method that accepts a numpy input tensor

    Note:
    This first version uses regular numpy host buffers and CUDA Runtime API
    device allocations. Later, if needed, we can optimize host buffers to
    pinned memory.
    """

    def __init__(self, engine_path: str | Path) -> None:
        self.engine_path = Path(engine_path)
        if not self.engine_path.exists():
            raise FileNotFoundError(f"TensorRT engine not found: {self.engine_path}")

        self.runtime = trt.Runtime(TRT_LOGGER)
        self.engine = self._load_engine()
        self.context = self.engine.create_execution_context()
        if self.context is None:
            raise RuntimeError("Failed to create TensorRT execution context")

        self.bindings: Dict[str, TensorBinding] = {}
        self.binding_addrs: Dict[str, int] = {}

        status, self.stream = cudart.cudaStreamCreate()
        check_cuda(status, "cudaStreamCreate")

        self._allocate_buffers()

    def _load_engine(self) -> trt.ICudaEngine:
        with self.engine_path.open("rb") as f:
            engine_bytes = f.read()

        engine = self.runtime.deserialize_cuda_engine(engine_bytes)
        if engine is None:
            raise RuntimeError(f"Failed to deserialize TensorRT engine: {self.engine_path}")

        return engine

    def _allocate_buffers(self) -> None:
        """
        Allocate one host buffer and one device buffer per tensor.

        TensorRT 10 uses tensor names rather than legacy binding indices.
        """
        num_tensors = self.engine.num_io_tensors

        for i in range(num_tensors):
            tensor_name = self.engine.get_tensor_name(i)
            tensor_shape = tuple(self.engine.get_tensor_shape(tensor_name))
            tensor_dtype = trt_dtype_to_numpy(self.engine.get_tensor_dtype(tensor_name))
            tensor_mode = self.engine.get_tensor_mode(tensor_name)

            volume = int(np.prod(tensor_shape))
            host_mem = np.empty(volume, dtype=tensor_dtype)

            status, device_ptr = cudart.cudaMalloc(host_mem.nbytes)
            check_cuda(status, f"cudaMalloc({tensor_name})")

            binding = TensorBinding(
                name=tensor_name,
                shape=tensor_shape,
                dtype=tensor_dtype,
                is_input=(tensor_mode == trt.TensorIOMode.INPUT),
                host=host_mem,
                device_ptr=int(device_ptr),
            )

            self.bindings[tensor_name] = binding
            self.binding_addrs[tensor_name] = int(device_ptr)

    @property
    def input_names(self) -> list[str]:
        return [name for name, binding in self.bindings.items() if binding.is_input]

    @property
    def output_names(self) -> list[str]:
        return [name for name, binding in self.bindings.items() if not binding.is_input]

    def infer(self, input_tensor: np.ndarray) -> dict[str, np.ndarray]:
        """
        Run inference on one input tensor.

        Expected for current YOLOv8 engine:
        - shape: (1, 3, 640, 640)
        - dtype: float32
        """
        if len(self.input_names) != 1:
            raise RuntimeError(f"Expected exactly 1 input tensor, found {self.input_names}")

        input_name = self.input_names[0]
        input_binding = self.bindings[input_name]

        if tuple(input_tensor.shape) != input_binding.shape:
            raise ValueError(
                f"Input shape mismatch for {input_name}: "
                f"expected {input_binding.shape}, got {tuple(input_tensor.shape)}"
            )

        if input_tensor.dtype != input_binding.dtype:
            raise ValueError(
                f"Input dtype mismatch for {input_name}: "
                f"expected {input_binding.dtype}, got {input_tensor.dtype}"
            )

        # Stage input into the preallocated host buffer.
        np.copyto(input_binding.host.reshape(input_binding.shape), input_tensor)

        # Copy host -> device (ASYNC)
        status = cudart.cudaMemcpyAsync(
            input_binding.device_ptr,
            input_binding.host.ctypes.data,
            input_binding.host.nbytes,
            cudart.cudaMemcpyKind.cudaMemcpyHostToDevice,
            self.stream
        )
        check_cuda(status, f"cudaMemcpyAsync H2D ({input_name})")

        # Register device addresses with TensorRT
        for tensor_name, addr in self.binding_addrs.items():
            self.context.set_tensor_address(tensor_name, addr)

        # Launch inference on our CUDA stream
        success = self.context.execute_async_v3(stream_handle=self.stream)
        if not success:
            raise RuntimeError("TensorRT execution failed")

        outputs: dict[str, np.ndarray] = {}

        # Copy outputs device -> host (ASYNC)
        for output_name in self.output_names:
            output_binding = self.bindings[output_name]

            status = cudart.cudaMemcpyAsync(
                output_binding.host.ctypes.data,
                output_binding.device_ptr,
                output_binding.host.nbytes,
                cudart.cudaMemcpyKind.cudaMemcpyDeviceToHost,
                self.stream
            )
            check_cuda(status, f"cudaMemcpyAsync D2H ({output_name})")

        # Synchronize stream (wait for all operations to finish)
        status = cudart.cudaStreamSynchronize(self.stream)
        check_cuda(status, "cudaStreamSynchronize")

        # Convert outputs to numpy
        for output_name in self.output_names:
            output_binding = self.bindings[output_name]
            output_array = np.array(output_binding.host, copy=True).reshape(output_binding.shape)
            outputs[output_name] = output_array

        return outputs

    def summary(self) -> str:
        lines = [
            f"TensorRT engine: {self.engine_path}",
            f"inputs: {self.input_names}",
            f"outputs: {self.output_names}",
        ]

        for name, binding in self.bindings.items():
            lines.append(
                f"  - {name}: shape={binding.shape}, dtype={binding.dtype}, "
                f"is_input={binding.is_input}"
            )

        return "\n".join(lines)

    def __del__(self) -> None:
        for binding in self.bindings.values():
            try:
                cudart.cudaFree(binding.device_ptr)
            except Exception:
                pass