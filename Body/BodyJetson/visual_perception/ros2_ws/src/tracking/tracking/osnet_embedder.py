from __future__ import annotations

import numpy as np

from scene_understanding.tensor_rt_engine import TensorRTEngine
from tracking.reid_preprocessor import ReIDPreprocessor


class OSNetEmbedder:
    """
    High-level wrapper for OSNet ReID inference.

    Input:
        person crop in OpenCV BGR format

    Output:
        L2-normalized embedding vector of shape (512,)
    """

    def __init__(self, engine_path: str) -> None:
        self.preprocessor = ReIDPreprocessor()
        self.engine = TensorRTEngine(engine_path)

        if len(self.engine.input_names) != 1:
            raise RuntimeError(f"Expected exactly one OSNet input tensor, got {self.engine.input_names}")

        if len(self.engine.output_names) != 1:
            raise RuntimeError(f"Expected exactly one OSNet output tensor, got {self.engine.output_names}")

        self.input_name = self.engine.input_names[0]
        self.output_name = self.engine.output_names[0]

    def embed(self, image_bgr: np.ndarray) -> np.ndarray:
        """
        Compute one normalized embedding vector from a BGR crop.

        Returns:
            np.ndarray with shape (512,), dtype float32
        """
        tensor = self.preprocessor.preprocess(image_bgr)

        outputs = self.engine.infer(tensor)
        embedding = outputs[self.output_name]

        # Expected output shape: (1, 512)
        if embedding.ndim != 2 or embedding.shape[0] != 1:
            raise RuntimeError(f"Unexpected embedding shape: {embedding.shape}")

        embedding = embedding[0].astype(np.float32)

        norm = np.linalg.norm(embedding)
        if norm > 1e-12:
            embedding = embedding / norm

        return embedding

    @staticmethod
    def cosine_distance(a: np.ndarray, b: np.ndarray) -> float:
        """
        Cosine distance for normalized embeddings.
        """
        return float(1.0 - np.dot(a, b))