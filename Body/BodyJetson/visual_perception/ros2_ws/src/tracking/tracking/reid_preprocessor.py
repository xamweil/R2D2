from __future__ import annotations

import cv2
import numpy as np


class ReIDPreprocessor:
    """
    Preprocesses person crops for OSNet ReID inference.

    Target input tensor:
        (1, 3, 256, 128)
    dtype:
        float32

    OSNet convention:
    - input image size: H=256, W=128
    - RGB
    - normalized with ImageNet mean/std
    """

    def __init__(self, input_height: int = 256, input_width: int = 128) -> None:
        self.input_height = input_height
        self.input_width = input_width

        self.mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
        self.std = np.array([0.229, 0.224, 0.225], dtype=np.float32)

    def preprocess(self, image_bgr: np.ndarray) -> np.ndarray:
        """
        Convert a BGR crop to OSNet input tensor.

        Input:
            image_bgr: HWC uint8 OpenCV image

        Output:
            tensor: (1, 3, 256, 128) float32
        """
        if image_bgr is None or image_bgr.size == 0:
            raise ValueError("Input crop is empty")

        img = cv2.resize(
            image_bgr,
            (self.input_width, self.input_height),
            interpolation=cv2.INTER_LINEAR,
        )

        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = img.astype(np.float32) / 255.0

        img = (img - self.mean) / self.std

        img = np.transpose(img, (2, 0, 1))
        img = np.expand_dims(img, axis=0)

        return img.astype(np.float32)