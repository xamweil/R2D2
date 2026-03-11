from __future__ import annotations

import cv2
import numpy as np


class ImagePreprocessor:
    """
    Converts OpenCV images into the TensorRT input format expected by YOLO.

    Steps:
    - letterbox resize (preserve aspect ratio)
    - BGR -> RGB
    - normalize to [0,1]
    - HWC -> CHW
    - add batch dimension

    Output tensor shape:
        (1, 3, 640, 640)
    dtype:
        float32
    """

    def __init__(self, input_size: int = 640):
        self.input_size = input_size

    def letterbox(self, image: np.ndarray) -> np.ndarray:
        """
        Resize image with unchanged aspect ratio and pad to square.

        This follows the standard YOLO letterbox approach.
        """

        h, w = image.shape[:2]

        scale = min(self.input_size / w, self.input_size / h)

        new_w = int(round(w * scale))
        new_h = int(round(h * scale))

        resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

        pad_w = self.input_size - new_w
        pad_h = self.input_size - new_h

        pad_left = pad_w // 2
        pad_right = pad_w - pad_left
        pad_top = pad_h // 2
        pad_bottom = pad_h - pad_top

        padded = cv2.copyMakeBorder(
            resized,
            pad_top,
            pad_bottom,
            pad_left,
            pad_right,
            cv2.BORDER_CONSTANT,
            value=(114, 114, 114),
        )

        return padded

    def preprocess(self, image: np.ndarray) -> np.ndarray:
        """
        Full preprocessing pipeline.
        """

        img = self.letterbox(image)

        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        img = img.astype(np.float32) / 255.0

        img = np.transpose(img, (2, 0, 1))

        img = np.expand_dims(img, axis=0)

        return img