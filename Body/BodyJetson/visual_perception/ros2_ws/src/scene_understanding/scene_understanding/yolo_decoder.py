from __future__ import annotations

import numpy as np
import cv2


class YOLODecoder:
    """
    Decodes YOLOv8 TensorRT output tensor.

    Input tensor shape:
        (1, 84, 8400)

    Layout:
        4 box parameters
        80 class scores
    """

    def __init__(self,
                 conf_threshold: float = 0.25,
                 nms_threshold: float = 0.45):

        self.conf_threshold = conf_threshold
        self.nms_threshold = nms_threshold

    def decode(self, output: np.ndarray):

        output = output.squeeze(0)          # (84, 8400)
        output = output.transpose(1, 0)     # (8400, 84)

        boxes = output[:, :4]
        scores = output[:, 4:]

        class_ids = np.argmax(scores, axis=1)
        confidences = scores[np.arange(scores.shape[0]), class_ids]

        mask = confidences > self.conf_threshold

        boxes = boxes[mask]
        confidences = confidences[mask]
        class_ids = class_ids[mask]

        if boxes.shape[0] == 0:
            return [], [], []

        boxes_xyxy = self.xywh_to_xyxy(boxes)

        indices = cv2.dnn.NMSBoxes(
            boxes_xyxy.tolist(),
            confidences.tolist(),
            self.conf_threshold,
            self.nms_threshold
        )

        final_boxes = []
        final_scores = []
        final_classes = []

        if len(indices) > 0:
            for i in indices.flatten():
                final_boxes.append(boxes_xyxy[i])
                final_scores.append(confidences[i])
                final_classes.append(class_ids[i])

        return final_boxes, final_scores, final_classes

    def xywh_to_xyxy(self, boxes):

        x = boxes[:, 0]
        y = boxes[:, 1]
        w = boxes[:, 2]
        h = boxes[:, 3]

        x1 = x - w / 2
        y1 = y - h / 2
        x2 = x + w / 2
        y2 = y + h / 2

        return np.stack([x1, y1, x2, y2], axis=1)