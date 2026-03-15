import { callDeviceCommand } from "../api/deviceService.js";

const MODEL_WIDTH = 640;
const MODEL_HEIGHT = 640;

const SOURCE_WIDTH = 640;
const SOURCE_HEIGHT = 360;

const PAD_X = 0;
const PAD_Y = (MODEL_HEIGHT - SOURCE_HEIGHT) / 2; // 140

//for YOLOv8n
const CLASS_LABELS = {
  "0": "person",
  "1": "bicycle",
  "2": "car",
  "3": "motorcycle",
  "4": "airplane",
  "5": "bus",
  "6": "train",
  "7": "truck",
  "8": "boat",
  "9": "traffic light",
  "10": "fire hydrant",
  "11": "stop sign",
  "12": "parking meter",
  "13": "bench",
  "14": "bird",
  "15": "cat",
  "16": "dog",
  "17": "horse",
  "18": "sheep",
  "19": "cow",
  "20": "elephant",
  "21": "bear",
  "22": "zebra",
  "23": "giraffe",
  "24": "backpack",
  "25": "umbrella",
  "26": "handbag",
  "27": "tie",
  "28": "suitcase",
  "29": "frisbee",
  "30": "skis",
  "31": "snowboard",
  "32": "sports ball",
  "33": "kite",
  "34": "baseball bat",
  "35": "baseball glove",
  "36": "skateboard",
  "37": "surfboard",
  "38": "tennis racket",
  "39": "bottle",
  "40": "wine glass",
  "41": "cup",
  "42": "fork",
  "43": "knife",
  "44": "spoon",
  "45": "bowl",
  "46": "banana",
  "47": "apple",
  "48": "sandwich",
  "49": "orange",
  "50": "broccoli",
  "51": "carrot",
  "52": "hot dog",
  "53": "pizza",
  "54": "donut",
  "55": "cake",
  "56": "chair",
  "57": "couch",
  "58": "potted plant",
  "59": "bed",
  "60": "dining table",
  "61": "toilet",
  "62": "tv",
  "63": "laptop",
  "64": "mouse",
  "65": "remote",
  "66": "keyboard",
  "67": "cell phone",
  "68": "microwave",
  "69": "oven",
  "70": "toaster",
  "71": "sink",
  "72": "refrigerator",
  "73": "book",
  "74": "clock",
  "75": "vase",
  "76": "scissors",
  "77": "teddy bear",
  "78": "hair drier",
  "79": "toothbrush"
};

const CAMERA_TILT_DEVICE = "CameraTilt";

function startCameraStream() {
    const cameraImage = document.getElementById("camera-image");
    const cameraStatus = document.getElementById("camera-status");

    console.log("cameraPanel: start stream");
    console.log("cameraImage:", cameraImage);
    console.log("cameraStatus:", cameraStatus);

    if (!cameraImage || !cameraStatus) {
        console.log("cameraPanel: required elements not found");
        return;
    }

    cameraStatus.textContent = "Connecting to camera stream...";
    cameraStatus.style.display = "flex";
    cameraImage.style.display = "none";

    cameraImage.onload = () => {
        console.log("cameraPanel: stream image loaded");
        cameraImage.style.display = "block";
        cameraStatus.style.display = "none";
        renderAllOverlays();
    };

    cameraImage.onerror = (event) => {
        console.log("cameraPanel: failed to load stream", event);
        cameraImage.style.display = "none";
        cameraStatus.style.display = "flex";
        cameraStatus.textContent = "Failed to load camera feed";
    };

    // Important: one fresh request only when we explicitly start/retry
    cameraImage.src = `/mjpeg?ts=${Date.now()}`;
}

function setCameraControlStatus(text) {
  const status = document.getElementById("camera-control-status");
  if (!status) return;
  status.textContent = text;
}

function setCameraControlButtonsDisabled(disabled) {
  const buttons = [
    document.getElementById("camera-setpos-button"),
    document.getElementById("camera-default-button"),
    document.getElementById("camera-low-button"),
  ];

  for (const button of buttons) {
    if (button) {
      button.disabled = disabled;
    }
  }
}

function getCameraTargetPosition() {
  const input = document.getElementById("camera-position-input");
  if (!input) return null;

  const value = Number(input.value);
  if (!Number.isFinite(value)) return null;

  return value;
}

async function runCameraTiltCommand(methodName, args = []) {
  setCameraControlButtonsDisabled(true);
  setCameraControlStatus(`Sending: ${methodName}...`);

  const result = await callDeviceCommand(CAMERA_TILT_DEVICE, methodName, args);

  if (result.ok) {
    setCameraControlStatus(`Last action: ${methodName}`);
  } else {
    setCameraControlStatus(`Failed: ${methodName}`);
    console.error("camera tilt command failed", {
      device: CAMERA_TILT_DEVICE,
      method: methodName,
      args,
      result,
    });
  }

  setCameraControlButtonsDisabled(false);
}

let latestSceneDetections = null;
let latestTrackingTracks = null;

let showDetections = false;
let showLabels = true;

let showTracks = false;
let showTrackLabels = true;

function getCameraOverlay() {
  return document.getElementById("camera-overlay");
}

function getCameraImage() {
  return document.getElementById("camera-image");
}

function getDetectionCenter(det) {
  return det?.bbox?.center?.position ?? null;
}

function getDetectionResult(det) {
  return det?.results?.[0] ?? null;
}

function getDetectionClassId(det) {
  const result = getDetectionResult(det);
  if (!result) return "";

  if (result.hypothesis?.class_id !== undefined) {
    return String(result.hypothesis.class_id);
  }

  if (result.class_id !== undefined) {
    return String(result.class_id);
  }

  return "";
}

function getDetectionScore(det) {
  const result = getDetectionResult(det);
  if (!result) return null;

  if (result.hypothesis?.score !== undefined) {
    return Number(result.hypothesis.score);
  }

  if (result.score !== undefined) {
    return Number(result.score);
  }

  return null;
}

function getDetectionLabel(det) {
  const classId = getDetectionClassId(det);
  return CLASS_LABELS[classId] ?? classId ?? "";
}

function detectionToCorners(det) {
  const center = getDetectionCenter(det);
  if (!center) return null;

  let cx = Number(center.x);
  let cy = Number(center.y);
  let w = Number(det?.bbox?.size_x);
  let h = Number(det?.bbox?.size_y);

  if (![cx, cy, w, h].every(Number.isFinite)) {
    return null;
  }

  cx *= MODEL_WIDTH;
  cy *= MODEL_HEIGHT;
  w *= MODEL_WIDTH;
  h *= MODEL_HEIGHT;

  return {
    x1: cx - w / 2,
    y1: cy - h / 2,
    x2: cx + w / 2,
    y2: cy + h / 2,
  };
}

function getDisplayedImageRect() {
  const image = getCameraImage();
  const frame = document.getElementById("camera-frame");

  if (!image || !frame) return null;
  if (!image.complete) return null;

  const frameRect = frame.getBoundingClientRect();
  const imageRect = image.getBoundingClientRect();

  return {
    left: imageRect.left - frameRect.left,
    top: imageRect.top - frameRect.top,
    width: imageRect.width,
    height: imageRect.height,
  };
}

function clearCameraOverlay() {
  const overlay = getCameraOverlay();
  if (!overlay) return;
  overlay.innerHTML = "";
}

function renderOverlayBox(det, imageRect, options = {}) {
  const overlay = getCameraOverlay();
  if (!overlay) return;

  const corners = detectionToCorners(det);
  if (!corners) return;

  const x1 = corners.x1 - PAD_X;
  const x2 = corners.x2 - PAD_X;
  const y1 = corners.y1 - PAD_Y;
  const y2 = corners.y2 - PAD_Y;

  const clippedX1 = Math.max(0, Math.min(SOURCE_WIDTH, x1));
  const clippedX2 = Math.max(0, Math.min(SOURCE_WIDTH, x2));
  const clippedY1 = Math.max(0, Math.min(SOURCE_HEIGHT, y1));
  const clippedY2 = Math.max(0, Math.min(SOURCE_HEIGHT, y2));

  const scaleX = imageRect.width / SOURCE_WIDTH;
  const scaleY = imageRect.height / SOURCE_HEIGHT;

  const left = imageRect.left + clippedX1 * scaleX;
  const top = imageRect.top + clippedY1 * scaleY;
  const width = Math.max(0, (clippedX2 - clippedX1) * scaleX);
  const height = Math.max(0, (clippedY2 - clippedY1) * scaleY);

  if (width <= 1 || height <= 1) {
    return;
  }

  const box = document.createElement("div");
  box.className = options.boxClassName ?? "camera-detection-box";
  box.style.left = `${left}px`;
  box.style.top = `${top}px`;
  box.style.width = `${width}px`;
  box.style.height = `${height}px`;

  if (options.showLabel) {
    const label = document.createElement("div");
    label.className = options.labelClassName ?? "camera-detection-label";

    const labelText = options.formatLabel ? options.formatLabel(det) : "";
    label.textContent = labelText;

    if (labelText) {
      box.appendChild(label);
    }
  }

  overlay.appendChild(box);
}

function formatDetectionLabel(det) {
  const text = getDetectionLabel(det);
  const score = getDetectionScore(det);

  if (Number.isFinite(score)) {
    return `${text} ${score.toFixed(2)}`;
  }

  return text;
}

function formatTrackLabel(det) {
  const text = getDetectionLabel(det);
  const trackId = getTrackId(det);

  if (trackId) {
    return `${text} #${trackId}`;
  }

  return text;
}

function renderDetectionCollection(imageRect) {
  const detections = latestSceneDetections?.data?.detections;
  if (!showDetections || !detections?.length) {
    return;
  }

  for (const det of detections) {
    renderOverlayBox(det, imageRect, {
      boxClassName: "camera-detection-box",
      labelClassName: "camera-detection-label",
      showLabel: showLabels,
      formatLabel: formatDetectionLabel,
    });
  }
}

function renderTrackCollection(imageRect) {
  const tracks = latestTrackingTracks?.data?.detections;
  if (!showTracks || !tracks?.length) {
    return;
  }

  for (const det of tracks) {
    renderOverlayBox(det, imageRect, {
      boxClassName: "camera-track-box",
      labelClassName: "camera-track-label",
      showLabel: showTrackLabels,
      formatLabel: formatTrackLabel,
    });
  }
}

function renderAllOverlays() {
  clearCameraOverlay();

  const imageRect = getDisplayedImageRect();
  if (!imageRect || imageRect.width <= 0 || imageRect.height <= 0) {
    return;
  }

  renderDetectionCollection(imageRect);
  renderTrackCollection(imageRect);
}

function updateOverlayToggleUi() {
  const detectionsButton = document.getElementById("camera-detections-toggle");
  const labelsButton = document.getElementById("camera-labels-toggle");
  const tracksButton = document.getElementById("camera-tracks-toggle");
  const trackLabelsButton = document.getElementById("camera-tracklabels-toggle");

  if (detectionsButton) {
    detectionsButton.textContent = showDetections ? "On" : "Off";
    detectionsButton.classList.toggle("is-active", showDetections);
  }

  if (labelsButton) {
    labelsButton.textContent = showLabels ? "On" : "Off";
    labelsButton.classList.toggle("is-active", showLabels);
  }

  if (tracksButton) {
    tracksButton.textContent = showTracks ? "On" : "Off";
    tracksButton.classList.toggle("is-active", showTracks);
  }

  if (trackLabelsButton) {
    trackLabelsButton.textContent = showTrackLabels ? "On" : "Off";
    trackLabelsButton.classList.toggle("is-active", showTrackLabels);
  }
}

function attachOverlayToggleBehavior() {
  const detectionsButton = document.getElementById("camera-detections-toggle");
  const labelsButton = document.getElementById("camera-labels-toggle");
  const tracksButton = document.getElementById("camera-tracks-toggle");
  const trackLabelsButton = document.getElementById("camera-tracklabels-toggle");

  detectionsButton?.addEventListener("click", () => {
    showDetections = !showDetections;
    updateOverlayToggleUi();
    renderAllOverlays();
  });

  labelsButton?.addEventListener("click", () => {
    showLabels = !showLabels;
    updateOverlayToggleUi();
    renderAllOverlays();
  });

  tracksButton?.addEventListener("click", () => {
    showTracks = !showTracks;
    updateOverlayToggleUi();
    renderAllOverlays();
  });

  trackLabelsButton?.addEventListener("click", () => {
    showTrackLabels = !showTrackLabels;
    updateOverlayToggleUi();
    renderAllOverlays();
  });

  updateOverlayToggleUi();
}

export function updateSceneDetections(entry) {
  latestSceneDetections = entry;
  renderAllOverlays();
}

export function updateTrackingTracks(entry) {
  latestTrackingTracks = entry;
  renderAllOverlays();
}

function getTrackId(det) {
  if (det?.id === null || det?.id === undefined) {
    return "";
  }

  return String(det.id);
}

export function initCameraPanel() {
    const retryButton = document.getElementById("camera-retry-button");
    const setPosButton = document.getElementById("camera-setpos-button");
    const defaultButton = document.getElementById("camera-default-button");
    const lowButton = document.getElementById("camera-low-button");

    console.log("cameraPanel: init start");

    retryButton?.addEventListener("click", () => {
        console.log("cameraPanel: retry clicked");
        startCameraStream();
    });

     setPosButton?.addEventListener("click", () => {
    const position = getCameraTargetPosition();

    if (position === null) {
      setCameraControlStatus("Please enter a valid numeric position");
      return;
    }

    runCameraTiltCommand("set_pos", [position]);
  });

  defaultButton?.addEventListener("click", () => {
    runCameraTiltCommand("default_pos", []);
  });

  lowButton?.addEventListener("click", () => {
    runCameraTiltCommand("low_pos", []);
  });

  attachOverlayToggleBehavior();
  window.addEventListener("resize", renderAllOverlays);

    startCameraStream();
}

