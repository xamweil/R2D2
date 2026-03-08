import { callDeviceCommand } from "../api/deviceService.js";

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

    startCameraStream();
}

