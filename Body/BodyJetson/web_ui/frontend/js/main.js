import { initCameraPanel } from "./ui/cameraPanel.js";
import { initMpuPanel, renderMpuState } from "./ui/mpuPanel.js";
import { connectWebSocket } from "./api/websocket.js";
import { initControlsPanel } from "./ui/controlsPanel.js";
import { initHeadControlPanel } from "./ui/headControlPanel.js";
import { initCameraControlsPanel } from "./ui/cameraControlsPanel.js";

console.log("main.js loaded");

const cameraPanel = document.getElementById("camera-panel");
const mpuPanel = document.getElementById("mpu-panel");
const controlsPanel = document.getElementById("controls-panel");
const headControlPanel = document.getElementById("head-control-panel");


if(cameraPanel) {
    console.log("camera panel found");
    initCameraPanel();
    initCameraControlsPanel();
} 
else {
    console.log("camera panel not found");
}

if (mpuPanel) {
  console.log("mpu panel found");
  initMpuPanel();
} else {
  console.log("mpu panel not found");
}

if (controlsPanel) {
  console.log("controls panel found");
  initControlsPanel();
} else {
  console.log("controls panel not found");
}

if (headControlPanel) {
  console.log("head control panel found");
  initHeadControlPanel();
} else {
  console.log("head control panel not found");
}

connectWebSocket({
  onHello(payload) {
    console.log("ws hello", payload);
  },
  onRobotState(payload) {
    renderMpuState(payload);
    const detections = payload?.entries?.scene_detections;
    if (detections) {
      console.log("scene_detections entry", detections);
  }
  },
  onError(error) {
    console.error("ws error", error);
  },
});