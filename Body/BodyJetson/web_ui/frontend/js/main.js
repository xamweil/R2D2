import { initCameraPanel } from "./ui/cameraPanel.js";
import { initMpuPanel, renderMpuState } from "./ui/mpuPanel.js";
import { connectWebSocket } from "./api/websocket.js";
import { initControlsPanel } from "./ui/controlsPanel.js";

console.log("main.js loaded");

const cameraPanel = document.getElementById("camera-panel");
const mpuPanel = document.getElementById("mpu-panel");
const controlsPanel = document.getElementById("controls-panel");

if(cameraPanel) {
    console.log("camera panel found");
    initCameraPanel();
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

connectWebSocket({
  onHello(payload) {
    console.log("ws hello", payload);
  },
  onRobotState(payload) {
    renderMpuState(payload);
  },
  onError(error) {
    console.error("ws error", error);
  },
});