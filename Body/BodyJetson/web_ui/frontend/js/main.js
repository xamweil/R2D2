import { initCameraPanel } from "./ui/cameraPanel.js";


console.log("main.js loaded");

const cameraPanel = document.getElementById("camera-panel");

if(cameraPanel) {
    console.log("camera panel found");
    initCameraPanel();
} 
else {
    console.log("camera panel not found");
}