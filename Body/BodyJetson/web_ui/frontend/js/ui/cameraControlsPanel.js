const CAMERA_INPUT_ROWS = [
  { id: "camera-position-input", label: "Position", value: "45" },
];

const CAMERA_VISION_TOGGLES = [
  { id: "camera-detections-toggle", label: "Detections", text: "Off" },
  { id: "camera-labels-toggle", label: "Labels", text: "On" },
  { id: "camera-tracks-toggle", label: "Tracks", text: "Off" },
  { id: "camera-tracklabels-toggle", label: "Track Labels", text: "On" },
];

function renderCameraInputRows() {
  const container = document.getElementById("camera-tilt-inputs");
  const template = document.getElementById("camera-control-row-template");

  if (!container || !template) {
    console.error("cameraControlsPanel: input container or template missing");
    return;
  }

  container.innerHTML = "";

  for (const inputDef of CAMERA_INPUT_ROWS) {
    const fragment = template.content.cloneNode(true);
    const row = fragment.querySelector(".camera-control-row");
    const label = fragment.querySelector("label");
    const input = fragment.querySelector("input");

    if (!row || !label || !input) {
      continue;
    }

    label.setAttribute("for", inputDef.id);
    label.textContent = inputDef.label;

    input.id = inputDef.id;
    input.type = "number";
    input.value = inputDef.value;

    container.appendChild(fragment);
  }
}

function renderCameraVisionToggles() {
  const container = document.getElementById("camera-vision-controls");
  const template = document.getElementById("camera-toggle-row-template");

  if (!container || !template) {
    console.error("cameraControlsPanel: vision container or template missing");
    return;
  }

  container.innerHTML = "";

  for (const toggleDef of CAMERA_VISION_TOGGLES) {
    const fragment = template.content.cloneNode(true);
    const row = fragment.querySelector(".camera-vision-toggle-row");
    const label = fragment.querySelector(".camera-vision-label");
    const button = fragment.querySelector(".camera-vision-toggle-button");

    if (!row || !label || !button) {
      continue;
    }

    row.id = `${toggleDef.id}-row`;
    label.textContent = toggleDef.label;

    button.id = toggleDef.id;
    button.textContent = toggleDef.text;

    container.appendChild(fragment);
  }
}

export function initCameraControlsPanel() {
  console.log("cameraControlsPanel: init start");

  renderCameraInputRows();
  renderCameraVisionToggles();
}