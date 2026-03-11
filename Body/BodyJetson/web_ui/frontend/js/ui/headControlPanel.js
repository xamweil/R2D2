import { publishMotorCommand } from "../api/motorCommandService.js";

const TOGGLE_GROUPS = [
  {
    id: "direction",
    label: "Direction",
    buttons: [
      { id: "head-direction-left", text: "Left" },
      { id: "head-direction-right", text: "Right", active: true },
    ],
  },
  {
    id: "anglemode",
    label: "Angle Mode",
    buttons: [
      { id: "head-anglemode-off", text: "Off" },
      { id: "head-anglemode-on", text: "On", active: true },
    ],
  },
];

const INPUT_ROWS = [
  { id: "head-angle-input", label: "Angle", value: "0" },
  { id: "head-velocity-input", label: "Velocity", value: "50" },
];

function renderToggleGroups() {
  const row = document.getElementById("head-toggle-row");
  const template = document.getElementById("head-toggle-group-template");

  if (!row || !template) {
    console.error("headControlPanel: toggle row or template missing");
    return;
  }

  row.innerHTML = "";

  for (const group of TOGGLE_GROUPS) {
    const fragment = template.content.cloneNode(true);
    const groupRoot = fragment.querySelector(".head-toggle-group");
    const label = fragment.querySelector(".head-toggle-label");
    const buttonContainer = fragment.querySelector(".head-toggle-buttons");

    if (!groupRoot || !label || !buttonContainer) {
      continue;
    }

    groupRoot.id = `head-toggle-group-${group.id}`;
    label.textContent = group.label;

    for (const buttonDef of group.buttons) {
      const button = document.createElement("button");
      button.type = "button";
      button.id = buttonDef.id;
      button.className = "head-toggle-button";
      if (buttonDef.active) {
        button.classList.add("is-active");
      }
      button.textContent = buttonDef.text;
      buttonContainer.appendChild(button);
    }

    row.appendChild(fragment);
  }
}

function renderInputRows() {
  const grid = document.getElementById("head-input-grid");
  const template = document.getElementById("head-input-row-template");

  if (!grid || !template) {
    console.error("headControlPanel: input grid or template missing");
    return;
  }

  grid.innerHTML = "";

  for (const inputDef of INPUT_ROWS) {
    const fragment = template.content.cloneNode(true);
    const row = fragment.querySelector(".head-input-row");
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

    grid.appendChild(fragment);
  }
}

function setActiveButton(clickedButton) {
  const container = clickedButton.parentElement;
  if (!container) return;

  const buttons = container.querySelectorAll(".head-toggle-button");
  for (const button of buttons) {
    button.classList.remove("is-active");
  }

  clickedButton.classList.add("is-active");
}

function attachToggleBehavior() {
  const toggleButtons = document.querySelectorAll(".head-toggle-button");

  for (const button of toggleButtons) {
    button.addEventListener("click", () => {
      setActiveButton(button);
    });
  }
}

function setAngleValue(angle) {
  const normalizedAngle = Math.round(angle);
  const image = document.getElementById("head-rotation-image");
  const readout = document.getElementById("head-angle-readout");
  const angleInput = document.getElementById("head-angle-input");

  if (image) {
    image.style.transform = `rotate(${-normalizedAngle}deg)`;
  }

  if (readout) {
    readout.textContent = `Target angle: ${normalizedAngle}°`;
  }

  if (angleInput) {
    angleInput.value = String(normalizedAngle);
  }
}

function attachAngleInputBehavior() {
  const angleInput = document.getElementById("head-angle-input");
  if (!angleInput) return;

  const applyAngle = () => {
    const value = Number(angleInput.value);

    if (!Number.isFinite(value)) {
      setAngleValue(0);
      return;
    }

    setAngleValue(value);
  };

  angleInput.addEventListener("input", applyAngle);
  applyAngle();
}

function pointerToAngle(clientX, clientY, element) {
  const rect = element.getBoundingClientRect();
  const centerX = rect.left + rect.width / 2;
  const centerY = rect.top + rect.height / 2;

  const dx = clientX - centerX;
  const dy = clientY - centerY;

  // 0° should point upward, positive angles should go to the left.
  const radians = Math.atan2(dx, -dy);
  const degrees = radians * (180 / Math.PI);

  return -degrees;
}

function attachDragBehavior() {
  const stage = document.querySelector(".head-visual-stage");
  if (!stage) return;

  let isDragging = false;
  let didDrag = false;

  const updateFromPointer = (clientX, clientY) => {
    const angle = pointerToAngle(clientX, clientY, stage);
    setAngleValue(angle);
  };

  const sendCurrentHeadCommand = () => {
    const message = buildHeadMotorCommand();
    sendHeadMotorCommand(message, "Sending motor command...");
  };

  stage.addEventListener("mousedown", (event) => {
    event.preventDefault();
    isDragging = true;
    didDrag = false;
    updateFromPointer(event.clientX, event.clientY);
  });

  window.addEventListener("mousemove", (event) => {
    if (!isDragging) return;

    didDrag = true;
    event.preventDefault();
    updateFromPointer(event.clientX, event.clientY);
  });

  window.addEventListener("mouseup", (event) => {
    if (!isDragging) return;

    event.preventDefault();
    isDragging = false;

    if (didDrag) {
      sendCurrentHeadCommand();
    }
  });

  stage.addEventListener("click", (event) => {
    if (didDrag) return;

    updateFromPointer(event.clientX, event.clientY);
    sendCurrentHeadCommand();
  });
}

function getActiveToggleValue(leftId, rightId) {
  const leftButton = document.getElementById(leftId);
  const rightButton = document.getElementById(rightId);

  if (leftButton?.classList.contains("is-active")) {
    return "left";
  }

  if (rightButton?.classList.contains("is-active")) {
    return "right";
  }

  return null;
}

function getAngleModeEnabled() {
  const onButton = document.getElementById("head-anglemode-on");
  return Boolean(onButton?.classList.contains("is-active"));
}

function getHeadAngleValue() {
  const input = document.getElementById("head-angle-input");
  if (!input) return 0.0;

  const value = Number(input.value);
  return Number.isFinite(value) ? value : 0.0;
}

function getHeadVelocityValue() {
  const input = document.getElementById("head-velocity-input");
  if (!input) return 50;

  const value = Number(input.value);
  return Number.isFinite(value) ? value : 50;
}

function setHeadCommandStatus(text) {
  const status = document.getElementById("head-command-status");
  if (!status) return;
  status.textContent = text;
}

function setHeadCommandButtonsDisabled(disabled) {
  const buttons = [
    document.getElementById("head-stop-button"),
    document.getElementById("head-send-button"),
  ];

  for (const button of buttons) {
    if (button) {
      button.disabled = disabled;
    }
  }
}

function buildHeadMotorCommand() {
  const directionValue = getActiveToggleValue(
    "head-direction-left",
    "head-direction-right"
  );

  const direction = directionValue === "left";
  const angle = Number(getHeadAngleValue());
  const velocity = getHeadVelocityValue();
  const angleModeEnabled = getAngleModeEnabled();

  return {
    ids: [1],
    enable: [true],
    direction: [direction],
    angle_set: [angleModeEnabled],
    velocity_set: [true],
    angle: [angle],
    velocity: [velocity],
  };
}

function buildHeadStopCommand() {
  return {
    ids: [1],
    enable: [true],
    direction: [false],
    angle_set: [false],
    velocity_set: [false],
    angle: [0.0],
    velocity: [0],
  };
}

function disableNativeImageDrag() {
  const image = document.getElementById("head-rotation-image");
  if (!image) return;

  image.addEventListener("dragstart", (event) => {
    event.preventDefault();
  });
}

async function sendHeadMotorCommand(message, statusText) {
  setHeadCommandButtonsDisabled(true);
  setHeadCommandStatus(statusText);

  const result = await publishMotorCommand(message);

  if (result.ok) {
    setHeadCommandStatus("Command sent");
  } else {
    setHeadCommandStatus("Failed to send command");
    console.error("head motor command failed", { message, result });
  }

  setHeadCommandButtonsDisabled(false);
}

function attachCommandButtonBehavior() {
  const sendButton = document.getElementById("head-send-button");
  const stopButton = document.getElementById("head-stop-button");

  sendButton?.addEventListener("click", () => {
    const message = buildHeadMotorCommand();
    sendHeadMotorCommand(message, "Sending motor command...");
  });

  stopButton?.addEventListener("click", () => {
    const message = buildHeadStopCommand();
    sendHeadMotorCommand(message, "Sending stop command...");
  });
}



export function initHeadControlPanel() {
  console.log("headControlPanel: init start");

  renderToggleGroups();
  renderInputRows();

  attachToggleBehavior();
  attachAngleInputBehavior();
  attachDragBehavior();
  attachCommandButtonBehavior();
  disableNativeImageDrag();
}