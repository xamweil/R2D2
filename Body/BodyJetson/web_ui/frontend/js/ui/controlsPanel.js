import { callDeviceCommand } from "../api/deviceService.js";

const LID_LIST = [
  { id: "lid1_1", title: "Bottom Lid 1" },
  { id: "lid1_2", title: "Bottom Lid 2" },
  { id: "lid1_3", title: "Bottom Lid 3" },
  { id: "lid1_4", title: "Bottom Lid 4" },
  { id: "lid1_5", title: "Bottom Lid 5" },
  { id: "lid1_6", title: "Bottom Lid 6" },
  { id: "lid2_1", title: "Top Lid 1" },
  { id: "lid2_2", title: "Top Lid 2" },
  { id: "lid2_3", title: "Top Lid 3" },
  { id: "lid2_4", title: "Top Lid 4" },
  { id: "lid2_5", title: "Top Lid 5" },
];

const NOB_LIST = [
  { id: "nob1", title: "Nob 1" },
  { id: "nob2", title: "Nob 2" },
  { id: "nob3", title: "Nob 3" },
];

const ACTION_LIST = [
  { id: "lids-bottom-open", title: "Bottom Open" },
  { id: "lids-top-open", title: "Top Open" },
  { id: "nobs-run", title: "Run Nobs" },
  { id: "lids-bottom-close", title: "Bottom Close" },
  { id: "lids-top-close", title: "Top Close" },
  { id: "system-test", title: "System Test" },
];

function setControlStatus(card, text) {
  const status = card.querySelector(".control-status");
  if (!status) return;
  status.textContent = text;
}

function setButtonsDisabled(card, disabled) {
  const buttons = card.querySelectorAll("button");
  for (const button of buttons) {
    button.disabled = disabled;
  }
}

async function runLidCommand(card, lidId, methodName) {
  console.log("lid action", { device: lidId, method: methodName });

  setButtonsDisabled(card, true);
  setControlStatus(card, `Sending: ${methodName}...`);

  const result = await callDeviceCommand(lidId, methodName, []);

  if (result.ok) {
    setControlStatus(card, `Last action: ${methodName}`);
  } else {
    setControlStatus(card, `Failed: ${methodName}`);
    console.error("lid command failed", {
      device: lidId,
      method: methodName,
      result,
    });
  }

  setButtonsDisabled(card, false);
}

function attachLidHandlers(card, lidId) {
  const openButton = card.querySelector(".control-open-button");
  const closeButton = card.querySelector(".control-close-button");

  openButton?.addEventListener("click", () => {
    runLidCommand(card, lidId, "open");
  });

  closeButton?.addEventListener("click", () => {
    runLidCommand(card, lidId, "close");
  });
}

function attachNobHandlers(card, nobId) {
  const runButton = card.querySelector(".control-run-button");

  runButton?.addEventListener("click", () => {
    console.log("nob action", { device: nobId, method: "run_circle" });
    setControlStatus(card, "Last action: run_circle (UI only)");
  });
}

function attachActionHandlers(card, actionId) {
  const button = card.querySelector(".action-trigger-button");

  button?.addEventListener("click", () => {
    console.log("group action", { action: actionId });
  });
}

function renderLidCards() {
  const grid = document.getElementById("lids-grid");
  const template = document.getElementById("lid-card-template");

  if (!grid || !template) {
    console.error("Controls Panel: missing lids-grid or lid-card-template");
    return;
  }

  grid.innerHTML = "";

  for (const lid of LID_LIST) {
    const fragment = template.content.cloneNode(true);
    const card = fragment.querySelector(".control-card");
    const title = fragment.querySelector(".control-title");

    if (!card || !title) continue;

    card.id = lid.id;
    title.textContent = lid.title;

    attachLidHandlers(card, lid.id);

    grid.appendChild(fragment);
  }
}

function renderNobCards() {
  const grid = document.getElementById("nobs-grid");
  const template = document.getElementById("nob-card-template");

  if (!grid || !template) {
    console.error("Controls Panel: missing nobs-grid or nob-card-template");
    return;
  }

  grid.innerHTML = "";

  for (const nob of NOB_LIST) {
    const fragment = template.content.cloneNode(true);
    const card = fragment.querySelector(".control-card");
    const title = fragment.querySelector(".control-title");

    if (!card || !title) continue;

    card.id = nob.id;
    title.textContent = nob.title;

    attachNobHandlers(card, nob.id);

    grid.appendChild(fragment);
  }
}

function renderActionCards() {
  const grid = document.getElementById("actions-grid");
  const template = document.getElementById("action-button-template");

  if (!grid || !template) {
    console.error("Controls Panel: missing actions-grid or action-button-template");
    return;
  }

  grid.innerHTML = "";

  for (const action of ACTION_LIST) {
    const fragment = template.content.cloneNode(true);
    const card = fragment.querySelector(".action-card");
    const title = fragment.querySelector(".action-title");

    if (!card || !title) continue;

    card.id = action.id;
    title.textContent = action.title;

    attachActionHandlers(card, action.id);

    grid.appendChild(fragment);
  }
}

export function initControlsPanel() {
  console.log("controlsPanel: init start");

  renderLidCards();
  renderNobCards();
  renderActionCards();
}