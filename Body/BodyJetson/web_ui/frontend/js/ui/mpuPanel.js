const MPU_LIST = [
  { id: "mpu-head", title: "Head", alias: null },
  { id: "mpu-body", title: "Body", alias: "imu_body" },
  { id: "mpu-left-leg", title: "Left Leg", alias: "imu_left_leg" },
  { id: "mpu-right-leg", title: "Right Leg", alias: "imu_right_leg" },
  { id: "mpu-left-foot", title: "Left Foot", alias: "imu_left_foot" },
  { id: "mpu-right-foot", title: "Right Foot", alias: "imu_right_foot" },
];

const ACCEL_SCALE = 2 / 32768;   // g per LSB

function fmtNum(value) {
    if (value === null || value === undefined) return "N.a.";
    const n = Number(value);
    if (!Number.isFinite(n)) return "N.a.";
    return n.toFixed(3);
}

function fmtAgeSeconds(value) {
  if (value === null || value === undefined) return "N.a.";
  const n = Number(value);
  if (!Number.isFinite(n)) return "N.a.";

  if (n < 1) {
    return `${Math.round(n * 1000)} ms`;
  }

  return `${n.toFixed(2)} s`;
}

function setField(card, fieldName, value) {
    const el = card.querySelector(`[data-field="${fieldName}"]`);
    if (!el) return;
    el.textContent = value;
}

function pick(obj, keys) {
    for (const key of keys) {
        if(obj && Object.prototype.hasOwnProperty.call(obj, key)) {
            return obj[key];
        }
    }
}

function renderEmptyCard(card) {
  setField(card, "acc-x", "N.a.");
  setField(card, "acc-y", "N.a.");
  setField(card, "acc-z", "N.a.");
  setField(card, "age", "N.a.");
}

function renderEntry(card, entry) {
  if (!entry || !entry.present) {
    renderEmptyCard(card);
    return;
  }

  const data = entry.data || {};

  let ax;
  let ay;
  let az;

  if (Array.isArray(data.accel) && data.accel.length >= 3) {
    ax = data.accel[0] * ACCEL_SCALE;
    ay = data.accel[1] * ACCEL_SCALE;
    az = data.accel[2] * ACCEL_SCALE;
  }

  setField(card, "acc-x", fmtNum(ax));
  setField(card, "acc-y", fmtNum(ay));
  setField(card, "acc-z", fmtNum(az));
  setField(card, "age", fmtAgeSeconds(entry.age));
}

export function initMpuPanel() {
  const grid = document.getElementById("mpu-grid");
  const template = document.getElementById("mpu-card-template");

  if (!grid || !template) {
    console.error("MPU Panel: grid or template not found");
    return;
  }

  grid.innerHTML = "";

  for (const mpu of MPU_LIST) {
    const fragment = template.content.cloneNode(true);
    const card = fragment.querySelector(".mpu-card");
    const title = fragment.querySelector(".mpu-title");

    if (!card || !title) continue;

    card.id = mpu.id;
    card.dataset.alias = mpu.alias ?? "";
    title.textContent = mpu.title;

    grid.appendChild(fragment);
  }
}

export function renderMpuState(state) {
  const entries = state?.entries || {};

  for (const mpu of MPU_LIST) {
    const card = document.getElementById(mpu.id);
    if (!card) continue;

    if (!mpu.alias) {
      renderEmptyCard(card);
      continue;
    }

    renderEntry(card, entries[mpu.alias]);
  }
}