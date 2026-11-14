import { create } from './vendor/nipplejs.mjs';

// Adjust to the hub's WebSocket endpoint.
const WS_URL = `${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws`;
const SEND_INTERVAL_MS = 50; // 20 Hz while a stick is held, doubles as a heartbeat
const RECONNECT_MS = 1000;
const DEADZONE = 0.05;
const STICK_COLOR = { front: '#8eb4ff', back: 'rgba(142, 180, 255, 0.18)' };

const linkEl = document.getElementById('ctrl-link');
const stopButton = document.getElementById('ctrl-stop');
const fullscreenButton = document.getElementById('ctrl-fullscreen');

// ---------- WebSocket ----------

let socket = null;

function setLink(state) {
  linkEl.dataset.state = state;
  linkEl.textContent = state;
}

function connect() {
  setLink('connecting');
  socket = new WebSocket(WS_URL);
  socket.addEventListener('open', () => setLink('online'));
  socket.addEventListener('close', () => {
    setLink('offline');
    sticks.forEach(release);
    setTimeout(connect, RECONNECT_MS);
  });
}

function send(message) {
  if (socket?.readyState === WebSocket.OPEN) {
    socket.send(JSON.stringify(message));
  }
}

// ---------- Joysticks ----------

const round = (v) => (Math.abs(v) < DEADZONE ? 0 : Math.round(v * 1000) / 1000);
const fmt = (v) => `${v >= 0 ? '+' : ''}${v.toFixed(2)}`;

// Set by the stop button, cleared by the next fresh touch on any stick.
let halted = false;

function setAxes(stick, x, y) {
  stick.x = round(x);
  stick.y = round(y);
  stick.readout.value = [
    stick.lock !== 'y' && `x ${fmt(stick.x)}`,
    stick.lock !== 'x' && `y ${fmt(stick.y)}`,
  ].filter(Boolean).join('   ');
}

function sendStick({ id, x, y }) {
  send({ type: 'joystick', id, x, y });
}

function release(stick) {
  const wasActive = stick.active;
  stick.active = false;
  setAxes(stick, 0, 0);
  if (wasActive) sendStick(stick);
  if (stick.remountPending) mount(stick);
}

function mount(stick) {
  // Never rebuild under the user's thumb, wait for release instead.
  if (stick.active) {
    stick.remountPending = true;
    return;
  }
  stick.remountPending = false;

  const { width, height } = stick.zone.getBoundingClientRect();
  const size = Math.round(Math.min(width, height, 320) * 0.75);
  if (size <= 0 || size === stick.size) return;

  stick.manager?.destroy();
  stick.size = size;
  stick.manager = create({
    zone: stick.zone,
    mode: 'static',
    position: { left: '50%', top: '50%' },
    size,
    color: STICK_COLOR,
    restOpacity: 0.6,
    lockX: stick.lock === 'x',
    lockY: stick.lock === 'y',
  });

  // nipplejs 1.x passes a single event object, payload is in evt.data
  stick.manager.on('start', () => {
    stick.active = true;
    halted = false;
  });
  stick.manager.on('move', ({ data }) => setAxes(stick, data.vector.x, data.vector.y));
  stick.manager.on('end', () => release(stick));
}

const sticks = [...document.querySelectorAll('[data-stick]')].map((section) => {
  const stick = {
    id: section.dataset.stick,
    lock: section.dataset.lock, // 'x' | 'y' | undefined
    zone: section.querySelector('.ctrl-zone'),
    readout: section.querySelector('.ctrl-readout'),
    x: 0,
    y: 0,
    active: false,
    remountPending: false,
    size: 0,
    manager: null,
  };
  setAxes(stick, 0, 0);
  new ResizeObserver(() => mount(stick)).observe(stick.zone);
  return stick;
});

setInterval(() => {
  if (halted) return;
  sticks.filter((s) => s.active).forEach(sendStick);
}, SEND_INTERVAL_MS);

// ---------- Safety ----------

stopButton.addEventListener('click', () => {
  halted = true;
  sticks.forEach((s) => setAxes(s, 0, 0));
  send({ type: 'stop' });
});

// Zero everything when the page loses focus, a stuck stick must not keep driving.
document.addEventListener('visibilitychange', () => {
  if (document.hidden) sticks.forEach(release);
});
window.addEventListener('blur', () => sticks.forEach(release));
window.addEventListener('pagehide', () => sticks.forEach(release));

// Suppress long-press menus on Android
document.addEventListener('contextmenu', (e) => e.preventDefault());

// ---------- Fullscreen ----------

if (document.fullscreenEnabled) {
  fullscreenButton.hidden = false;
  fullscreenButton.addEventListener('click', async () => {
    if (document.fullscreenElement) {
      await document.exitFullscreen();
      return;
    }
    await document.documentElement.requestFullscreen();
    try {
      await screen.orientation.lock('landscape');
    } catch {
      // Not supported (desktop, iOS), layout handles both orientations anyway
    }
  });
}

connect();
