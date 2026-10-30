let ws = null;
let entries = {};

export function connectWebSocket({ onHello, onRobotState, onError } = {}) {
  if (ws && (ws.readyState === WebSocket.OPEN || ws.readyState === WebSocket.CONNECTING)) {
    return ws;
  }

  entries = {};

  const proto = window.location.protocol === "https:" ? "wss" : "ws";
  ws = new WebSocket(`${proto}://${window.location.host}/ws`);

  ws.onopen = () => {
    console.log("websocket: connected");
  };

  ws.onclose = () => {
    console.log("websocket: disconnected");
  };

  ws.onerror = (event) => {
    console.log("websocket: error", event);
    if (onError) onError(event);
  };

  ws.onmessage = (event) => {
    let msg;
    try {
      msg = JSON.parse(event.data);
    } catch {
      return;
    }

    if (msg.type === "hello") {
      onHello?.(msg.payload);
      return;
    }

    if (msg.type === "robot_state") {
      onRobotState?.(msg.payload);
      return;
    }

    // Only changed entries are sent. Age uses the server's t, so a clock
    // difference between robot and browser doesn't matter.
    if (msg.type === "robot_state_delta") {
      const { t, stale_sec, entries: changed } = msg.payload;

      for (const [alias, entry] of Object.entries(changed)) {
        entries[alias] = { ...entry, present: true };
      }

      for (const entry of Object.values(entries)) {
        entry.age = t - entry.t;
        entry.stale = entry.age > stale_sec;
      }

      onRobotState?.({ t, stale_sec, entries });
      return;
    }
  };

  return ws;
}