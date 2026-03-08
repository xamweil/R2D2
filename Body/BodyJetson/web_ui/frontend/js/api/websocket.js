let ws = null; 

export function connectWebSocket({ onHello, onRobotState, onError } = {}) {
  if (ws && (ws.readyState === WebSocket.OPEN || ws.readyState === WebSocket.CONNECTING)) {
    return ws;
  }

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
  };

  return ws;
}