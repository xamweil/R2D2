export async function callDeviceCommand(deviceName, methodName, args = []) {
  const body = {
    alias: "device_command",
    request: {
      device_name: deviceName,
      method_name: methodName,
      args: args,
    },
    timeout_sec: 2.0,
  };

  let response;
  try {
    response = await fetch("/call/allowed", {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
      },
      body: JSON.stringify(body),
    });
  } catch (error) {
    return {
      ok: false,
      error: `Network error: ${error instanceof Error ? error.message : String(error)}`,
    };
  }

  let data = {};
  try {
    data = await response.json();
  } catch {
    return {
      ok: false,
      error: "Invalid JSON response from backend",
    };
  }

  return data;
}