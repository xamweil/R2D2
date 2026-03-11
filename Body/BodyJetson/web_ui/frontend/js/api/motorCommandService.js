export async function publishMotorCommand(message) {
  const body = {
    alias: "motor_command",
    msg: message,
  };

  let response;
  try {
    response = await fetch("/publish/allowed", {
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