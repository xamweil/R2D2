export async function startFollowTrack(trackId) {
  let response;

  try {
    response = await fetch("/action/start", {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
      },
      body: JSON.stringify({
        alias: "follow_track",
        goal: {
          track_id: Number(trackId),
        },
      }),
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

  if (!response.ok) {
    return {
      ok: false,
      error: data?.detail ?? "Failed to start follow action",
    };
  }

  return {
    ok: true,
    data,
  };
}

export async function cancelFollowTrack() {
  let response;

  try {
    response = await fetch("/action/cancel", {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
      },
      body: JSON.stringify({
        alias: "follow_track",
      }),
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

  if (!response.ok) {
    return {
      ok: false,
      error: data?.detail ?? "Failed to cancel follow action",
    };
  }

  return {
    ok: true,
    data,
  };
}