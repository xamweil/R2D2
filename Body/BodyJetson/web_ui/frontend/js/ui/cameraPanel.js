function startCameraStream() {
    const cameraImage = document.getElementById("camera-image");
    const cameraStatus = document.getElementById("camera-status");

    console.log("cameraPanel: start stream");
    console.log("cameraImage:", cameraImage);
    console.log("cameraStatus:", cameraStatus);

    if (!cameraImage || !cameraStatus) {
        console.log("cameraPanel: required elements not found");
        return;
    }

    cameraStatus.textContent = "Connecting to camera stream...";
    cameraStatus.style.display = "flex";
    cameraImage.style.display = "none";

    cameraImage.onload = () => {
        console.log("cameraPanel: stream image loaded");
        cameraImage.style.display = "block";
        cameraStatus.style.display = "none";
    };

    cameraImage.onerror = (event) => {
        console.log("cameraPanel: failed to load stream", event);
        cameraImage.style.display = "none";
        cameraStatus.style.display = "flex";
        cameraStatus.textContent = "Failed to load camera feed";
    };

    // Important: one fresh request only when we explicitly start/retry
    cameraImage.src = `/mjpeg?ts=${Date.now()}`;
}

export function initCameraPanel() {
    const retryButton = document.getElementById("camera-retry-button");

    console.log("cameraPanel: init start");

    retryButton?.addEventListener("click", () => {
        console.log("cameraPanel: retry clicked");
        startCameraStream();
    });

    startCameraStream();
}

