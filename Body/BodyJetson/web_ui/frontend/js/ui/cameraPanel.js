export function initCameraPanel() {
    const cameraImage = document.getElementById('camera-image');
    const cameraStatus = document.getElementById('camera-status');

    console.log("cameraPanel: init start");
    console.log("cameraImage:", cameraImage);
    console.log("cameraStatus:", cameraStatus);

    if (!cameraImage || !cameraStatus) {
        console.log("cameraPanel: required elements not found");
        return;
    }

    cameraStatus.textContent = "Connecting to camera stream...";
    cameraStatus.style.display = "flex";
    cameraImage.style.display = "none";

    console.log("cameraPanel: setting image src to /mjpeg");

    cameraImage.onload = () => {
        console.log("cameraPanel: stream image loaded");
        cameraImage.style.display = "block";
        cameraStatus.style.display = "none";
    };

    cameraImage.onerror = () => {
        console.log("cameraPanel: failed to load stream", event);
        cameraImage.style.display = "none";
        cameraStatus.style.display = "flex";
        cameraStatus.textContent = "Failed to load camera feed";
    };

    cameraImage.src = "/mjpeg";
}