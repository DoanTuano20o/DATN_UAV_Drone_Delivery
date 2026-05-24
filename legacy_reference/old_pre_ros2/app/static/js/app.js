const socket = io();

async function loadHealth() {
  const res = await fetch("/health");
  const data = await res.json();
  document.getElementById("server-status").textContent =
    `${data.service}: ${data.status} | FC: ${data.fc_connected ? "connected" : "waiting"}`;
}

async function loadStations() {
  const res = await fetch("/api/gps-stations");
  const stations = await res.json();

  const ul = document.getElementById("station-list");
  ul.innerHTML = "";

  stations.forEach((s) => {
    const li = document.createElement("li");
    li.textContent = `${s.name} (${s.lat}, ${s.lon})`;
    ul.appendChild(li);
  });
}

async function loadMarkers() {
  const res = await fetch("/api/aruco-markers");
  const markers = await res.json();

  const ul = document.getElementById("marker-list");
  ul.innerHTML = "";

  markers.forEach((m) => {
    const li = document.createElement("li");
    li.textContent = `ID ${m.id} | ${m.status} | (${m.lat}, ${m.lon})`;
    ul.appendChild(li);
  });

  if (window.updateArucoMarkersOnMap) {
    window.updateArucoMarkersOnMap(markers);
  }
}

function updateTelemetryView(data) {
  document.getElementById("mode").textContent = data.mode ?? "N/A";
  document.getElementById("armed").textContent = String(data.armed);
  document.getElementById("battery").textContent = data.battery_v != null ? `${data.battery_v} V` : "N/A";
  document.getElementById("gps-fix").textContent = data.gps_fix ?? "N/A";
  document.getElementById("sats").textContent = data.satellites ?? "N/A";
  document.getElementById("alt").textContent = data.rel_alt_m != null ? `${data.rel_alt_m} m` : "N/A";
  document.getElementById("speed").textContent = data.speed_mps != null ? `${data.speed_mps} m/s` : "N/A";
  document.getElementById("latlon").textContent =
    (data.lat != null && data.lon != null) ? `${data.lat}, ${data.lon}` : "N/A";

  if (window.updateDronePositionOnMap) {
    window.updateDronePositionOnMap(data);
  }
}

async function loadInitialTelemetry() {
  const res = await fetch("/api/telemetry");
  const data = await res.json();
  updateTelemetryView(data);
}

socket.on("connect", () => {
  console.log("Socket connected");
});

socket.on("telemetry", (data) => {
  updateTelemetryView(data);
});

socket.on("aruco_markers", (markers) => {
  const ul = document.getElementById("marker-list");
  ul.innerHTML = "";

  markers.forEach((m) => {
    const li = document.createElement("li");
    li.textContent = `ID ${m.id} | ${m.status} | (${m.lat}, ${m.lon})`;
    ul.appendChild(li);
  });

  if (window.updateArucoMarkersOnMap) {
    window.updateArucoMarkersOnMap(markers);
  }
});

window.addEventListener("load", async () => {
  await loadHealth();
  await loadStations();
  await loadMarkers();
  await loadInitialTelemetry();

  setInterval(loadHealth, 3000);
});
