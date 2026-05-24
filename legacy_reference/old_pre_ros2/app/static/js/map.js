let map;
let stationMarkersLayer;
let arucoMarkersLayer;
let waypointMarkersLayer;
let waypointPolyline;
let droneTrackLayer;

let droneMarker = null;
let droneTrack = null;
let droneTrackPoints = [];
let firstDroneFix = true;

let selectedWaypoints = [];
let arucoMarkerRefs = {};

const DEFAULT_CENTER = [10.8509, 106.7723];
const DEFAULT_ZOOM = 18;

function renderWaypointList() {
  const ul = document.getElementById("waypoint-list");
  ul.innerHTML = "";

  if (selectedWaypoints.length === 0) {
    const li = document.createElement("li");
    li.textContent = "No waypoint selected";
    ul.appendChild(li);
    return;
  }

  selectedWaypoints.forEach((wp, index) => {
    const li = document.createElement("li");
    li.textContent = `WP${index + 1}: ${wp.lat.toFixed(6)}, ${wp.lon.toFixed(6)}`;
    ul.appendChild(li);
  });
}

function redrawWaypointPolyline() {
  const latlngs = selectedWaypoints.map((wp) => [wp.lat, wp.lon]);

  if (waypointPolyline) {
    waypointPolyline.setLatLngs(latlngs);
  } else {
    waypointPolyline = L.polyline(latlngs, { weight: 3 }).addTo(waypointMarkersLayer);
  }
}

function addWaypoint(lat, lon) {
  const wp = { lat, lon };
  selectedWaypoints.push(wp);

  L.marker([lat, lon])
    .addTo(waypointMarkersLayer)
    .bindPopup(`Waypoint ${selectedWaypoints.length}<br>${lat.toFixed(6)}, ${lon.toFixed(6)}`);

  redrawWaypointPolyline();
  renderWaypointList();
}

function clearWaypoints() {
  selectedWaypoints = [];
  waypointMarkersLayer.clearLayers();
  waypointPolyline = null;
  renderWaypointList();
}

async function loadStationsOnMap() {
  const res = await fetch("/api/gps-stations");
  const stations = await res.json();

  stationMarkersLayer.clearLayers();

  if (stations.length > 0) {
    map.setView([stations[0].lat, stations[0].lon], DEFAULT_ZOOM);
  }

  stations.forEach((s) => {
    L.marker([s.lat, s.lon])
      .addTo(stationMarkersLayer)
      .bindPopup(`<b>${s.name}</b><br>${s.lat}, ${s.lon}`);
  });
}

function updateArucoMarkersOnMap(markers) {
  arucoMarkersLayer.clearLayers();
  arucoMarkerRefs = {};

  markers.forEach((m) => {
    const marker = L.circleMarker([m.lat, m.lon], {
      radius: 8,
      weight: 2,
    }).addTo(arucoMarkersLayer);

    marker.bindPopup(`ArUco ID ${m.id}<br>Status: ${m.status}`);
    arucoMarkerRefs[m.id] = marker;
  });
}

function updateDronePositionOnMap(data) {
  if (typeof data.lat !== "number" || typeof data.lon !== "number") {
    return;
  }

  const latlng = [data.lat, data.lon];

  if (!droneMarker) {
    droneMarker = L.marker(latlng).addTo(droneTrackLayer);
  } else {
    droneMarker.setLatLng(latlng);
  }

  const popupHtml = `
    <b>Drone</b><br>
    Mode: ${data.mode ?? "N/A"}<br>
    Armed: ${String(data.armed)}<br>
    Alt: ${data.rel_alt_m ?? "N/A"} m<br>
    Sats: ${data.satellites ?? "N/A"}<br>
    Lat/Lon: ${data.lat}, ${data.lon}
  `;
  droneMarker.bindPopup(popupHtml);

  droneTrackPoints.push(latlng);
  if (droneTrackPoints.length > 300) {
    droneTrackPoints.shift();
  }

  if (!droneTrack) {
    droneTrack = L.polyline(droneTrackPoints, { weight: 3 }).addTo(droneTrackLayer);
  } else {
    droneTrack.setLatLngs(droneTrackPoints);
  }

  if (firstDroneFix) {
    map.setView(latlng, DEFAULT_ZOOM);
    firstDroneFix = false;
  }
}

function initMap() {
  map = L.map("map").setView(DEFAULT_CENTER, DEFAULT_ZOOM);

  L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
    maxZoom: 22,
    attribution: "&copy; OpenStreetMap contributors",
  }).addTo(map);

  stationMarkersLayer = L.layerGroup().addTo(map);
  arucoMarkersLayer = L.layerGroup().addTo(map);
  waypointMarkersLayer = L.layerGroup().addTo(map);
  droneTrackLayer = L.layerGroup().addTo(map);

  map.on("click", (e) => {
    addWaypoint(e.latlng.lat, e.latlng.lng);
  });

  document.getElementById("clear-waypoints-btn").addEventListener("click", clearWaypoints);

  renderWaypointList();
}

window.addEventListener("load", async () => {
  initMap();
  await loadStationsOnMap();
});

window.updateArucoMarkersOnMap = updateArucoMarkersOnMap;
window.updateDronePositionOnMap = updateDronePositionOnMap;
