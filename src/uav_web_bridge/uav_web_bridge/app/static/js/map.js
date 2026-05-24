(() => {
  const ui = window.UAVUI || {};
  const fetchJson = ui.fetchJson || (async (url) => (await fetch(url, { cache: "no-store" })).json());

  let map;
  let stationMarkersLayer;
  let arucoMarkersLayer;
  let missionGoalLayer;
  let droneTrackLayer;
  let droneIcon;
  let missionGoalIcon;
  let droneMarker = null;
  let droneTrack = null;
  let missionGoalMarker = null;
  let currentMissionGoal = null;
  let latestDroneData = null;
  let lastRenderedDrone = null;
  let lastDronePopupHtml = "";
  let lastArucoSignature = "";
  let goalRequestPending = false;
  let firstDroneFix = true;
  let renderScheduled = false;
  let renderTimer = null;
  let lastMapRenderAt = 0;
  let toastTimer = null;

  const droneTrackPoints = [];
  const DEFAULT_CENTER = [10.8509, 106.7723];
  const DEFAULT_ZOOM = 18;
  const DEFAULT_SEARCH_ALT_M = 7.0;
  const DEFAULT_SMALL_ALT_M = 3.0;
  const MAX_TRACK_POINTS = 250;
  const MAP_RENDER_MS = 300;
  const MIN_MOVE_DEG = 0.00001;

  function isFiniteNumber(value) {
    return typeof value === "number" && Number.isFinite(value);
  }

  function showToast(message, level = "info") {
    let toast = document.getElementById("map-toast");
    if (!toast) {
      toast = document.createElement("div");
      toast.id = "map-toast";
      toast.className = "map-toast";
      const mapEl = document.getElementById("map");
      if (mapEl?.parentElement) mapEl.parentElement.appendChild(toast);
      else document.body.appendChild(toast);
    }

    toast.textContent = message;
    toast.dataset.level = level;
    toast.classList.add("is-visible");

    if (toastTimer) window.clearTimeout(toastTimer);
    toastTimer = window.setTimeout(() => toast.classList.remove("is-visible"), 2600);
  }

  function renderWaypointList() {
    const ul = document.getElementById("waypoint-list");
    if (!ul) return;

    ul.innerHTML = "";
    const li = document.createElement("li");

    if (!currentMissionGoal) {
      li.textContent = "No mission goal selected";
    } else {
      li.textContent =
        `Goal: ${currentMissionGoal.lat.toFixed(6)}, ${currentMissionGoal.lon.toFixed(6)} ` +
        `| search_alt=${currentMissionGoal.search_alt_m}m | small_alt=${currentMissionGoal.small_alt_m}m`;
    }

    ul.appendChild(li);
  }

  function renderStationList(stations) {
    const ul = document.getElementById("station-list");
    if (!ul) return;

    ul.innerHTML = "";

    if (!stations || stations.length === 0) {
      const li = document.createElement("li");
      li.textContent = "No GPS stations";
      ul.appendChild(li);
      return;
    }

    stations.forEach((station) => {
      const li = document.createElement("li");
      li.textContent = `${station.name}: ${Number(station.lat).toFixed(6)}, ${Number(station.lon).toFixed(6)}`;
      ul.appendChild(li);
    });
  }

  function drawMissionGoalMarker(goal) {
    if (!map || !goal || !isFiniteNumber(goal.lat) || !isFiniteNumber(goal.lon)) return;

    const normalized = {
      lat: goal.lat,
      lon: goal.lon,
      search_alt_m: Number(goal.search_alt_m ?? DEFAULT_SEARCH_ALT_M),
      small_alt_m: Number(goal.small_alt_m ?? DEFAULT_SMALL_ALT_M),
    };

    const sameGoal =
      currentMissionGoal &&
      Math.abs(currentMissionGoal.lat - normalized.lat) < 1e-9 &&
      Math.abs(currentMissionGoal.lon - normalized.lon) < 1e-9 &&
      currentMissionGoal.search_alt_m === normalized.search_alt_m &&
      currentMissionGoal.small_alt_m === normalized.small_alt_m;

    currentMissionGoal = normalized;

    const popupHtml =
      `<b>Mission Goal</b><br>` +
      `Lat: ${normalized.lat.toFixed(6)}<br>` +
      `Lon: ${normalized.lon.toFixed(6)}<br>` +
      `Search Alt: ${normalized.search_alt_m} m<br>` +
      `Small Alt: ${normalized.small_alt_m} m`;

    if (!missionGoalMarker) {
      missionGoalMarker = L.marker([normalized.lat, normalized.lon], { icon: missionGoalIcon })
        .addTo(missionGoalLayer)
        .bindPopup(popupHtml);
    } else if (!sameGoal) {
      missionGoalMarker.setLatLng([normalized.lat, normalized.lon]);
      missionGoalMarker.setPopupContent(popupHtml);
    }

    renderWaypointList();
  }

  async function sendMissionGoal(lat, lon, searchAlt = DEFAULT_SEARCH_ALT_M, smallAlt = DEFAULT_SMALL_ALT_M) {
    const payload = {
      lat,
      lon,
      search_alt_m: searchAlt,
      small_alt_m: smallAlt,
    };

    const res = await fetch("/api/mission/goal", {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
      },
      body: JSON.stringify(payload),
    });

    const data = await res.json();
    if (!res.ok || !data.ok) {
      throw new Error(data.error || "Failed to set mission goal");
    }

    drawMissionGoalMarker(payload);
    return data;
  }

  function clearGoal() {
    if (missionGoalMarker) {
      missionGoalLayer.removeLayer(missionGoalMarker);
      missionGoalMarker = null;
    }

    currentMissionGoal = null;
    renderWaypointList();
    showToast("Mission goal cleared on UI", "info");
  }

  function updateMissionGoalOnMap(goal) {
    if (!goal || !isFiniteNumber(goal.lat) || !isFiniteNumber(goal.lon)) return;
    drawMissionGoalMarker(goal);
  }

  async function loadStationsOnMap() {
    if (!map) return;

    const stations = await fetchJson("/api/gps-stations");
    stationMarkersLayer.clearLayers();
    renderStationList(stations);

    if (stations.length > 0) {
      map.setView([stations[0].lat, stations[0].lon], DEFAULT_ZOOM);
    }

    stations.forEach((station) => {
      L.marker([station.lat, station.lon])
        .addTo(stationMarkersLayer)
        .bindPopup(`<b>${station.name}</b><br>${station.lat}, ${station.lon}`);
    });
  }

  function markerSignature(markers) {
    if (!Array.isArray(markers)) return "";
    return markers
      .map((m) => {
        const lat = isFiniteNumber(m.lat) ? Number(m.lat).toFixed(6) : "";
        const lon = isFiniteNumber(m.lon) ? Number(m.lon).toFixed(6) : "";
        return `${m.marker_id ?? m.id ?? "N/A"}:${lat}:${lon}:${m.status ?? ""}`;
      })
      .sort()
      .join("|");
  }

  function updateArucoMarkersOnMap(markers) {
    if (!arucoMarkersLayer) return;

    const signature = markerSignature(markers);
    if (signature === lastArucoSignature) return;
    lastArucoSignature = signature;

    arucoMarkersLayer.clearLayers();
    if (!Array.isArray(markers)) return;

    markers.forEach((markerData) => {
      if (!isFiniteNumber(markerData.lat) || !isFiniteNumber(markerData.lon)) return;

      const markerId = markerData.marker_id ?? markerData.id ?? "N/A";
      const status = markerData.status ?? "DETECTED";

      const marker = L.circleMarker([markerData.lat, markerData.lon], {
        radius: 8,
        weight: 2,
        color: "#22d3ee",
        fillColor: "#22d3ee",
        fillOpacity: 0.22,
        className: "aruco-glow-marker",
      }).addTo(arucoMarkersLayer);

      marker.bindPopup(`ArUco ID ${markerId}<br>Status: ${status}`);
    });
  }

  function shouldRenderDrone(latlng) {
    if (!lastRenderedDrone) return true;
    return (
      Math.abs(lastRenderedDrone[0] - latlng[0]) > MIN_MOVE_DEG ||
      Math.abs(lastRenderedDrone[1] - latlng[1]) > MIN_MOVE_DEG
    );
  }

  function shouldAppendTrackPoint(latlng) {
    const lastPoint = droneTrackPoints.length > 0 ? droneTrackPoints[droneTrackPoints.length - 1] : null;
    return (
      !lastPoint ||
      Math.abs(lastPoint[0] - latlng[0]) > MIN_MOVE_DEG ||
      Math.abs(lastPoint[1] - latlng[1]) > MIN_MOVE_DEG
    );
  }

  function renderDroneNow() {
    renderScheduled = false;
    renderTimer = null;
    if (document.hidden) return;

    if (!map || !latestDroneData || !isFiniteNumber(latestDroneData.lat) || !isFiniteNumber(latestDroneData.lon)) return;

    const now = performance.now();
    if (now - lastMapRenderAt < MAP_RENDER_MS) {
      scheduleDroneRender(MAP_RENDER_MS - (now - lastMapRenderAt));
      return;
    }

    const latlng = [latestDroneData.lat, latestDroneData.lon];
    if (!shouldRenderDrone(latlng)) return;
    lastMapRenderAt = now;
    lastRenderedDrone = latlng;

    if (!droneMarker) {
      droneMarker = L.marker(latlng, { icon: droneIcon }).addTo(droneTrackLayer);
    } else {
      droneMarker.setLatLng(latlng);
    }

    const popupHtml =
      `<b>Drone</b><br>` +
      `Mode: ${latestDroneData.mode ?? "N/A"}<br>` +
      `Armed: ${String(latestDroneData.armed)}<br>` +
      `Alt: ${latestDroneData.rel_alt_m ?? "N/A"} m<br>` +
      `Sats: ${latestDroneData.satellites ?? "N/A"}<br>` +
      `Lat/Lon: ${latestDroneData.lat}, ${latestDroneData.lon}`;

    if (popupHtml !== lastDronePopupHtml) {
      if (droneMarker.getPopup()) droneMarker.setPopupContent(popupHtml);
      else droneMarker.bindPopup(popupHtml);
      lastDronePopupHtml = popupHtml;
    }

    if (shouldAppendTrackPoint(latlng)) {
      droneTrackPoints.push(latlng);
    }

    while (droneTrackPoints.length > MAX_TRACK_POINTS) {
      droneTrackPoints.shift();
    }

    if (!droneTrack) {
      droneTrack = L.polyline(droneTrackPoints, {
        color: "#22d3ee",
        weight: 3,
        opacity: 0.92,
        lineCap: "round",
        lineJoin: "round",
        className: "drone-track-line",
      }).addTo(droneTrackLayer);
    } else {
      droneTrack.setLatLngs(droneTrackPoints);
    }

    if (firstDroneFix) {
      map.setView(latlng, DEFAULT_ZOOM);
      firstDroneFix = false;
    }
  }

  function scheduleDroneRender(delayMs = 0) {
    if (document.hidden) return;
    if (renderScheduled) return;
    renderScheduled = true;

    const run = () => requestAnimationFrame(renderDroneNow);
    if (delayMs > 0) {
      renderTimer = window.setTimeout(run, delayMs);
    } else {
      run();
    }
  }

  function updateDronePositionOnMap(data) {
    if (!map || !data || !isFiniteNumber(data.lat) || !isFiniteNumber(data.lon)) return;
    latestDroneData = data;
    scheduleDroneRender();
  }

  function debounce(fn, waitMs) {
    let timer = null;
    return (...args) => {
      if (timer) window.clearTimeout(timer);
      timer = window.setTimeout(() => fn(...args), waitMs);
    };
  }

  function initMap() {
    const mapEl = document.getElementById("map");
    if (!mapEl || typeof L === "undefined") return;

    droneIcon = L.icon({
      iconUrl: "/static/img/drone.png",
      iconSize: [42, 42],
      iconAnchor: [21, 21],
      popupAnchor: [0, -22],
      className: "drone-live-icon",
    });

    missionGoalIcon = L.divIcon({
      className: "",
      html: '<span class="mission-goal-icon"></span>',
      iconSize: [24, 24],
      iconAnchor: [12, 12],
    });

    map = L.map(mapEl, {
      center: DEFAULT_CENTER,
      zoom: DEFAULT_ZOOM,
      zoomControl: true,
    });

    const streetLayer = L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
      maxZoom: 22,
      attribution: "&copy; OpenStreetMap contributors",
    });

    const darkLayer = L.tileLayer("https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png", {
      maxZoom: 22,
      attribution: "&copy; OpenStreetMap &copy; CARTO",
    });

    const satelliteLayer = L.tileLayer(
      "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
      {
        maxZoom: 22,
        attribution: "Tiles &copy; Esri",
      }
    );

    darkLayer.addTo(map);

    stationMarkersLayer = L.layerGroup().addTo(map);
    arucoMarkersLayer = L.layerGroup().addTo(map);
    missionGoalLayer = L.layerGroup().addTo(map);
    droneTrackLayer = L.layerGroup().addTo(map);

    L.control.layers(
      {
        Dark: darkLayer,
        Street: streetLayer,
        Satellite: satelliteLayer,
      },
      null,
      { collapsed: false }
    ).addTo(map);

    satelliteLayer.on("tileerror", () => {
      console.warn("Satellite tile load failed, fallback to dark map");
      if (map.hasLayer(satelliteLayer)) map.removeLayer(satelliteLayer);
      if (!map.hasLayer(darkLayer)) darkLayer.addTo(map);
    });

    map.on("click", async (event) => {
      if (goalRequestPending) {
        showToast("Goal request is still pending", "warn");
        return;
      }

      goalRequestPending = true;
      showToast("Sending mission goal...", "info");

      try {
        const data = await sendMissionGoal(
          event.latlng.lat,
          event.latlng.lng,
          DEFAULT_SEARCH_ALT_M,
          DEFAULT_SMALL_ALT_M
        );

        if (missionGoalMarker) missionGoalMarker.openPopup();
        showToast("Mission goal sent", "ok");
        console.log("Mission goal sent:", data);
      } catch (err) {
        console.error("Failed to send mission goal:", err);
        showToast(`Goal failed: ${err.message || "server error"}`, "error");
      } finally {
        goalRequestPending = false;
      }
    });

    const clearBtn = document.getElementById("clear-waypoints-btn");
    if (clearBtn) clearBtn.addEventListener("click", clearGoal);

    window.addEventListener(
      "resize",
      debounce(() => {
        if (map) map.invalidateSize();
      }, 250)
    );

    document.addEventListener("visibilitychange", () => {
      if (!document.hidden) {
        if (renderTimer) {
          window.clearTimeout(renderTimer);
          renderTimer = null;
          renderScheduled = false;
        }
        if (latestDroneData) scheduleDroneRender();
        window.setTimeout(() => {
          if (map) map.invalidateSize();
        }, 120);
      }
    });

    renderWaypointList();
    loadStationsOnMap().catch((err) => console.error("[WEB] station load error:", err));
  }

  document.addEventListener("DOMContentLoaded", initMap);

  window.updateArucoMarkersOnMap = updateArucoMarkersOnMap;
  window.updateDronePositionOnMap = updateDronePositionOnMap;
  window.updateMissionGoalOnMap = updateMissionGoalOnMap;
})();
