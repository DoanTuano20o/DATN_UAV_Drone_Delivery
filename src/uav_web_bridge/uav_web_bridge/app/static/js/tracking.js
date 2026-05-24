(() => {
  const ui = window.UAVUI || {};
  const setText = ui.setText || ((el, value) => { if (el) el.textContent = String(value); });
  const fetchJson = ui.fetchJson || (async (url) => (await fetch(url, { cache: "no-store" })).json());

  let map;
  let droneIcon;
  let destinationIcon;
  let droneMarker;
  let destinationMarker;
  let routeLine;
  let socket = null;
  let socketConnected = false;
  let latestTelemetry = null;
  let latestMission = null;
  let destination = null;
  let firstFix = true;
  let renderScheduled = false;
  let renderTimer = null;
  let lastRenderAt = 0;
  let lastTelemetryAt = 0;
  let lastMissionAt = 0;
  let lastPollAt = 0;
  let lastStatusKey = "";
  let lastEtaText = "";
  let lastDestinationKey = "";
  let lastDronePoint = null;

  const dronePath = [];
  const DEFAULT_CENTER = [10.8509, 106.7723];
  const DEFAULT_ZOOM = 17;
  const MAX_PATH_POINTS = 180;
  const MAP_RENDER_MS = 400;
  const DATA_STALE_MS = 6000;
  const POLL_VISIBLE_MS = 5000;
  const POLL_HIDDEN_MS = 15000;
  const MIN_MOVE_DEG = 0.00001;

  function byId(id) {
    return document.getElementById(id);
  }

  function set(id, value) {
    setText(byId(id), value);
  }

  function setBodyState(name, value) {
    if (document.body) document.body.dataset[name] = value;
  }

  function isFiniteNumber(value) {
    return typeof value === "number" && Number.isFinite(value);
  }

  function numberFrom(data, ...keys) {
    for (const key of keys) {
      const value = Number(data?.[key]);
      if (Number.isFinite(value)) return value;
    }
    return null;
  }

  function valueFrom(data, ...keys) {
    for (const key of keys) {
      const value = data?.[key];
      if (value !== null && value !== undefined && value !== "") return value;
    }
    return null;
  }

  function formatNumber(value, digits = 1, suffix = "") {
    const number = Number(value);
    if (!Number.isFinite(number)) return "—";
    return `${number.toFixed(digits)}${suffix}`;
  }

  function haversineMeters(a, b) {
    const toRad = (deg) => (deg * Math.PI) / 180;
    const r = 6371000;
    const dLat = toRad(b[0] - a[0]);
    const dLon = toRad(b[1] - a[1]);
    const lat1 = toRad(a[0]);
    const lat2 = toRad(b[0]);
    const x =
      Math.sin(dLat / 2) * Math.sin(dLat / 2) +
      Math.cos(lat1) * Math.cos(lat2) * Math.sin(dLon / 2) * Math.sin(dLon / 2);
    return 2 * r * Math.atan2(Math.sqrt(x), Math.sqrt(1 - x));
  }

  function classifyDelivery(mission, telemetry) {
    const state = String(mission?.state || "").toUpperCase();
    const connected = !!telemetry?.connected;

    if (state.includes("DONE")) {
      return {
        step: "delivered",
        title: "Giao hàng hoàn tất",
        delivery: "Đã giao",
        drone: "Hoàn tất nhiệm vụ",
        package: "Hoàn tất",
        description: "Đơn hàng đã được giao thành công.",
      };
    }

    if (state.includes("LAND") || state.includes("DROP") || state.includes("LOCK_SMALL")) {
      return {
        step: "near",
        title: "Drone đang tiếp cận điểm giao",
        delivery: "Gần điểm giao",
        drone: "Đang hạ cánh",
        package: "Sắp giao",
        description: "Drone đang ở gần khu vực giao hàng và chuẩn bị hạ cánh.",
      };
    }

    if (state.includes("SEARCH") || state.includes("ALIGN") || state.includes("LOCK")) {
      return {
        step: "near",
        title: "Drone đang căn chỉnh điểm giao",
        delivery: "Gần điểm giao",
        drone: "Đang tìm marker",
        package: "Đang tiếp cận",
        description: "Drone đang tìm marker ArUco và căn chỉnh để hạ cánh chính xác.",
      };
    }

    if (state.includes("GOTO") || state.includes("FLY") || state.includes("ROUTE")) {
      return {
        step: "route",
        title: "Drone đang trên đường giao hàng",
        delivery: "Đang giao hàng",
        drone: "Đang bay",
        package: "Đang vận chuyển",
        description: "Drone đang di chuyển đến khu vực giao hàng đã được thiết lập.",
      };
    }

    if (state.includes("TAKEOFF") || state.includes("HOLD")) {
      return {
        step: "takeoff",
        title: "Drone đã bắt đầu nhiệm vụ",
        delivery: "Đã cất cánh",
        drone: "Đang khởi hành",
        package: "Đã nhận",
        description: "Drone đã cất cánh và đang bắt đầu hành trình giao hàng.",
      };
    }

    if (connected) {
      return {
        step: "preparing",
        title: "Đơn hàng đang được chuẩn bị",
        delivery: "Đang chuẩn bị",
        drone: "Online",
        package: "Đang chuẩn bị",
        description: "Hệ thống đang chuẩn bị nhiệm vụ giao hàng và chờ dữ liệu hành trình.",
      };
    }

    return {
      step: "preparing",
      title: "Đơn hàng đang chờ dữ liệu",
      delivery: "Mất kết nối",
      drone: "Chưa có dữ liệu",
      package: "Đang chuẩn bị",
      description: "Đang chờ dữ liệu giao hàng realtime từ hệ thống UAV.",
    };
  }

  function updateTimeline(activeStep) {
    const order = ["received", "preparing", "takeoff", "route", "near", "delivered"];
    const activeIndex = Math.max(1, order.indexOf(activeStep));

    document.querySelectorAll("#delivery-timeline li").forEach((item) => {
      const idx = order.indexOf(item.dataset.step);
      item.classList.toggle("done", idx < activeIndex);
      item.classList.toggle("active", idx === activeIndex);
    });
  }

  function updateCustomerStatus() {
    const status = classifyDelivery(latestMission, latestTelemetry);
    const key = Object.values(status).join("|");
    if (key === lastStatusKey) return;
    lastStatusKey = key;

    set("tracking-title", status.title);
    set("delivery-status", status.delivery);
    set("customer-drone-status", status.drone);
    set("package-status", status.package);
    set("customer-status-text", status.description);
    setBodyState("delivery", status.step);
    updateTimeline(status.step);
  }

  function updateEta() {
    let eta = "Đang chờ tín hiệu";
    let distanceText = "—";

    if (latestTelemetry && destination && isFiniteNumber(latestTelemetry.lat) && isFiniteNumber(latestTelemetry.lon)) {
      const current = [latestTelemetry.lat, latestTelemetry.lon];
      const target = [destination.lat, destination.lon];
      const distance = haversineMeters(current, target);
      const speed = Number(latestTelemetry.speed_mps || 0);

      distanceText = distance >= 1000 ? `${(distance / 1000).toFixed(2)} km` : `${Math.round(distance)} m`;
      if (distance < 4) eta = "Sắp đến";
      else if (speed > 0.5) eta = `${Math.max(1, Math.ceil(distance / speed / 60))} phút`;
      else eta = `${Math.round(distance)} m còn lại`;
    } else if (latestTelemetry || destination) {
      eta = "Đang tính...";
    }

    if (eta !== lastEtaText) {
      lastEtaText = eta;
      set("eta", eta);
    }
    set("tracking-distance", distanceText);
  }

  function updateDestination(goal) {
    if (!map || !goal || !isFiniteNumber(goal.lat) || !isFiniteNumber(goal.lon)) return;

    const key = `${goal.lat.toFixed(7)},${goal.lon.toFixed(7)}`;
    destination = { lat: goal.lat, lon: goal.lon };
    if (key === lastDestinationKey) return;
    lastDestinationKey = key;

    const latlng = [destination.lat, destination.lon];
    if (!destinationMarker) {
      destinationMarker = L.marker(latlng, { icon: destinationIcon }).addTo(map);
      destinationMarker.bindPopup("<b>Điểm giao hàng</b>");
    } else {
      destinationMarker.setLatLng(latlng);
    }

    updateEta();
  }

  function shouldAppendPoint(latlng) {
    const lastPoint = dronePath.length ? dronePath[dronePath.length - 1] : null;
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

    if (!map || !latestTelemetry || !isFiniteNumber(latestTelemetry.lat) || !isFiniteNumber(latestTelemetry.lon)) return;

    const now = performance.now();
    if (now - lastRenderAt < MAP_RENDER_MS) {
      scheduleMapRender(MAP_RENDER_MS - (now - lastRenderAt));
      return;
    }
    lastRenderAt = now;

    const latlng = [latestTelemetry.lat, latestTelemetry.lon];
    if (
      lastDronePoint &&
      Math.abs(lastDronePoint[0] - latlng[0]) <= MIN_MOVE_DEG &&
      Math.abs(lastDronePoint[1] - latlng[1]) <= MIN_MOVE_DEG
    ) {
      return;
    }
    lastDronePoint = latlng;

    if (!droneMarker) {
      droneMarker = L.marker(latlng, { icon: droneIcon }).addTo(map).bindPopup("<b>Drone giao hàng</b>");
    } else {
      droneMarker.setLatLng(latlng);
    }

    if (shouldAppendPoint(latlng)) dronePath.push(latlng);
    while (dronePath.length > MAX_PATH_POINTS) dronePath.shift();

    if (!routeLine) {
      routeLine = L.polyline(dronePath, {
        color: "#34d399",
        weight: 3,
        opacity: 0.9,
        lineCap: "round",
        lineJoin: "round",
        className: "tracking-route-line",
      }).addTo(map);
    } else {
      routeLine.setLatLngs(dronePath);
    }

    if (firstFix) {
      map.setView(latlng, DEFAULT_ZOOM);
      firstFix = false;
    }
  }

  function scheduleMapRender(delayMs = 0) {
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

  function acceptTelemetry(data, source = "socket") {
    latestTelemetry = data || {};
    lastTelemetryAt = Date.now();
    if (source === "socket") setConnectionStatus("live", "Live");
    else if (!socketConnected) setConnectionStatus("waiting", "Đang cập nhật");
    scheduleMapRender();
    updateRealtimeDetails();
    updateCustomerStatus();
    updateEta();

    if (source === "poll" && socketConnected) {
      console.warn("[WEB] tracking fallback poll used while socket connected");
    }
  }

  function acceptMission(data) {
    latestMission = data || {};
    lastMissionAt = Date.now();
    if (latestMission.goal) updateDestination(latestMission.goal);
    updateRealtimeDetails();
    updateCustomerStatus();
    updateEta();
  }

  function updateRealtimeDetails() {
    const telemetry = latestTelemetry || {};
    const lastUpdate = numberFrom(telemetry, "last_update_unix", "last_msg");
    const altitude = numberFrom(telemetry, "rel_alt_m", "altitude_m", "alt");
    const speed = numberFrom(telemetry, "speed_mps", "groundspeed_mps", "speed");
    const satellites = valueFrom(telemetry, "satellites", "gps_satellites", "sats");
    const gpsFix = valueFrom(telemetry, "gps_fix", "gps_status");
    const batteryV = numberFrom(telemetry, "battery_v", "battery_voltage");
    const batteryPct = numberFrom(telemetry, "battery_percent", "battery_pct");

    if (lastTelemetryAt) {
      set("last-update", `Cập nhật: ${new Date(lastTelemetryAt).toLocaleTimeString()}`);
    } else if (lastUpdate) {
      set("last-update", `Cập nhật: ${new Date(lastUpdate * 1000).toLocaleTimeString()}`);
    }

    set("tracking-altitude", formatNumber(altitude, 1, " m"));
    set("tracking-speed", formatNumber(speed, 1, " m/s"));
    set("tracking-satellites", satellites ?? "—");

    if (batteryPct !== null && batteryPct <= 100) {
      set("tracking-battery", `${Math.round(batteryPct)}%`);
    } else {
      set("tracking-battery", formatNumber(batteryV, 1, " V"));
    }

    if (gpsFix !== null) {
      set("tracking-gps", String(gpsFix));
    } else if (isFiniteNumber(telemetry.lat) && isFiniteNumber(telemetry.lon)) {
      set("tracking-gps", "Có tọa độ");
    } else {
      set("tracking-gps", "—");
    }
  }

  async function loadDefaultDestination() {
    try {
      const stations = await fetchJson("/api/gps-stations");
      if (stations.length && !destination) {
        updateDestination(stations[0]);
      }
    } catch (err) {
      console.error("[WEB] tracking station error:", err);
    }
  }

  function socketDataFresh() {
    const now = Date.now();
    return (
      socketConnected &&
      now - lastTelemetryAt < DATA_STALE_MS &&
      now - lastMissionAt < DATA_STALE_MS
    );
  }

  async function pollFallback() {
    if (socketDataFresh()) return;

    try {
      acceptTelemetry(await fetchJson("/api/telemetry"), "poll");
    } catch (err) {
      console.error("[WEB] tracking telemetry poll error:", err);
    }

    try {
      acceptMission(await fetchJson("/api/mission/state"));
    } catch (err) {
      console.error("[WEB] tracking mission poll error:", err);
    }
  }

  function tickPolling() {
    const interval = document.hidden ? POLL_HIDDEN_MS : POLL_VISIBLE_MS;
    const now = Date.now();
    updateConnectionFreshness(now);
    if (now - lastPollAt < interval) return;
    lastPollAt = now;
    pollFallback();
  }

  function initSocket() {
    if (socket || typeof io !== "function") {
      if (!socket) setConnectionStatus("unavailable", "Socket không khả dụng");
      return;
    }

    socket = io();

    socket.on("connect", () => {
      socketConnected = true;
      setConnectionStatus("live", "Live");
      lastPollAt = Date.now();
    });

    socket.on("disconnect", () => {
      socketConnected = false;
      setConnectionStatus("reconnecting", "Đang kết nối lại");
      lastPollAt = 0;
    });

    socket.on("telemetry", (data) => acceptTelemetry(data, "socket"));
    socket.on("mission_state", acceptMission);
  }

  function setConnectionStatus(state, label) {
    set("tracking-connection", label);
    setBodyState("connection", state);
  }

  function updateConnectionFreshness(now = Date.now()) {
    if (socketConnected && now - lastTelemetryAt < DATA_STALE_MS) {
      setConnectionStatus("live", "Live");
      return;
    }

    if (socketConnected) {
      setConnectionStatus("waiting", "Đang chờ dữ liệu");
      return;
    }

    if (lastTelemetryAt && now - lastTelemetryAt > DATA_STALE_MS) {
      setConnectionStatus("offline", "Mất kết nối");
    }
  }

  function initTrackingMap() {
    const el = byId("tracking-map");
    if (!el || typeof L === "undefined") return false;

    droneIcon = L.icon({
      iconUrl: "/static/img/drone.png",
      iconSize: [42, 42],
      iconAnchor: [21, 21],
      popupAnchor: [0, -22],
      className: "tracking-drone-icon",
    });

    destinationIcon = L.divIcon({
      className: "",
      html: '<span class="tracking-destination-icon"></span>',
      iconSize: [24, 24],
      iconAnchor: [12, 12],
    });

    map = L.map(el, {
      center: DEFAULT_CENTER,
      zoom: DEFAULT_ZOOM,
    });

    const darkLayer = L.tileLayer("https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png", {
      maxZoom: 22,
      attribution: "&copy; OpenStreetMap &copy; CARTO",
    });

    const streetLayer = L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
      maxZoom: 22,
      attribution: "&copy; OpenStreetMap contributors",
    });

    const satelliteLayer = L.tileLayer(
      "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
      {
        maxZoom: 22,
        attribution: "Tiles &copy; Esri",
      }
    );

    darkLayer.addTo(map);
    L.control.layers({ Dark: darkLayer, Street: streetLayer, Satellite: satelliteLayer }, null, {
      collapsed: false,
    }).addTo(map);

    return true;
  }

  function initTracking() {
    if (!byId("tracking-map")) return;
    if (!initTrackingMap()) return;

    setConnectionStatus("waiting", "Đang kết nối");
    updateRealtimeDetails();
    initSocket();
    loadDefaultDestination();
    lastPollAt = 0;
    tickPolling();
    window.setInterval(tickPolling, 1000);
    document.addEventListener("visibilitychange", () => {
      document.body.classList.toggle("is-hidden-tab", document.hidden);
      if (!document.hidden) {
        if (renderTimer) {
          window.clearTimeout(renderTimer);
          renderTimer = null;
          renderScheduled = false;
        }
        if (latestTelemetry) scheduleMapRender();
        if (map) {
          window.setTimeout(() => map.invalidateSize(), 120);
        }
        tickPolling();
      }
    });

    let resizeTimer = null;
    window.addEventListener(
      "resize",
      () => {
        if (!map) return;
        if (resizeTimer) window.clearTimeout(resizeTimer);
        resizeTimer = window.setTimeout(() => {
          resizeTimer = null;
          map.invalidateSize();
        }, 160);
      },
      { passive: true }
    );
  }

  document.addEventListener("DOMContentLoaded", initTracking);
})();
