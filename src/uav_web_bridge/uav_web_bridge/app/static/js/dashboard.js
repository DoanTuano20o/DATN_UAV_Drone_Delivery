(() => {
  const ui = window.UAVUI || {};
  const fmtNum = ui.fmtNum || ((v, d = 2) => (Number.isFinite(Number(v)) ? Number(v).toFixed(d) : "N/A"));
  const setText = ui.setText || ((el, value) => { if (el) el.textContent = String(value); });
  const fetchJson = ui.fetchJson || (async (url) => (await fetch(url, { cache: "no-store" })).json());

  const refs = {};
  const textCache = new Map();
  const timers = new Set();

  const TELEMETRY_STALE_MS = 4500;
  const MISSION_STALE_MS = 4500;
  const HEALTH_VISIBLE_MS = 8000;
  const HEALTH_HIDDEN_MS = 20000;
  const FALLBACK_VISIBLE_MS = 3000;
  const FALLBACK_HIDDEN_MS = 10000;
  const UI_THROTTLE_MS = 220;
  const MINI_OVERLAY_MS = 300;
  const MAX_LOG_LINES = 200;

  let socket = null;
  let socketConnected = false;
  let activeLogFilter = "all";

  let latestTelemetry = null;
  let latestMission = null;
  let latestMarkers = null;
  let lastTelemetryAt = 0;
  let lastMissionAt = 0;
  let lastHealthPollAt = 0;
  let lastFallbackPollAt = 0;
  let lastTelemetryRenderAt = 0;
  let lastMissionRenderAt = 0;
  let lastMiniOverlayAt = 0;
  let cameraHealthState = "idle";
  let latestHealthConnected = null;

  let telemetryRenderScheduled = false;
  let missionRenderScheduled = false;
  let markerRenderScheduled = false;
  let telemetryRenderTimer = null;
  let missionRenderTimer = null;

  let lastMissionState = null;
  let lastBigMarkerSeen = null;
  let lastSmallMarkerSeen = null;
  let lastBigMarkerKey = null;
  let lastSmallMarkerKey = null;
  let lastTelemetryMode = null;

  const missionProgress = [
    ["IDLE", 0, "Idle"],
    ["WAIT_GUIDED", 5, "Waiting Guided"],
    ["TAKEOFF", 14, "Takeoff"],
    ["HOLD_BEFORE_GOTO", 22, "Hold"],
    ["GOTO_MARKER_REGION", 32, "Fly To Goal"],
    ["SEARCH_BIG_PATTERN", 45, "Search Marker"],
    ["ALIGN_BIG", 55, "Align Big Marker"],
    ["LOCK_BIG", 62, "Lock Big Marker"],
    ["DESCEND_TO_SMALL_ALT", 70, "Descend"],
    ["SEARCH_SMALL", 76, "Search Small Marker"],
    ["ALIGN_SMALL", 82, "Align Small Marker"],
    ["LOCK_SMALL", 88, "Lock Small Marker"],
    ["LAND_ON_SMALL_ARUCO", 92, "Landing"],
    ["DESCEND_TO_DROP_ALT", 92, "Descend To Drop"],
    ["DROP_PAYLOAD", 96, "Drop Payload"],
    ["WAIT_DROP_DONE", 98, "Waiting Drop Done"],
    ["RTL_RETURN", 98, "Return"],
    ["LAND", 98, "Landing"],
    ["DONE", 100, "Done"],
    ["FAILSAFE", 100, "Failsafe"],
    ["MANUAL_OVERRIDE", 100, "Manual Override"],
  ];

  const missionStepOrder = [
    "TAKEOFF",
    "HOLD",
    "GOTO",
    "SEARCH_BIG",
    "ALIGN_BIG",
    "LOCK_BIG",
    "DESCEND",
    "SEARCH_SMALL",
    "ALIGN_SMALL",
    "LAND",
    "DROP",
  ];

  function byId(id) {
    if (!refs[id]) refs[id] = document.getElementById(id);
    return refs[id];
  }

  function smartText(id, value) {
    const next = String(value ?? "N/A");
    if (textCache.get(id) === next) return;
    textCache.set(id, next);
    setText(byId(id), next);
  }

  function setElementState(el, state) {
    if (!el) return;
    if (state) el.dataset.state = state;
    else delete el.dataset.state;
  }

  function setMirrorStatus(key, value, ok = null) {
    const topId = `top-${key}`;
    smartText(topId, value);
    const el = byId(topId)?.closest(".metric-pill");
    const state = ok === null ? "warn" : ok ? "ok" : "bad";
    setElementState(el, state);
  }

  function setMetricState(id, state) {
    const el = byId(id)?.closest(".metric-pill") || byId(id)?.closest(".telemetry-widget");
    setElementState(el, state);
  }

  function nowText() {
    return new Date().toLocaleTimeString();
  }

  function isFiniteNumber(value) {
    return typeof value === "number" && Number.isFinite(value);
  }

  function normalizeLogLevel(level) {
    const raw = String(level || "info").toLowerCase();
    if (["ok", "success", "done"].includes(raw)) return "ok";
    if (["warn", "warning"].includes(raw)) return "warn";
    if (["err", "error", "danger", "fail"].includes(raw)) return "error";
    return "info";
  }

  function applyLogFilter(line) {
    if (!line) return;
    const visible = activeLogFilter === "all" || line.dataset.level === activeLogFilter;
    line.classList.toggle("is-hidden", !visible);
  }

  function appendLog(text, level = "info") {
    const liveLogEl = byId("live-log");
    if (!liveLogEl) return;

    const nearBottom =
      liveLogEl.scrollHeight - liveLogEl.scrollTop - liveLogEl.clientHeight < 28;
    const safeLevel = normalizeLogLevel(level);
    const line = document.createElement("div");
    line.className = "log-line";
    line.dataset.level = safeLevel;

    const timeSpan = document.createElement("span");
    timeSpan.className = "log-time";
    timeSpan.textContent = `[${nowText()}]`;

    const textSpan = document.createElement("span");
    textSpan.className = `log-${safeLevel}`;
    textSpan.textContent = String(text || "Log event");

    line.appendChild(timeSpan);
    line.appendChild(textSpan);
    liveLogEl.appendChild(line);
    applyLogFilter(line);

    while (liveLogEl.children.length > MAX_LOG_LINES) {
      liveLogEl.removeChild(liveLogEl.firstChild);
    }

    if (nearBottom) {
      liveLogEl.scrollTop = liveLogEl.scrollHeight;
    }
  }

  function setStatusPill(connected, text) {
    const el = byId("server-status");
    if (!el) return;

    el.classList.toggle("status-ok", connected);
    el.classList.toggle("status-danger", !connected);

    const strong = el.querySelector("strong") || el;
    setText(strong, text);

    const pulse = el.querySelector(".pulse-dot");
    if (pulse) pulse.classList.toggle("offline", !connected);
  }

  function setSystemStatus(id, value, ok = null) {
    const el = byId(id);
    if (!el) return;
    setText(el, value);
    el.style.color = ok === null ? "" : ok ? "var(--green)" : "var(--red)";
    const article = el.closest("article");
    setElementState(article, ok === null ? "warn" : ok ? "ok" : "bad");

    const mirrors = {
      "status-fc": "fc",
      "status-camera": "camera",
      "status-ros2": "ros2",
      "status-vision": "vision",
      "status-socket": "socket",
    };
    if (mirrors[id]) setMirrorStatus(mirrors[id], value, ok);
  }

  function markerKey(marker) {
    if (!marker) return "none";
    return `${marker.marker_id ?? ""}|${fmtNum(marker.err_x, 1)}|${fmtNum(marker.err_y, 1)}|${fmtNum(marker.z_m, 2)}`;
  }

  function getBatteryPercent(data) {
    const pct = Number(data?.battery_percent ?? data?.battery_pct);
    if (Number.isFinite(pct) && pct >= 0 && pct <= 100) return pct;

    const voltage = Number(data?.battery_v);
    if (!Number.isFinite(voltage)) return null;
    const estimated = ((voltage - 10.5) / (12.6 - 10.5)) * 100;
    return Math.max(0, Math.min(100, estimated));
  }

  function classifyGps(data) {
    const sats = Number(data?.satellites);
    const fix = String(data?.gps_fix ?? "").toUpperCase();
    if (fix.includes("3D") || fix.includes("DGPS") || sats >= 10) return { label: "GPS OK", state: "ok" };
    if (sats >= 6 || fix.includes("2D")) return { label: "GPS FAIR", state: "warn" };
    if (data?.gps_fix || Number.isFinite(sats)) return { label: "GPS WEAK", state: "bad" };
    return { label: "Unknown", state: "warn" };
  }

  function updateReadiness() {
    const telemetry = latestTelemetry || {};
    const fcConnected = telemetry.connected === true || latestHealthConnected === true;
    const gps = classifyGps(telemetry);
    const batteryPct = getBatteryPercent(telemetry);
    const batteryLow = batteryPct !== null && batteryPct < 20;
    const cameraLive = cameraHealthState === "streaming" || cameraHealthState === "reconnecting";
    const visionReady = latestMarkers !== null;

    const issues = [];
    if (!fcConnected) issues.push("Waiting for FC");
    if (gps.state === "bad") issues.push("GPS not ready");
    if (batteryLow) issues.push("Battery low");
    if (!cameraLive) issues.push("Camera not live");
    if (!socketConnected) issues.push("Socket disconnected");
    if (!visionReady) issues.push("Vision waiting");

    let state = "ready";
    let title = "Ready for Mission";
    if (!fcConnected && !socketConnected) {
      state = "offline";
      title = "System Offline";
    } else if (batteryLow || !fcConnected || gps.state === "bad") {
      state = "critical";
      title = "Action Required";
    } else if (issues.length) {
      state = "degraded";
      title = "Degraded";
    }

    const strip = byId("readiness-strip");
    if (strip) strip.dataset.state = state;
    smartText("readiness-state", title);
    smartText("readiness-detail", issues.length ? issues.slice(0, 3).join(" | ") : "FC, GPS, camera và socket sẵn sàng.");
  }

  function appendMarkerLog(marker, fallbackId) {
    if (!marker) return;

    const id = marker.marker_id ?? fallbackId ?? "N/A";
    appendLog(
      `Marker ${id} | err=(${fmtNum(marker.err_x, 1)}, ${fmtNum(marker.err_y, 1)}) | z=${fmtNum(marker.z_m, 2)}m | d=${fmtNum(marker.distance_m, 2)}m`,
      "info"
    );
  }

  function updateMiniOverlay(data) {
    const now = performance.now();
    if (now - lastMiniOverlayAt < MINI_OVERLAY_MS) return;
    lastMiniOverlayAt = now;

    smartText("mini-mode", `Mode: ${data.mode ?? "N/A"}`);
    smartText(
      "mini-alt",
      data.rel_alt_m !== null && data.rel_alt_m !== undefined ? `Alt: ${fmtNum(data.rel_alt_m, 2)} m` : "Alt: N/A"
    );
    smartText(
      "mini-speed",
      data.speed_mps !== null && data.speed_mps !== undefined ? `Speed: ${fmtNum(data.speed_mps, 2)} m/s` : "Speed: N/A"
    );
    smartText(
      "mini-battery",
      data.battery_v !== null && data.battery_v !== undefined ? `Battery: ${fmtNum(data.battery_v, 2)} V` : "Battery: N/A"
    );
  }

  function renderTelemetryNow() {
    telemetryRenderScheduled = false;
    telemetryRenderTimer = null;
    if (document.hidden) return;

    const data = latestTelemetry;
    if (!data) return;

    const now = performance.now();
    if (now - lastTelemetryRenderAt < UI_THROTTLE_MS) {
      scheduleTelemetryRender(UI_THROTTLE_MS - (now - lastTelemetryRenderAt));
      return;
    }
    lastTelemetryRenderAt = now;

    const connected = !!data.connected;
    setStatusPill(
      connected,
      connected ? "FC connected" : `FC disconnected${data.error ? " | " + data.error : ""}`
    );

    smartText("mode", data.mode ?? "N/A");
    smartText("top-mode", data.mode ?? "N/A");
    const armedText = data.armed === true ? "ARMED" : data.armed === false ? "DISARMED" : "N/A";
    smartText("armed", armedText);
    smartText("top-armed", armedText);
    setMetricState("top-mode", data.mode ? "ok" : "warn");
    setMetricState("top-armed", data.armed === true ? "warn" : data.armed === false ? "ok" : "warn");
    setMetricState("armed", data.armed === true ? "warn" : data.armed === false ? "ok" : "warn");

    const battery =
      data.battery_v !== null && data.battery_v !== undefined ? `${fmtNum(data.battery_v, 2)} V` : "N/A";
    smartText("battery", battery);
    smartText("top-battery", battery);
    const batteryPct = getBatteryPercent(data);
    const batteryFill = byId("battery-fill");
    if (batteryFill) batteryFill.style.width = batteryPct !== null ? `${batteryPct}%` : "0%";
    const batteryState = batteryPct === null ? "warn" : batteryPct < 20 ? "bad" : batteryPct < 35 ? "warn" : "ok";
    setMetricState("battery", batteryState);
    setMetricState("top-battery", batteryState);

    smartText("gps-fix", data.gps_fix ?? "N/A");
    smartText("sats", data.satellites ?? "N/A");
    smartText("top-gps", data.satellites ?? "N/A");
    const gps = classifyGps(data);
    smartText("gps-quality", gps.label);
    setElementState(byId("gps-quality"), gps.state);
    setMetricState("gps-fix", gps.state);
    setMetricState("top-gps", gps.state);
    smartText("alt", data.rel_alt_m !== null && data.rel_alt_m !== undefined ? `${fmtNum(data.rel_alt_m, 2)} m` : "N/A");
    smartText("speed", data.speed_mps !== null && data.speed_mps !== undefined ? `${fmtNum(data.speed_mps, 2)} m/s` : "N/A");

    if (isFiniteNumber(data.lat) && isFiniteNumber(data.lon)) {
      smartText("latlon", `${fmtNum(data.lat, 7)}, ${fmtNum(data.lon, 7)}`);
      smartText("map-drone-position", `${fmtNum(data.lat, 6)}, ${fmtNum(data.lon, 6)}`);
    } else {
      smartText("latlon", "N/A");
      smartText("map-drone-position", "N/A");
    }
    smartText("map-last-update", nowText());

    updateMiniOverlay(data);
    setSystemStatus("status-fc", connected ? "ONLINE" : "OFFLINE", connected);
    setSystemStatus("status-ros2", connected || data.last_update_unix ? "ACTIVE" : "WAITING", connected || !!data.last_update_unix);
    latestHealthConnected = connected;
    updateReadiness();

    if (window.updateDronePositionOnMap) {
      window.updateDronePositionOnMap(data);
    }

    if (data.mode && data.mode !== lastTelemetryMode) {
      appendLog(`Flight mode: ${data.mode}`, "info");
      lastTelemetryMode = data.mode;
    }
  }

  function scheduleTelemetryRender(delayMs = 0) {
    if (document.hidden) return;
    if (telemetryRenderScheduled) return;
    telemetryRenderScheduled = true;
    const run = () => requestAnimationFrame(renderTelemetryNow);
    if (delayMs > 0) {
      telemetryRenderTimer = window.setTimeout(run, delayMs);
    } else {
      run();
    }
  }

  function resolveMissionProgress(state) {
    const text = String(state || "N/A").toUpperCase();
    const exact = missionProgress.find(([name]) => name === text);
    if (exact) return exact;
    const partial = missionProgress.find(([name]) => text.includes(name));
    return partial || ["N/A", 0, "Waiting"];
  }

  function resolveMissionStep(state) {
    const text = String(state || "").toUpperCase();
    if (text.includes("TAKEOFF")) return "TAKEOFF";
    if (text.includes("HOLD")) return "HOLD";
    if (text.includes("GOTO") || text.includes("ROUTE") || text.includes("RTL")) return "GOTO";
    if (text.includes("SEARCH_BIG") || (text.includes("SEARCH") && text.includes("BIG"))) return "SEARCH_BIG";
    if (text.includes("ALIGN_BIG")) return "ALIGN_BIG";
    if (text.includes("LOCK_BIG")) return "LOCK_BIG";
    if (text.includes("DESCEND")) return "DESCEND";
    if (text.includes("SEARCH_SMALL") || (text.includes("SEARCH") && text.includes("SMALL"))) return "SEARCH_SMALL";
    if (text.includes("ALIGN_SMALL")) return "ALIGN_SMALL";
    if (text.includes("LOCK_SMALL") || text.includes("LAND")) return "LAND";
    if (text.includes("DROP") || text.includes("DONE")) return "DROP";
    return "";
  }

  function updateMissionTimeline(state) {
    const active = resolveMissionStep(state);
    const activeIndex = missionStepOrder.indexOf(active);
    document.querySelectorAll("[data-mission-step]").forEach((item) => {
      const idx = missionStepOrder.indexOf(item.dataset.missionStep);
      item.classList.toggle("is-active", idx === activeIndex);
      item.classList.toggle("is-done", activeIndex >= 0 && idx >= 0 && idx < activeIndex);
    });
  }

  function setMarkerAlignment(kind, seen, marker) {
    const card = document.querySelector(`[data-marker-card="${kind}"]`);
    const dot = byId(`${kind}-align-dot`);
    if (card) card.dataset.state = seen ? "seen" : "missing";
    if (!dot) return;

    if (!seen || !marker) {
      dot.style.transform = "translate(-50%, -50%)";
      return;
    }

    const errX = Number(marker.err_x);
    const errY = Number(marker.err_y);
    if (!Number.isFinite(errX) || !Number.isFinite(errY)) {
      dot.style.transform = "translate(-50%, -50%)";
      return;
    }

    const clamp = (value, min, max) => Math.min(max, Math.max(min, value));
    const x = clamp(errX / 120, -0.42, 0.42) * 76;
    const y = clamp(errY / 120, -0.42, 0.42) * 76;
    dot.style.transform = `translate(calc(-50% + ${x.toFixed(1)}px), calc(-50% + ${y.toFixed(1)}px))`;
  }

  function renderMissionNow() {
    missionRenderScheduled = false;
    missionRenderTimer = null;
    if (document.hidden) return;

    const data = latestMission;
    if (!data) return;

    const now = performance.now();
    if (now - lastMissionRenderAt < UI_THROTTLE_MS) {
      scheduleMissionRender(UI_THROTTLE_MS - (now - lastMissionRenderAt));
      return;
    }
    lastMissionRenderAt = now;

    smartText("mission-state", data.state ?? "N/A");
    smartText("mission-mode", data.mode ?? "N/A");
    smartText("mission-armed", data.armed === true ? "ARMED" : data.armed === false ? "DISARMED" : "N/A");

    const progress = resolveMissionProgress(data.state);
    smartText("mission-phase", progress[2]);
    updateMissionTimeline(data.state);
    const fill = byId("mission-progress-fill");
    if (fill) fill.style.width = `${progress[1]}%`;

    if (data.goal && isFiniteNumber(data.goal.lat) && isFiniteNumber(data.goal.lon)) {
      smartText("mission-goal", `${Number(data.goal.lat).toFixed(7)}, ${Number(data.goal.lon).toFixed(7)}`);
      smartText("map-goal-position", `${Number(data.goal.lat).toFixed(6)}, ${Number(data.goal.lon).toFixed(6)}`);
      if (window.updateMissionGoalOnMap) window.updateMissionGoalOnMap(data.goal);
    } else {
      smartText("mission-goal", "N/A");
      smartText("map-goal-position", "N/A");
    }

    smartText("big-marker-seen", data.big_marker_seen ? "YES" : "NO");
    smartText("big-marker-id", data.big_marker_id ?? "N/A");

    const bm = data.big_marker || {};
    smartText("big-err-x", fmtNum(bm.err_x, 1));
    smartText("big-err-y", fmtNum(bm.err_y, 1));
    smartText("big-z", bm.z_m !== undefined ? `${fmtNum(bm.z_m, 2)} m` : "N/A");
    smartText("big-distance", bm.distance_m !== undefined ? `${fmtNum(bm.distance_m, 2)} m` : "N/A");
    setMarkerAlignment("big", !!data.big_marker_seen, bm);

    smartText("small-marker-seen", data.small_marker_seen ? "YES" : "NO");
    smartText("small-marker-id", data.small_marker_id ?? "N/A");

    const sm = data.small_marker || {};
    smartText("small-err-x", fmtNum(sm.err_x, 1));
    smartText("small-err-y", fmtNum(sm.err_y, 1));
    smartText("small-z", sm.z_m !== undefined ? `${fmtNum(sm.z_m, 2)} m` : "N/A");
    smartText("small-distance", sm.distance_m !== undefined ? `${fmtNum(sm.distance_m, 2)} m` : "N/A");
    setMarkerAlignment("small", !!data.small_marker_seen, sm);

    if (data.state && data.state !== lastMissionState) {
      appendLog(`Mission state -> ${data.state}`, "ok");
      lastMissionState = data.state;
    }

    if (data.big_marker_seen !== lastBigMarkerSeen) {
      if (data.big_marker_seen) appendLog(`Big marker detected (ID ${data.big_marker_id})`, "ok");
      else if (lastBigMarkerSeen === true) appendLog("Big marker lost", "warn");
      lastBigMarkerSeen = data.big_marker_seen;
    }

    if (data.small_marker_seen !== lastSmallMarkerSeen) {
      if (data.small_marker_seen) appendLog(`Small marker detected (ID ${data.small_marker_id})`, "ok");
      else if (lastSmallMarkerSeen === true) appendLog("Small marker lost", "warn");
      lastSmallMarkerSeen = data.small_marker_seen;
    }

    const bmKey = markerKey(bm);
    if (data.big_marker_seen && bmKey !== lastBigMarkerKey) {
      appendMarkerLog(bm, data.big_marker_id ?? 150);
      lastBigMarkerKey = bmKey;
    }

    const smKey = markerKey(sm);
    if (data.small_marker_seen && smKey !== lastSmallMarkerKey) {
      appendMarkerLog(sm, data.small_marker_id ?? 40);
      lastSmallMarkerKey = smKey;
    }
  }

  function scheduleMissionRender(delayMs = 0) {
    if (document.hidden) return;
    if (missionRenderScheduled) return;
    missionRenderScheduled = true;
    const run = () => requestAnimationFrame(renderMissionNow);
    if (delayMs > 0) {
      missionRenderTimer = window.setTimeout(run, delayMs);
    } else {
      run();
    }
  }

  function renderMarkersNow() {
    markerRenderScheduled = false;
    if (document.hidden) return;

    const markers = latestMarkers;
    setSystemStatus("status-vision", Array.isArray(markers) && markers.length ? "MARKERS" : "READY", true);
    if (window.updateArucoMarkersOnMap) {
      window.updateArucoMarkersOnMap(markers);
    }
    updateReadiness();
  }

  function scheduleMarkerRender() {
    if (document.hidden) return;
    if (markerRenderScheduled) return;
    markerRenderScheduled = true;
    requestAnimationFrame(renderMarkersNow);
  }

  function acceptTelemetry(data, source = "socket") {
    latestTelemetry = data || {};
    lastTelemetryAt = Date.now();
    scheduleTelemetryRender();
  }

  function acceptMission(data) {
    latestMission = data || {};
    lastMissionAt = Date.now();
    scheduleMissionRender();
  }

  function acceptMarkers(markers) {
    latestMarkers = Array.isArray(markers) ? markers : [];
    scheduleMarkerRender();
  }

  async function loadHealth() {
    try {
      const data = await fetchJson("/health");
      const connected = !!data.fc_connected;
      latestHealthConnected = connected;
      setStatusPill(connected, connected ? "FC connected" : "Server OK - FC not connected");
      setSystemStatus("status-fc", connected ? "ONLINE" : "OFFLINE", connected);
      updateReadiness();
    } catch (err) {
      console.error("[WEB] /health error:", err);
      setStatusPill(false, "Server offline");
      setSystemStatus("status-fc", "OFFLINE", false);
      latestHealthConnected = false;
      updateReadiness();
    }
  }

  function telemetryIsFresh() {
    return socketConnected && Date.now() - lastTelemetryAt < TELEMETRY_STALE_MS;
  }

  function missionIsFresh() {
    return socketConnected && Date.now() - lastMissionAt < MISSION_STALE_MS;
  }

  async function pollTelemetryFallback() {
    if (telemetryIsFresh()) return;

    try {
      acceptTelemetry(await fetchJson("/api/telemetry"), "poll");
    } catch (err) {
      console.error("[WEB] telemetry poll error:", err);
    }
  }

  async function pollMissionFallback() {
    if (missionIsFresh()) return;

    try {
      acceptMission(await fetchJson("/api/mission/state"));
    } catch (err) {
      console.error("[WEB] mission poll error:", err);
    }
  }

  async function loadMarkersOnce() {
    try {
      acceptMarkers(await fetchJson("/api/aruco-markers"));
    } catch (err) {
      console.error("[WEB] load markers error:", err);
    }
  }

  function tickPolling() {
    const now = Date.now();
    const healthInterval = document.hidden ? HEALTH_HIDDEN_MS : HEALTH_VISIBLE_MS;
    const fallbackInterval = document.hidden ? FALLBACK_HIDDEN_MS : FALLBACK_VISIBLE_MS;

    if (now - lastHealthPollAt >= healthInterval) {
      lastHealthPollAt = now;
      loadHealth();
    }

    if (now - lastFallbackPollAt >= fallbackInterval) {
      lastFallbackPollAt = now;
      pollTelemetryFallback();
      pollMissionFallback();
    }
  }

  function startManagedInterval(fn, ms) {
    const id = window.setInterval(fn, ms);
    timers.add(id);
    return id;
  }

  function initLogControls() {
    const clearLogBtn = byId("clear-log-btn");
    const toggleLogBtn = byId("log-toggle-btn");
    const liveLogEl = byId("live-log");
    const logPanel = byId("log-panel");

    if (clearLogBtn) {
      clearLogBtn.addEventListener("click", () => {
        if (liveLogEl) liveLogEl.innerHTML = "";
      });
    }

    if (toggleLogBtn && logPanel) {
      toggleLogBtn.addEventListener("click", () => {
        const expanded = !logPanel.classList.contains("is-expanded");
        logPanel.classList.toggle("is-expanded", expanded);
        setText(toggleLogBtn, expanded ? "Collapse" : "Expand");
      });
    }

    document.querySelectorAll("[data-log-filter]").forEach((btn) => {
      btn.addEventListener("click", () => {
        activeLogFilter = btn.dataset.logFilter || "all";
        document.querySelectorAll("[data-log-filter]").forEach((item) => {
          item.classList.toggle("active", item === btn);
        });
        if (liveLogEl) Array.from(liveLogEl.children).forEach(applyLogFilter);
      });
    });
  }

  function initFocusView() {
    const btn = byId("focus-view-btn");
    if (!btn) return;
    btn.addEventListener("click", () => {
      const enabled = !document.body.classList.contains("operator-focus");
      document.body.classList.toggle("operator-focus", enabled);
      setText(btn, enabled ? "Exit Focus" : "Focus View");
      window.setTimeout(() => window.dispatchEvent(new Event("resize")), 120);
    });
  }

  function initCameraManager() {
    const img = byId("camera-stream");
    if (!img) return null;

    const streamUrl = img.dataset.streamUrl || img.getAttribute("src") || "/video_feed";
    const pauseBtn = byId("camera-toggle-btn");
    const reconnectBtn = byId("camera-reconnect-btn");
    const fullscreenBtn = byId("fullscreen-camera-btn");
    const frame = byId("camera-frame");
    const statusEl = byId("camera-status-text");

    let userPaused = false;
    let active = false;
    let reconnectTimer = null;
    let hiddenPauseTimer = null;
    let reconnectAttempts = 0;
    let streamToken = 0;

    function setCameraStatus(text, state = "idle") {
      cameraHealthState = state;
      setText(statusEl, text);
      const badge = byId("camera-status-badge");
      if (badge) {
        badge.dataset.state = state;
      }
      setSystemStatus(
        "status-camera",
        text,
        state === "streaming" || state === "reconnecting" ? true : state === "error" ? false : null
      );
      updateReadiness();
    }

    function clearReconnectTimer() {
      if (reconnectTimer) {
        window.clearTimeout(reconnectTimer);
        reconnectTimer = null;
      }
    }

    function nextStreamUrl() {
      streamToken += 1;
      const joiner = streamUrl.includes("?") ? "&" : "?";
      return `${streamUrl}${joiner}_=${Date.now()}-${streamToken}`;
    }

    function startStream(reason = "resume") {
      if (userPaused || document.hidden) return;
      if (active && img.getAttribute("src")) return;
      clearReconnectTimer();
      active = true;
      reconnectAttempts = 0;
      setCameraStatus(reason === "reconnect" ? "Reconnecting..." : "Streaming", reason === "reconnect" ? "reconnecting" : "streaming");
      img.setAttribute("src", nextStreamUrl());

      if (pauseBtn) setText(pauseBtn, "Pause Camera");
    }

    function stopStream(statusText = "Paused", state = "paused") {
      clearReconnectTimer();
      active = false;
      if (img.getAttribute("src")) {
        img.removeAttribute("src");
      }
      setCameraStatus(statusText, state);
      if (pauseBtn) setText(pauseBtn, "Resume Camera");
    }

    function scheduleReconnect() {
      if (userPaused || document.hidden) return;
      active = false;
      clearReconnectTimer();
      reconnectAttempts += 1;
      const delay = Math.min(12000, 900 * reconnectAttempts);
      setCameraStatus(`Reconnecting in ${Math.round(delay / 1000)}s`, "reconnecting");
      reconnectTimer = window.setTimeout(() => {
        img.removeAttribute("src");
        window.setTimeout(() => startStream("reconnect"), 180);
      }, delay);
    }

    img.addEventListener("load", () => {
      active = true;
      reconnectAttempts = 0;
      setCameraStatus("Streaming", "streaming");
    });

    img.addEventListener("error", () => {
      if (!img.getAttribute("src")) return;
      setCameraStatus("Camera error", "error");
      scheduleReconnect();
    });

    if (pauseBtn) {
      pauseBtn.addEventListener("click", () => {
        userPaused = !userPaused;
        if (userPaused) {
          stopStream("Paused", "paused");
        } else {
          startStream("resume");
        }
      });
    }

    if (reconnectBtn) {
      reconnectBtn.addEventListener("click", () => {
        userPaused = false;
        stopStream("Reconnecting...", "reconnecting");
        window.setTimeout(() => startStream("reconnect"), 220);
      });
    }

    if (fullscreenBtn && frame) {
      fullscreenBtn.addEventListener("click", async () => {
        try {
          if (!document.fullscreenElement) await frame.requestFullscreen();
          else await document.exitFullscreen();
        } catch (err) {
          console.error("[WEB] fullscreen error:", err);
        }
      });
    }

    function handleVisibilityChange() {
      if (document.hidden) {
        if (hiddenPauseTimer) window.clearTimeout(hiddenPauseTimer);
        hiddenPauseTimer = window.setTimeout(() => {
          if (document.hidden && !userPaused) stopStream("Paused - hidden tab", "paused");
        }, 900);
      } else {
        if (hiddenPauseTimer) {
          window.clearTimeout(hiddenPauseTimer);
          hiddenPauseTimer = null;
        }
        if (!userPaused) {
          window.setTimeout(() => startStream("resume"), 250);
        }
      }
    }

    document.addEventListener("visibilitychange", handleVisibilityChange);
    window.addEventListener("pagehide", () => stopStream("Paused", "paused"));

    startStream("resume");

    return {
      stop: () => stopStream("Paused", "paused"),
      start: () => startStream("resume"),
    };
  }

  function initSocket() {
    if (socket) return;

    if (typeof io !== "function") {
      appendLog("Socket.IO client not loaded", "error");
      setSystemStatus("status-socket", "OFFLINE", false);
      updateReadiness();
      return;
    }

    socket = io();

    socket.on("connect", () => {
      socketConnected = true;
      appendLog("Socket connected", "ok");
      setSystemStatus("status-socket", "ONLINE", true);
      updateReadiness();
      lastFallbackPollAt = Date.now();
    });

    socket.on("disconnect", () => {
      socketConnected = false;
      appendLog("Socket disconnected", "warn");
      setStatusPill(false, "Socket disconnected");
      setSystemStatus("status-socket", "OFFLINE", false);
      updateReadiness();
      lastFallbackPollAt = 0;
    });

    socket.on("telemetry", (data) => acceptTelemetry(data, "socket"));
    socket.on("aruco_markers", acceptMarkers);
    socket.on("mission_state", acceptMission);
    socket.on("live_log", (payload) => {
      if (typeof payload === "string") appendLog(payload, "info");
      else appendLog(payload?.message ?? "Live log event", payload?.level ?? "info");
    });
  }

  function initDashboard() {
    if (!byId("dashboard-overview")) return;

    initLogControls();
    initFocusView();
    initCameraManager();
    initSocket();

    lastHealthPollAt = 0;
    lastFallbackPollAt = 0;
    tickPolling();
    loadMarkersOnce();

    startManagedInterval(tickPolling, 1000);

    document.addEventListener("visibilitychange", () => {
      document.body.classList.toggle("is-hidden-tab", document.hidden);
      if (!document.hidden) {
        if (latestTelemetry) scheduleTelemetryRender();
        if (latestMission) scheduleMissionRender();
        if (latestMarkers) scheduleMarkerRender();
        tickPolling();
      }
    });

    appendLog("Web UI started", "info");
    updateReadiness();
  }

  document.addEventListener("DOMContentLoaded", initDashboard);
})();
