(() => {
  "use strict";

  const data = window.UAM_MOTION_DATA;
  const engine = window.UAM_TRAFFIC_ENGINE;
  const errorPanel = document.getElementById("error-panel");
  if (!data || !window.L || !engine) {
    errorPanel.hidden = false;
    errorPanel.textContent = !data
      ? "Lane-change data bundle is missing. Rebuild it with scripts/build_motion_dashboard.py."
      : !window.L ? "The bundled Leaflet map library did not load." : "The shared radio engine did not load.";
    return;
  }

  const colors = { C: "#238b57", R: "#e3b735", F: "#d65353" };
  const POLICIES = "CRF";
  const summary = data.summary;
  const parameters = summary.parameters;
  const field = Object.fromEntries(summary.row_fields.map((name, i) => [name, i]));
  const route = data.route_metric;
  const corridorLengthM = route.at(-1).s_m;
  const frameS = Number(summary.frame_s);
  const offsets = [...new Set(summary.grid.map((cell) => cell.offset_m))].sort((a, b) => a - b);
  const altitudes = [...new Set(summary.grid.map((cell) => cell.altitude_m))].sort((a, b) => a - b);
  const cellIndexOf = summary.grid.map((cell) => ({
    offset: offsets.indexOf(cell.offset_m), altitude: altitudes.indexOf(cell.altitude_m),
  }));
  const controllerLabels = {
    cruise: "Cruise · no binding leader",
    ks2_tracker: "AKS reference tracking",
    feedback_fallback: "ACC feedback spacing",
  };
  const $ = (id) => document.getElementById(id);
  const fmt = (value, digits = 1) => Number(value).toFixed(digits);
  const spacing = summary.spacing_m;
  const referenceSpeed = Number(parameters.cruise_mps);

  function formatTime(seconds) {
    const value = Math.max(0, Math.round(seconds));
    return `${String(Math.floor(value / 60)).padStart(2, "0")}:${String(value % 60).padStart(2, "0")}`;
  }

  function lowerTail(values, rho) {
    const ordered = [...values].sort((a, b) => a - b);
    return ordered[Math.max(0, Math.min(ordered.length - 1, Math.floor((1 - rho) * (ordered.length - 1))))];
  }

  // ------------------------------------------------------------------ fixed settings
  const fixedSettings = [
    ["Cruise speed", `${fmt(parameters.cruise_mps, 0)} m/s`],
    ["Speed range", `${fmt(parameters.speed_min_mps, 0)}–${fmt(parameters.speed_max_mps, 0)} m/s`],
    ["Longitudinal accel.", `+${fmt(parameters.acceleration_limit_mps2)} / −${fmt(parameters.deceleration_limit_mps2)} m/s²`],
    ["Lane-change accel.", `${fmt(summary.envelope.lateral_accel_max_mps2, 3)} m/s² (0.1 g)`],
    ["Spacing law", `${fmt(parameters.d0_m)} + τ·v + ${parameters.buffer_s2_per_m}·v²`],
    ["Spacing C / R / F", `${fmt(spacing.C, 0)} / ${fmt(spacing.R, 0)} / ${fmt(spacing.F, 0)} m`],
    ["Headways τ", `${fmt(parameters.tau_c_s, 0)} / ${fmt(parameters.tau_r_s, 0)} / ${fmt(parameters.tau_f_s, 0)} s`],
    ["Demand", `1 request / ${fmt(summary.global_headway_s, 0)} s · ${summary.requests} requests`],
    ["Replay grid", `${fmt(frameS, 0)} s (run sampled radio every ${fmt(parameters.radio_s, 0)} s)`],
  ];
  $("settings-list").replaceChildren(...fixedSettings.map(([label, value]) => {
    const item = document.createElement("div");
    item.innerHTML = `<dt>${label}</dt><dd>${value}</dd>`;
    return item;
  }));
  $("run-note").textContent = `Run ${summary.run_id} · validation ${summary.validation} · ${summary.display.route_label || summary.scenario_id}`;
  $("footer-run").textContent = `Replay of verified run ${summary.run_id} · frames every ${fmt(frameS, 0)} s, interpolated for playback`;

  // ------------------------------------------------------------------ archived state
  function decodeRow(row) {
    return {
      idx: row[field.aircraft], lat: row[field.lat_e5] / 1e5, lon: row[field.lon_e5] / 1e5,
      x: row[field.x_m], y: row[field.y_m], altitude: row[field.altitude_m], offset: row[field.offset_m],
      speed: row[field.speed_dmps] / 10, policy: POLICIES[row[field.policy]],
      controller: summary.controllers[row[field.controller]], gap: row[field.gap_m],
      moving: row[field.moving] === 1, cell: row[field.cell], q: row[field.q_m],
    };
  }

  const decoded = {};
  const archived = {};
  Object.entries(data.cases).forEach(([caseId, caseData]) => {
    decoded[caseId] = caseData.frames.map((frame) => new Map(frame.rows.map((row) => [row[field.aircraft], decodeRow(row)])));
    archived[caseId] = {
      policies: decoded[caseId].map((rows) => new Map([...rows].map(([idx, row]) => [idx, row.policy]))),
      capacity: caseData.capacity,
      stats: { ...caseData.stats },
      observations: decoded[caseId].reduce((total, rows) => total + rows.size, 0),
    };
  });
  const active = {};
  Object.keys(archived).forEach((caseId) => { active[caseId] = archived[caseId]; });
  let recomputed = false;

  const baselineParameters = {
    thresholdDb: Number(parameters.threshold_db),
    windowS: Number(parameters.window_s),
    persistenceK: Number(parameters.persistence_k),
    coordinatedTolerance: Number(parameters.exposure_c),
    reactiveTolerance: Number(parameters.exposure_r),
    groupSize: 5,
    groupMode: String(parameters.group_mode),
    reliabilityRho: 0.95,
  };
  let currentParameters = { ...baselineParameters };

  function populateForm(values) {
    $("input-theta").value = String(values.thresholdDb);
    $("input-window").value = String(values.windowS);
    $("input-persistence").value = String(values.persistenceK);
    $("input-c-tolerance").value = String(Math.round(100 * values.coordinatedTolerance));
    $("input-r-tolerance").value = String(Math.round(100 * values.reactiveTolerance));
    $("input-group-size").value = String(values.groupSize);
    $("input-group-mode").value = values.groupMode;
    $("input-reliability").value = String(Math.round(100 * values.reliabilityRho));
  }

  function readForm() {
    const values = {
      thresholdDb: Number($("input-theta").value),
      windowS: Number($("input-window").value),
      persistenceK: Number($("input-persistence").value),
      coordinatedTolerance: Number($("input-c-tolerance").value) / 100,
      reactiveTolerance: Number($("input-r-tolerance").value) / 100,
      groupSize: Number($("input-group-size").value),
      groupMode: $("input-group-mode").value,
      reliabilityRho: Number($("input-reliability").value) / 100,
    };
    if (!(values.windowS >= frameS)) throw new Error(`exposure window must be at least ${frameS} s`);
    if (!Number.isInteger(values.persistenceK) || values.persistenceK < 1) throw new Error("persistence k must be a positive integer");
    if (!(values.coordinatedTolerance >= 0 && values.coordinatedTolerance <= values.reactiveTolerance && values.reactiveTolerance <= 1)) {
      throw new Error("exposure limits must satisfy 0 ≤ C ≤ R ≤ 1");
    }
    if (!(values.reliabilityRho > 0 && values.reliabilityRho <= 1)) throw new Error("reliability ρ must lie in (0, 1]");
    return values;
  }

  // ------------------------------------------------------------------ radio and policy recomputation
  const radioCache = {};   // caseId -> array over frames of Map(idx -> {sinr, rsrp, site})

  function radioFor(caseId) {
    if (radioCache[caseId]) return radioCache[caseId];
    radioCache[caseId] = decoded[caseId].map((rows) => new Map([...rows].map(([idx, row]) => [
      idx, engine.evaluateRadio(data.stations, summary.radio, { x_m: row.x, y_m: row.y }, row.altitude),
    ])));
    return radioCache[caseId];
  }

  function groupMembers(rows, focal, values) {
    const neighbours = Math.floor(values.groupSize / 2);
    const cap = neighbours * spacing.F;
    const focalCell = cellIndexOf[focal.cell];
    const candidates = [];
    for (const row of rows.values()) {
      if (row.idx === focal.idx) continue;
      const cell = cellIndexOf[row.cell];
      const adjacent = values.groupMode === "lane_order"
        ? row.cell === focal.cell
        : Math.abs(cell.offset - focalCell.offset) <= 1 && Math.abs(cell.altitude - focalCell.altitude) <= 1;
      if (!adjacent) continue;
      const along = Math.abs(row.q - focal.q);
      if (along > cap) continue;
      candidates.push({ row, along });
    }
    candidates.sort((a, b) => a.along - b.along);
    return [focal, ...candidates.slice(0, 2 * neighbours).map((entry) => entry.row)];
  }

  function recomputeCase(caseId, values) {
    const frames = data.cases[caseId].frames;
    const rowsByFrame = decoded[caseId];
    const radio = radioFor(caseId);
    const windowFrames = Math.max(1, Math.round(values.windowS / frameS));
    const history = new Map();
    const held = new Map();
    const candidate = new Map();
    const policies = [];
    const capacity = [];
    const counts = { C: 0, R: 0, F: 0 };
    rowsByFrame.forEach((rows, k) => {
      const ok = new Map([...rows].map(([idx]) => [idx, radio[k].get(idx).sinr >= values.thresholdDb]));
      const framePolicies = new Map();
      for (const [idx, row] of rows) {
        const members = groupMembers(rows, row, values);
        const support = members.filter((member) => ok.get(member.idx)).length / members.length;
        const past = history.get(idx) || [];
        past.push(support);
        while (past.length > windowFrames) past.shift();
        history.set(idx, past);
        const exposure = 1 - past.reduce((total, value) => total + value, 0) / past.length;
        const raw = exposure <= values.coordinatedTolerance + 1e-12 ? "C"
          : exposure <= values.reactiveTolerance + 1e-12 ? "R" : "F";
        if (!held.has(idx)) {
          held.set(idx, raw);
          candidate.set(idx, { policy: raw, count: 0 });
        } else if (raw === held.get(idx)) {
          candidate.set(idx, { policy: raw, count: 0 });
        } else {
          const previous = candidate.get(idx);
          const count = previous && previous.policy === raw ? previous.count + 1 : 1;
          candidate.set(idx, { policy: raw, count });
          if (count >= values.persistenceK) {
            held.set(idx, raw);
            candidate.set(idx, { policy: raw, count: 0 });
          }
        }
        framePolicies.set(idx, held.get(idx));
        counts[held.get(idx)] += 1;
      }
      policies.push(framePolicies);
      const framePolicyValues = [...framePolicies.values()];
      const mean = framePolicyValues.reduce((total, policy) => total + spacing[policy], 0) / Math.max(1, framePolicyValues.length);
      capacity.push([frames[k].t, 3600 * referenceSpeed / mean,
        framePolicyValues.filter((p) => p === "C").length,
        framePolicyValues.filter((p) => p === "R").length,
        framePolicyValues.filter((p) => p === "F").length]);
    });
    const total = counts.C + counts.R + counts.F;
    const [lo, hi] = summary.compared_window_s;
    const windowValues = capacity.filter((row) => row[0] >= lo && row[0] <= hi).map((row) => row[1]);
    return {
      policies, capacity, observations: total,
      stats: {
        ...data.cases[caseId].stats,
        policy_shares: { C: counts.C / total, R: counts.R / total, F: counts.F / total },
        window_mean_uam_h: windowValues.reduce((sum, value) => sum + value, 0) / windowValues.length,
        window_q95_uam_h: lowerTail(windowValues, values.reliabilityRho),
      },
    };
  }

  function applyRecompute(values) {
    const before = active[caseId].stats.policy_shares;
    Object.keys(data.cases).forEach((id) => { active[id] = recomputeCase(id, values); });
    currentParameters = values;
    recomputed = true;
    seriesCache = null;
    const after = active[caseId].stats.policy_shares;
    $("experiment-status").textContent =
      `Recomputed on the ${fmt(frameS, 0)} s grid · C/R/F ${fmt(100 * before.C)}/${fmt(100 * before.R)}/${fmt(100 * before.F)}% → `
      + `${fmt(100 * after.C)}/${fmt(100 * after.R)}/${fmt(100 * after.F)}%`;
    renderCompareTable();
    draw();
  }

  function restoreArchived() {
    Object.keys(data.cases).forEach((id) => { active[id] = archived[id]; });
    currentParameters = { ...baselineParameters };
    recomputed = false;
    seriesCache = null;
    populateForm(currentParameters);
    $("experiment-status").textContent = "Archived run restored";
    renderCompareTable();
    draw();
  }

  $("experiment-form").addEventListener("submit", (event) => {
    event.preventDefault();
    $("run-experiment").disabled = true;
    $("experiment-status").textContent = "Recomputing…";
    window.setTimeout(() => {
      try {
        applyRecompute(readForm());
      } catch (error) {
        $("experiment-status").textContent = `Input error: ${error.message}`;
      } finally {
        $("run-experiment").disabled = false;
      }
    }, 20);
  });
  $("reset-experiment").addEventListener("click", restoreArchived);

  // ------------------------------------------------------------------ case state
  const params = new URLSearchParams(window.location.search);
  let caseId = data.cases[params.get("case")] ? params.get("case") : "spatial_grid";
  let caseData = null;
  let frames = [];
  let aircraft = [];
  let rowsByFrame = [];
  let index = 0;
  let simulatedTime = 0;
  let selectedIdx = null;
  let playing = false;
  let lastTimestamp = null;
  let rafId = null;
  let seriesCache = null;

  function loadCase(nextCase) {
    caseId = nextCase;
    caseData = data.cases[caseId];
    frames = caseData.frames;
    aircraft = caseData.aircraft;
    rowsByFrame = decoded[caseId];
    index = 0;
    simulatedTime = frames[0].t;
    seriesCache = null;
    if (selectedIdx !== null && !rowsByFrame.some((rows) => rows.has(selectedIdx))) selectedIdx = null;
    $("case-select").value = caseId;
    $("time-slider").max = String(frames.length - 1);
    $("total-time").textContent = `/ ${formatTime(frames.at(-1).t)}`;
    const [lo, hi] = summary.compared_window_s;
    $("window-caption").textContent = `mean / 95% reliable · ${fmt(lo / 60)}–${fmt(hi / 60)} min`;
    renderCompareTable();
    clearAircraftLayers();
    const url = new URL(window.location.href);
    url.searchParams.set("case", caseId);
    window.history.replaceState(null, "", url);
  }

  function policyAt(k, idx, fallback) {
    const frame = active[caseId].policies[k];
    return (frame && frame.get(idx)) || fallback;
  }

  function statesAt(t) {
    const k = Math.max(0, Math.min(frames.length - 1, Math.floor((t - frames[0].t) / frameS + 1e-9)));
    const a = rowsByFrame[k];
    const b = rowsByFrame[Math.min(frames.length - 1, k + 1)];
    const span = k + 1 < frames.length ? frames[k + 1].t - frames[k].t : 1;
    const f = Math.max(0, Math.min(1, (t - frames[k].t) / span));
    const out = [];
    for (const [idx, row] of a) {
      const policy = policyAt(k, idx, row.policy);
      const next = b.get(idx);
      if (!next || f === 0) { out.push({ ...row, policy }); continue; }
      out.push({
        ...row, policy,
        lat: row.lat + f * (next.lat - row.lat), lon: row.lon + f * (next.lon - row.lon),
        x: row.x + f * (next.x - row.x), y: row.y + f * (next.y - row.y),
        altitude: row.altitude + f * (next.altitude - row.altitude), offset: row.offset + f * (next.offset - row.offset),
        speed: row.speed + f * (next.speed - row.speed), q: row.q + f * (next.q - row.q),
      });
    }
    return out;
  }

  function headingOf(state) {
    const point = engine.interpolateRoute(route, corridorLengthM, state.q, 0);
    return Math.atan2(point.tangentX, point.tangentY) * 180 / Math.PI;
  }

  function activeChange(idx, t) {
    return caseData.changes.find((change) => change[1] === idx && change[0] <= t + 1e-9 && t < change[0] + change[4]);
  }

  function cellLabel(cellIndex) {
    const cell = summary.grid[cellIndex];
    return `${cell.offset_m > 0 ? "+" : ""}${fmt(cell.offset_m, 0)} m · ${fmt(cell.altitude_m, 0)} m`;
  }

  // ------------------------------------------------------------------ lane geometry
  // The corridor polyline has sub-metre segments at some vertices; offsetting each vertex along its
  // own segment normal spikes there. Sample the centreline at a fixed step and use a smoothed
  // tangent so the parallel lanes stay parallel through corners.
  const LANE_STEP_M = 200;
  const TANGENT_HALF_M = 150;
  function lanePoint(sM, offsetM) {
    const centre = engine.interpolateRoute(route, corridorLengthM, sM, 0);
    if (offsetM === 0) return { lat: centre.lat, lon: centre.lon };
    const back = engine.interpolateRoute(route, corridorLengthM, Math.max(0, sM - TANGENT_HALF_M), 0);
    const ahead = engine.interpolateRoute(route, corridorLengthM, Math.min(corridorLengthM, sM + TANGENT_HALF_M), 0);
    const dx = ahead.x_m - back.x_m;
    const dy = ahead.y_m - back.y_m;
    const length = Math.hypot(dx, dy) || 1;
    const normalX = -dy / length;
    const normalY = dx / length;
    return {
      lat: centre.lat + offsetM * normalY / 111320,
      lon: centre.lon + offsetM * normalX / (111320 * Math.cos(centre.lat * Math.PI / 180)),
    };
  }
  function laneLine(offsetM) {
    const steps = Math.ceil(corridorLengthM / LANE_STEP_M);
    return Array.from({ length: steps + 1 }, (_unused, i) => lanePoint(Math.min(corridorLengthM, i * LANE_STEP_M), offsetM));
  }
  const laneGeometry = new Map(offsets.map((offset) => [offset, laneLine(offset)]));

  // ------------------------------------------------------------------ map
  const map = L.map("motion-map", { preferCanvas: true });
  L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", { maxZoom: 19, attribution: "&copy; OpenStreetMap contributors" }).addTo(map);
  const laneLines = offsets.map((offset) => L.polyline(
    laneGeometry.get(offset).map((point) => [point.lat, point.lon]),
    offset === 0 ? { color: "#168c85", weight: 4, opacity: .85 } : { color: "#607177", weight: 2, opacity: .7, dashArray: "6 6" },
  ).addTo(map));
  const mapNode = document.getElementById("motion-map");
  let viewerMovedMap = false;
  const fitCorridor = () => {
    if (viewerMovedMap) return;
    map.invalidateSize();
    map.fitBounds(laneLines[Math.floor(laneLines.length / 2)].getBounds(), { padding: [45, 45] });
  };
  ["pointerdown", "wheel"].forEach((type) =>
    mapNode.addEventListener(type, () => { viewerMovedMap = true; }, { capture: true, passive: true }));
  fitCorridor();
  window.requestAnimationFrame(() => window.setTimeout(fitCorridor, 50));
  window.addEventListener("load", fitCorridor);
  if (window.ResizeObserver) new ResizeObserver(() => fitCorridor()).observe(mapNode);
  data.stations.forEach((site) => {
    L.circleMarker([site.lat, site.lon], { radius: 4, color: "#fff", weight: 1.2, fillColor: "#17364a", fillOpacity: .85 })
      .bindTooltip(`${site.id} · ${site.physical_form}`).addTo(map);
  });
  const servingLine = L.polyline([], { color: "#e46f51", weight: 2, opacity: .7, dashArray: "5 5" }).addTo(map);
  const markers = new Map();

  function clearAircraftLayers() {
    for (const marker of markers.values()) map.removeLayer(marker);
    markers.clear();
    if (viewer3d) { for (const entity of aircraft3d.values()) viewer3d.entities.remove(entity); aircraft3d.clear(); }
  }

  // Same aircraft symbol as the other two pages: a plane rotated to its heading and coloured by
  // policy. Altitude rides along as a small badge, because this page has three levels.
  function markerIcon(state, heading, selected) {
    const classes = ["uam-traffic-marker"];
    if (selected) classes.push("uam-traffic-marker--selected");
    if (state.moving) classes.push("uam-traffic-marker--moving");
    return L.divIcon({
      className: "",
      html: `<div class="uam-traffic-hit">`
        + `<div class="${classes.join(" ")}" style="background:${colors[state.policy]};transform:rotate(${heading + 45}deg)" aria-label="${state.policy} policy aircraft">✈</div>`
        + `<span class="motion-ac-alt">${Math.round(state.altitude)} m</span></div>`,
      iconSize: selected ? [40, 40] : [32, 32],
      iconAnchor: selected ? [20, 20] : [16, 16],
    });
  }

  function update2d(states) {
    const activeIds = new Set(states.map((state) => state.idx));
    for (const [idx, marker] of markers) {
      if (!activeIds.has(idx)) { map.removeLayer(marker); markers.delete(idx); }
    }
    states.forEach((state) => {
      const selected = state.idx === selectedIdx;
      const heading = headingOf(state);
      const key = `${state.policy}${state.moving}${selected}${Math.round(state.altitude)}${Math.round(heading / 5)}`;
      let marker = markers.get(state.idx);
      if (!marker) {
        marker = L.marker([state.lat, state.lon], { title: `Select ${aircraft[state.idx].id}`, keyboard: true })
          .on("click", () => select(state.idx)).addTo(map);
        markers.set(state.idx, marker);
      }
      marker.setLatLng([state.lat, state.lon]);
      if (marker._motionKey !== key) {
        marker.setIcon(markerIcon(state, heading, selected));
        marker.setZIndexOffset(selected ? 1200 : 800);
        marker.bindTooltip(`${aircraft[state.idx].id} · ${state.policy} · ${Math.round(state.altitude)} m`, { direction: "top" });
        marker._motionKey = key;
      }
    });
  }

  // ------------------------------------------------------------------ 3D
  let viewer3d = null;
  const aircraft3d = new Map();
  let servingLink3d = null;
  let cameraMode = "follow";
  const billboardCache = new Map();

  function billboard(policy, moving) {
    const key = `${policy}${moving}`;
    if (billboardCache.has(key)) return billboardCache.get(key);
    const canvas = document.createElement("canvas");
    canvas.width = 48; canvas.height = 48;
    const ctx = canvas.getContext("2d");
    ctx.fillStyle = colors[policy];
    ctx.strokeStyle = moving ? "#17364a" : "#ffffff";
    ctx.lineWidth = moving ? 6 : 4;
    ctx.beginPath(); ctx.arc(24, 24, 19, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
    ctx.fillStyle = "#ffffff";
    ctx.beginPath(); ctx.moveTo(22, 9); ctx.lineTo(26, 9); ctx.lineTo(29, 21); ctx.lineTo(39, 27); ctx.lineTo(39, 30); ctx.lineTo(28, 28);
    ctx.lineTo(26, 38); ctx.lineTo(22, 38); ctx.lineTo(20, 28); ctx.lineTo(9, 30); ctx.lineTo(9, 27); ctx.lineTo(19, 21); ctx.closePath(); ctx.fill();
    billboardCache.set(key, canvas);
    return canvas;
  }

  function initialize3d() {
    if (window.location.protocol === "file:" || !window.Cesium) { $("three-warning").hidden = false; $("motion-3d").hidden = true; return; }
    try {
      viewer3d = new Cesium.Viewer("motion-3d", {
        baseLayer: new Cesium.ImageryLayer(new Cesium.OpenStreetMapImageryProvider({ url: "https://tile.openstreetmap.org/", credit: "© OpenStreetMap contributors" })),
        terrainProvider: new Cesium.EllipsoidTerrainProvider(),
        animation: false, timeline: false, geocoder: false, homeButton: false, sceneModePicker: false,
        baseLayerPicker: false, navigationHelpButton: false, fullscreenButton: false, selectionIndicator: false, infoBox: false,
      });
    } catch (_error) { $("three-warning").hidden = false; viewer3d = null; return; }
    viewer3d.scene.globe.depthTestAgainstTerrain = false;
    const laneColor = { 200: "#8fb9d6", 300: "#168c85", 400: "#17364a" };
    offsets.forEach((offset) => altitudes.forEach((altitude) => {
      const positions = laneGeometry.get(offset).flatMap((point) => [point.lon, point.lat, altitude]);
      viewer3d.entities.add({ polyline: { positions: Cesium.Cartesian3.fromDegreesArrayHeights(positions), width: offset === 0 && altitude === 300 ? 4 : 2,
        material: Cesium.Color.fromCssColorString(laneColor[altitude] || "#607177").withAlpha(.65) } });
    }));
    data.stations.forEach((site) => {
      viewer3d.entities.add({ position: Cesium.Cartesian3.fromDegrees(site.lon, site.lat, site.height_m),
        point: { pixelSize: 8, color: Cesium.Color.fromCssColorString("#17364a"), outlineColor: Cesium.Color.WHITE, outlineWidth: 2, disableDepthTestDistance: Number.POSITIVE_INFINITY },
        label: { text: site.id, font: "700 11px system-ui", pixelOffset: new Cesium.Cartesian2(0, -16), fillColor: Cesium.Color.WHITE, showBackground: true,
          backgroundColor: Cesium.Color.fromCssColorString("#17364a").withAlpha(.8), distanceDisplayCondition: new Cesium.DistanceDisplayCondition(0, 12000) } });
    });
    servingLink3d = viewer3d.entities.add({ polyline: { positions: [], width: 2, material: new Cesium.PolylineDashMaterialProperty({ color: Cesium.Color.fromCssColorString("#ff7658"), dashLength: 16 }) } });
    const canvas = viewer3d.scene.canvas;
    canvas.addEventListener("pointerdown", releaseCamera, { capture: true });
    canvas.addEventListener("wheel", releaseCamera, { capture: true, passive: true });
  }

  function releaseCamera() {
    if (cameraMode === "free") return;
    cameraMode = "free";
    if (viewer3d) viewer3d.camera.lookAtTransform(Cesium.Matrix4.IDENTITY);
    annotateCamera();
  }

  function annotateCamera() {
    $("three-camera-state").textContent = cameraMode === "follow" ? "315° bearing · fixed pitch" : "Free view";
    $("three-free-view").setAttribute("aria-pressed", String(cameraMode === "free"));
    $("three-recenter").setAttribute("aria-pressed", String(cameraMode === "follow"));
    $("three-camera-note").textContent = cameraMode === "follow"
      ? `Following ${selectedIdx !== null ? aircraft[selectedIdx].id : "selected aircraft"}.`
      : "Free view · drag to orbit, scroll to zoom.";
  }

  function update3d(states, selected, link) {
    if (!viewer3d) return;
    const activeIds = new Set(states.map((state) => state.idx));
    for (const [idx, entity] of aircraft3d) {
      if (!activeIds.has(idx)) { viewer3d.entities.remove(entity); aircraft3d.delete(idx); }
    }
    states.forEach((state) => {
      const isSelected = state.idx === selectedIdx;
      const position = Cesium.Cartesian3.fromDegrees(state.lon, state.lat, state.altitude);
      let entity = aircraft3d.get(state.idx);
      if (!entity) {
        entity = viewer3d.entities.add({ position, billboard: { image: billboard(state.policy, state.moving), width: 22, height: 22, disableDepthTestDistance: Number.POSITIVE_INFINITY },
          label: { text: "", font: "700 12px system-ui", fillColor: Cesium.Color.WHITE, showBackground: true, pixelOffset: new Cesium.Cartesian2(0, -30), disableDepthTestDistance: Number.POSITIVE_INFINITY } });
        aircraft3d.set(state.idx, entity);
      }
      entity.position = position;
      const key = `${state.policy}${state.moving}${isSelected}`;
      if (entity._motionKey !== key) {
        entity.billboard.image = billboard(state.policy, state.moving);
        entity.billboard.width = isSelected ? 36 : 22;
        entity.billboard.height = isSelected ? 36 : 22;
        entity.label.show = isSelected;
        entity.label.backgroundColor = Cesium.Color.fromCssColorString(colors[state.policy]).withAlpha(.9);
        entity._motionKey = key;
      }
      if (isSelected) entity.label.text = `${aircraft[state.idx].id} · ${Math.round(state.altitude)} m · ${state.policy}`;
    });
    if (selected && link) {
      const from = Cesium.Cartesian3.fromDegrees(selected.lon, selected.lat, selected.altitude);
      servingLink3d.polyline.positions = [from, Cesium.Cartesian3.fromDegrees(link.site.lon, link.site.lat, link.site.height_m)];
      if (cameraMode === "follow") {
        viewer3d.camera.lookAt(Cesium.Cartesian3.fromDegrees(selected.lon, selected.lat, selected.altitude * .6),
          new Cesium.HeadingPitchRange(Cesium.Math.toRadians(315), Cesium.Math.toRadians(-25), 3300));
      }
    }
  }

  $("three-free-view").addEventListener("click", releaseCamera);
  $("three-recenter").addEventListener("click", () => { cameraMode = "follow"; annotateCamera(); draw(); });

  // ------------------------------------------------------------------ canvases
  function setupCanvas(canvas, height) {
    const ratio = window.devicePixelRatio || 1;
    const width = Math.max(300, canvas.clientWidth);
    canvas.width = width * ratio; canvas.height = height * ratio;
    canvas.style.height = `${height}px`;
    const ctx = canvas.getContext("2d");
    ctx.scale(ratio, ratio);
    ctx.clearRect(0, 0, width, height);
    ctx.font = "10px system-ui";
    return { ctx, width, height };
  }

  // Cross-section: per cell, how many aircraft are in it and which policies they hold.
  // Fixed vertical slots (tag, count, bar, counts) so nothing overlaps in a short cell.
  function drawGrid(states, selected) {
    const { ctx, width, height } = setupCanvas($("grid-chart"), 270);
    const pad = { left: 54, right: 14, top: 16, bottom: 34 };
    const cellW = (width - pad.left - pad.right) / offsets.length;
    const cellH = (height - pad.top - pad.bottom) / altitudes.length;
    const columnX = (offset) => pad.left + (offsets.indexOf(offset) + .5) * cellW;
    const rowY = (altitude) => pad.top + (altitudes.length - 1 - altitudes.indexOf(altitude) + .5) * cellH;
    const exactX = (offset) => pad.left + ((offset - offsets[0]) / (offsets.at(-1) - offsets[0]) * (offsets.length - 1) + .5) * cellW;
    const exactY = (altitude) => pad.top + ((altitudes.at(-1) - altitude) / (altitudes.at(-1) - altitudes[0]) * (altitudes.length - 1) + .5) * cellH;
    const counts = summary.grid.map(() => ({ C: 0, R: 0, F: 0 }));
    states.forEach((state) => { counts[state.cell][state.policy] += 1; });

    summary.grid.forEach((cell, cellIndex) => {
      const count = counts[cellIndex];
      const total = count.C + count.R + count.F;
      const left = columnX(cell.offset_m) - cellW / 2 + 5;
      const top = rowY(cell.altitude_m) - cellH / 2 + 5;
      const boxW = cellW - 10;
      const boxH = cellH - 10;
      const entry = cellIndex === summary.entry_flow_index;
      ctx.fillStyle = total ? "#eef4f1" : "#f7f7f5";
      ctx.strokeStyle = entry ? "#17364a" : "#d7ddd9";
      ctx.lineWidth = entry ? 2 : 1;
      ctx.fillRect(left, top, boxW, boxH);
      ctx.strokeRect(left, top, boxW, boxH);
      if (entry) {
        ctx.fillStyle = "#64747c";
        ctx.font = "9px system-ui";
        const tag = "entry";
        ctx.fillText(tag, left + boxW - ctx.measureText(tag).width - 8, top + 13);
      }
      ctx.fillStyle = total ? "#17364a" : "#b3bcbf";
      ctx.font = "700 21px system-ui";
      const countText = String(total);
      ctx.fillText(countText, left + 10, top + 34);
      const countWidth = ctx.measureText(countText).width;
      ctx.fillStyle = "#64747c";
      ctx.font = "9px system-ui";
      ctx.fillText(total === 1 ? "aircraft" : "aircraft", left + 14 + countWidth, top + 34);
      if (total) {
        const barW = boxW - 20;
        const barY = top + boxH - 27;
        let x = left + 10;
        POLICIES.split("").forEach((policy) => {
          const share = count[policy] / total;
          if (share <= 0) return;
          ctx.fillStyle = colors[policy];
          ctx.fillRect(x, barY, barW * share, 8);
          x += barW * share;
        });
        ctx.font = "700 10px system-ui";
        let labelX = left + 10;
        POLICIES.split("").forEach((policy) => {
          if (!count[policy]) return;
          const label = `${policy} ${count[policy]}`;
          ctx.fillStyle = colors[policy];
          ctx.fillText(label, labelX, top + boxH - 7);
          labelX += ctx.measureText(label).width + 9;
        });
      }
    });

    ctx.fillStyle = "#64747c";
    ctx.font = "10px system-ui";
    offsets.forEach((offset) => {
      const label = `${offset > 0 ? "+" : ""}${offset} m`;
      ctx.fillText(label, columnX(offset) - ctx.measureText(label).width / 2, height - 18);
    });
    ctx.fillText("lateral offset", pad.left, height - 5);
    altitudes.forEach((altitude) => ctx.fillText(`${altitude} m`, 8, rowY(altitude) + 3));

    if (selected) {
      const x = exactX(selected.offset);
      const y = exactY(selected.altitude);
      const change = activeChange(selected.idx, simulatedTime);
      if (change) {
        const target = summary.grid[change[3]];
        ctx.strokeStyle = "#ffffff"; ctx.lineWidth = 4; ctx.setLineDash([4, 3]);
        ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(columnX(target.offset_m), rowY(target.altitude_m)); ctx.stroke();
        ctx.strokeStyle = "#17364a"; ctx.lineWidth = 1.5;
        ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(columnX(target.offset_m), rowY(target.altitude_m)); ctx.stroke();
        ctx.setLineDash([]);
      }
      ctx.strokeStyle = "#ffffff"; ctx.lineWidth = 5;
      ctx.beginPath(); ctx.arc(x, y, 9, 0, Math.PI * 2); ctx.stroke();
      ctx.strokeStyle = "#17364a"; ctx.lineWidth = 2.5;
      ctx.beginPath(); ctx.arc(x, y, 9, 0, Math.PI * 2); ctx.stroke();
    }
  }

  function aircraftSeries(idx) {
    if (seriesCache && seriesCache.idx === idx && seriesCache.caseId === caseId) return seriesCache.rows;
    const radio = radioFor(caseId);
    const rows = [];
    rowsByFrame.forEach((frameRows, k) => {
      const row = frameRows.get(idx);
      if (!row) return;
      const link = radio[k].get(idx);
      rows.push({ t: frames[k].t, ...row, policy: policyAt(k, idx, row.policy), sinr: link.sinr, rsrp: link.rsrp, site: link.site });
    });
    seriesCache = { idx, caseId, rows };
    return rows;
  }

  // Link quality of the selected aircraft over its own flight, with the policy it produced.
  function drawLinkChart(selected) {
    const { ctx, width, height } = setupCanvas($("link-quality-chart"), 200);
    if (!selected) { ctx.fillStyle = "#64747c"; ctx.fillText("Select an aircraft on the map.", 14, 24); return; }
    const rows = aircraftSeries(selected.idx);
    const pad = { left: 46, right: 12, top: 34, bottom: 26 };
    const t0 = rows[0].t; const t1 = rows.at(-1).t;
    const threshold = recomputed ? currentParameters.thresholdDb : Number(parameters.threshold_db);
    const values = rows.map((row) => row.sinr).concat([threshold]);
    const min = Math.floor(Math.min(...values) - 1);
    const max = Math.ceil(Math.max(...values) + 1);
    const xAt = (t) => pad.left + (t - t0) / Math.max(1, t1 - t0) * (width - pad.left - pad.right);
    const yAt = (value) => pad.top + (max - value) / (max - min) * (height - pad.top - pad.bottom);

    // policy of the aircraft as a band under the curve
    for (let i = 0; i + 1 < rows.length; i += 1) {
      ctx.fillStyle = colors[rows[i].policy];
      ctx.globalAlpha = .18;
      ctx.fillRect(xAt(rows[i].t), pad.top, Math.max(1, xAt(rows[i + 1].t) - xAt(rows[i].t)), height - pad.top - pad.bottom);
      ctx.globalAlpha = 1;
    }
    ctx.strokeStyle = "#d7ddd9"; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(pad.left, pad.top); ctx.lineTo(pad.left, height - pad.bottom); ctx.lineTo(width - pad.right, height - pad.bottom); ctx.stroke();
    ctx.strokeStyle = "#d65353"; ctx.setLineDash([5, 4]);
    ctx.beginPath(); ctx.moveTo(pad.left, yAt(threshold)); ctx.lineTo(width - pad.right, yAt(threshold)); ctx.stroke(); ctx.setLineDash([]);
    ctx.fillStyle = "#d65353";
    ctx.fillText(`Θ ${fmt(threshold)} dB`, width - pad.right - 62, yAt(threshold) - 4);
    ctx.strokeStyle = "#168c85"; ctx.lineWidth = 2; ctx.beginPath();
    rows.forEach((row, i) => { const x = xAt(row.t); const y = yAt(row.sinr); if (i) ctx.lineTo(x, y); else ctx.moveTo(x, y); });
    ctx.stroke();
    ctx.strokeStyle = "#17364a"; ctx.setLineDash([3, 3]);
    ctx.beginPath(); ctx.moveTo(xAt(simulatedTime), pad.top); ctx.lineTo(xAt(simulatedTime), height - pad.bottom); ctx.stroke(); ctx.setLineDash([]);
    ctx.fillStyle = "#64747c";
    ctx.fillText("SINR (dB)", pad.left + 4, pad.top - 6);
    ctx.fillText(String(max), 6, pad.top + 8);
    ctx.fillText(String(min), 6, height - pad.bottom);
    ctx.fillText(formatTime(t0), pad.left, height - 8);
    ctx.fillText(formatTime(t1), width - pad.right - 32, height - 8);
    const switches = rows.reduce((total, row, i) => total + (i && row.policy !== rows[i - 1].policy ? 1 : 0), 0);
    $("link-current").textContent = `${aircraft[selected.idx].id} · ${switches} policy changes`;
  }

  function drawAircraftChart(selected) {
    const { ctx, width, height } = setupCanvas($("aircraft-chart"), 210);
    if (!selected) { ctx.fillStyle = "#64747c"; ctx.fillText("Select an aircraft on the map.", 14, 24); return; }
    const rows = aircraftSeries(selected.idx);
    const pad = { left: 46, right: 12 };
    const t0 = rows[0].t; const t1 = rows.at(-1).t;
    const xAt = (t) => pad.left + (t - t0) / Math.max(1, t1 - t0) * (width - pad.left - pad.right);
    const speedPanel = { top: 14, h: 64, min: parameters.speed_min_mps - 2, max: parameters.speed_max_mps + 2, label: "speed (m/s)" };
    const gapPanel = { top: 108, h: 74, min: 0, max: spacing.F * 1.35, label: "gap and spacing target (m, capped)" };
    [speedPanel, gapPanel].forEach((panel) => {
      panel.yAt = (value) => panel.top + (panel.max - Math.min(value, panel.max)) / (panel.max - panel.min) * panel.h;
      ctx.strokeStyle = "#d7ddd9"; ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(pad.left, panel.top); ctx.lineTo(pad.left, panel.top + panel.h); ctx.lineTo(width - pad.right, panel.top + panel.h); ctx.stroke();
      ctx.fillStyle = "#64747c";
      ctx.fillText(panel.label, pad.left + 4, panel.top - 3);
      ctx.fillText(fmt(panel.max, 0), 4, panel.top + 8);
      ctx.fillText(fmt(panel.min, 0), 4, panel.top + panel.h);
    });

    ctx.fillStyle = "#e9e9e6";
    let runStart = null;
    rows.forEach((row, i) => {
      if (row.gap < 0 && runStart === null) runStart = row.t;
      const ends = row.gap >= 0 || i === rows.length - 1;
      if (runStart !== null && ends) {
        ctx.fillRect(xAt(runStart), gapPanel.top, Math.max(1, xAt(row.t) - xAt(runStart)), gapPanel.h);
        runStart = null;
      }
    });

    ctx.strokeStyle = "#294f70"; ctx.lineWidth = 2; ctx.beginPath();
    rows.forEach((row, i) => { const x = xAt(row.t); const y = speedPanel.yAt(row.speed); if (i) ctx.lineTo(x, y); else ctx.moveTo(x, y); });
    ctx.stroke();

    for (let i = 0; i + 1 < rows.length; i += 1) {
      const level = gapPanel.yAt(spacing[rows[i].policy]);
      ctx.strokeStyle = colors[rows[i].policy]; ctx.lineWidth = 3;
      ctx.beginPath(); ctx.moveTo(xAt(rows[i].t), level); ctx.lineTo(xAt(rows[i + 1].t), level); ctx.stroke();
      if (rows[i + 1].policy !== rows[i].policy) {
        ctx.strokeStyle = "#9aa5a9"; ctx.lineWidth = 1.5;
        ctx.beginPath(); ctx.moveTo(xAt(rows[i + 1].t), level); ctx.lineTo(xAt(rows[i + 1].t), gapPanel.yAt(spacing[rows[i + 1].policy])); ctx.stroke();
      }
    }

    ctx.strokeStyle = "#17364a"; ctx.lineWidth = 1.6; ctx.beginPath();
    let open = false;
    rows.forEach((row) => {
      if (row.gap < 0) { open = false; return; }
      const x = xAt(row.t); const y = gapPanel.yAt(row.gap);
      if (open) ctx.lineTo(x, y); else { ctx.moveTo(x, y); open = true; }
    });
    ctx.stroke();

    ctx.strokeStyle = "#17364a"; ctx.setLineDash([3, 3]);
    ctx.beginPath(); ctx.moveTo(xAt(simulatedTime), 8); ctx.lineTo(xAt(simulatedTime), 186); ctx.stroke(); ctx.setLineDash([]);
    ctx.fillStyle = "#64747c";
    ctx.fillText(formatTime(t0), pad.left, 202);
    ctx.fillText(formatTime(t1), width - pad.right - 32, 202);
  }

  function drawPolicyShares() {
    const stats = active[caseId].stats;
    POLICIES.split("").forEach((policy) => {
      const share = Number(stats.policy_shares[policy] || 0);
      $(`share-${policy.toLowerCase()}`).textContent = `${fmt(100 * share)}%`;
      $(`share-${policy.toLowerCase()}-bar`).style.width = `${100 * share}%`;
    });
    const observations = active[caseId].observations || archived[caseId].observations;
    $("policy-observations").textContent = `${observations.toLocaleString()} aircraft-frames`;
    $("policy-description").textContent = recomputed
      ? `Share of aircraft-time in each policy, recomputed with Θ ${fmt(currentParameters.thresholdDb)} dB, ${fmt(currentParameters.windowS, 0)} s window, k = ${currentParameters.persistenceK}.`
      : `Share of aircraft-time in each policy in the archived run (Θ ${fmt(parameters.threshold_db)} dB, ${fmt(parameters.window_s, 0)} s window, k = ${parameters.persistence_k}).`;
  }

  function drawCapacity() {
    const { ctx, width, height } = setupCanvas($("capacity-chart"), 190);
    const pad = { left: 40, right: 12, top: 18, bottom: 24 };
    const all = Object.keys(data.cases).flatMap((id) => active[id].capacity.map((row) => row[1]));
    const tMax = Math.max(...Object.keys(data.cases).map((id) => active[id].capacity.at(-1)[0]));
    const minY = Math.floor(Math.min(...all) / 10) * 10; const maxY = Math.ceil(Math.max(...all) / 10) * 10;
    const xAt = (t) => pad.left + t / tMax * (width - pad.left - pad.right);
    const yAt = (v) => pad.top + (maxY - v) / (maxY - minY) * (height - pad.top - pad.bottom);
    const [lo, hi] = summary.compared_window_s;
    ctx.fillStyle = "#eef2f7"; ctx.fillRect(xAt(lo), pad.top, xAt(hi) - xAt(lo), height - pad.top - pad.bottom);
    ctx.fillStyle = "#17364a"; ctx.fillText("compared window", (xAt(lo) + xAt(hi)) / 2 - 40, 12);
    ctx.strokeStyle = "#d7ddd9"; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(pad.left, pad.top); ctx.lineTo(pad.left, height - pad.bottom); ctx.lineTo(width - pad.right, height - pad.bottom); ctx.stroke();
    Object.keys(data.cases).forEach((id) => {
      const current = id === caseId;
      ctx.strokeStyle = current ? "#168c85" : "#9aa5a9"; ctx.lineWidth = current ? 2 : 1.2;
      ctx.beginPath();
      active[id].capacity.forEach((row, i) => { const x = xAt(row[0]); const y = yAt(row[1]); if (i) ctx.lineTo(x, y); else ctx.moveTo(x, y); });
      ctx.stroke();
    });
    ctx.strokeStyle = "#17364a"; ctx.setLineDash([3, 3]);
    ctx.beginPath(); ctx.moveTo(xAt(simulatedTime), pad.top); ctx.lineTo(xAt(simulatedTime), height - pad.bottom); ctx.stroke(); ctx.setLineDash([]);
    ctx.fillStyle = "#64747c";
    ctx.fillText(String(maxY), 6, pad.top + 8); ctx.fillText(String(minY), 6, height - pad.bottom);
  }

  function renderCompareTable() {
    const ids = Object.keys(data.cases);
    const stat = (id) => active[id].stats;
    const pct = (value) => `${fmt(100 * value)}%`;
    const rows = [
      ["C / R / F time", (s) => `${pct(s.policy_shares.C)} / ${pct(s.policy_shares.R)} / ${pct(s.policy_shares.F)}`],
      ["Planning rate, mean", (s) => `${fmt(s.window_mean_uam_h)} UAM/h`],
      ["Planning rate, 95% reliable", (s) => `${fmt(s.window_q95_uam_h)} UAM/h`],
      ["Completed lane changes", (s) => String(s.completed_lane_changes)],
      ["Aircraft held at entry", (s) => String(s.held_at_entry)],
      ["Speed reversals per flight", (s) => fmt(s.speed_reversals_per_flight)],
      ["Completed / sampled NMAC", (s) => `${s.completed}/${s.scheduled} · ${s.sampled_nmac ? "yes" : "none"}`],
    ];
    const header = `<thead><tr><th></th>${ids.map((id) => `<th class="${id === caseId ? "motion-table__current" : ""}">${id === "spatial_grid" ? "Lane change" : "No change"}</th>`).join("")}</tr></thead>`;
    const body = rows.map(([label, value]) => `<tr><td>${label}</td>${ids.map((id) => `<td class="${id === caseId ? "motion-table__current" : ""}">${value(stat(id))}</td>`).join("")}</tr>`).join("");
    $("compare-table").innerHTML = header + `<tbody>${body}</tbody>`;
    const [lo, hi] = summary.compared_window_s;
    $("compare-window").textContent = `window ${fmt(lo / 60)}–${fmt(hi / 60)} min${recomputed ? " · recomputed" : ""}`;
  }

  // ------------------------------------------------------------------ frame update
  function select(idx) {
    setPlaying(false);
    selectedIdx = idx;
    seriesCache = null;
    cameraMode = "follow";
    annotateCamera();
    draw();
  }

  function draw() {
    const states = statesAt(simulatedTime);
    if (selectedIdx === null || !states.some((state) => state.idx === selectedIdx)) {
      const moving = states.find((state) => state.moving);
      selectedIdx = (moving || states[0] || { idx: null }).idx;
      seriesCache = null;
    }
    const selected = states.find((state) => state.idx === selectedIdx) || null;
    const link = selected ? engine.evaluateRadio(data.stations, summary.radio, { x_m: selected.x, y_m: selected.y }, selected.altitude) : null;
    update2d(states);
    update3d(states, selected, link);
    const frame = frames[index];
    const capacityRows = active[caseId].capacity;
    const capacityRow = capacityRows.reduce((best, row) => (Math.abs(row[0] - frame.t) < Math.abs(best[0] - frame.t) ? row : best), capacityRows[0]);
    $("time-slider").value = String(index);
    $("current-time").textContent = formatTime(simulatedTime);
    $("active-count").textContent = String(states.length);
    $("current-capacity").textContent = fmt(capacityRow[1]);
    $("current-counts").textContent = `C/R/F now · ${capacityRow[2]}/${capacityRow[3]}/${capacityRow[4]}`;
    const stats = active[caseId].stats;
    $("window-capacity").textContent = `${fmt(stats.window_mean_uam_h)} / ${fmt(stats.window_q95_uam_h)}`;
    const started = caseData.changes.filter((change) => change[0] <= simulatedTime + 1e-9).length;
    $("lane-changes").textContent = caseData.lane_change_allowed ? `${started} / ${caseData.changes.length}` : "none";
    $("lane-change-caption").textContent = caseData.lane_change_allowed ? "started so far / total" : "this case keeps every aircraft on the entry cell";
    const movingNow = states.filter((state) => state.moving).length;
    $("grid-note").textContent = caseData.lane_change_allowed ? `${movingNow} changing lane now` : "all on the entry cell";
    if (selected) {
      const record = aircraft[selected.idx];
      const change = activeChange(selected.idx, simulatedTime);
      $("selected-uam").textContent = record.id;
      $("selected-policy").textContent = selected.policy;
      $("selected-policy").style.color = colors[selected.policy];
      $("selected-cell").textContent = `${selected.offset > 0 ? "+" : ""}${fmt(selected.offset, 0)} m · ${fmt(selected.altitude, 0)} m`;
      $("selected-speed").textContent = `${fmt(selected.speed)} m/s`;
      $("selected-controller").textContent = controllerLabels[selected.controller] || selected.controller;
      $("selected-gap").textContent = selected.gap >= 0 ? `${fmt(selected.gap, 0)} m` : "no leader in group";
      $("selected-target").textContent = `${fmt(spacing[selected.policy], 0)} m (${selected.policy})`;
      $("selected-sinr").textContent = `${fmt(link.sinr)} dB · ${link.site.id}`;
      $("selected-rsrp").textContent = `${fmt(link.rsrp)} dBm/RE`;
      $("selected-progress").textContent = `${fmt(selected.q / 1000, 2)} km · ${fmt(100 * selected.q / corridorLengthM)}%`;
      $("selected-move").textContent = change
        ? `${cellLabel(change[2])} → ${cellLabel(change[3])} · ${fmt(simulatedTime - change[0], 0)} of ${fmt(change[4], 1)} s`
        : `none now · ${caseData.changes.filter((c) => c[1] === selected.idx).length} in this flight · entry delay ${fmt(record.entry_delay_s)} s`;
      servingLine.setLatLngs([[selected.lat, selected.lon], [link.site.lat, link.site.lon]]);
    } else {
      servingLine.setLatLngs([]);
      $("link-current").textContent = "—";
    }
    drawPolicyShares();
    drawGrid(states, selected);
    drawLinkChart(selected);
    drawAircraftChart(selected);
    drawCapacity();
  }

  function setPlaying(next) {
    playing = next;
    $("play-button").textContent = next ? "❚❚" : "▶";
    $("play-button").setAttribute("aria-label", next ? "Pause simulation" : "Play simulation");
    if (next) { lastTimestamp = null; rafId = requestAnimationFrame(animate); }
    else if (rafId !== null) { cancelAnimationFrame(rafId); rafId = null; }
  }

  let lastDraw = 0;
  function animate(timestamp) {
    if (!playing) return;
    if (lastTimestamp === null) lastTimestamp = timestamp;
    simulatedTime = Math.min(frames.at(-1).t, simulatedTime + (timestamp - lastTimestamp) / 1000 * Number($("playback-speed").value));
    lastTimestamp = timestamp;
    index = Math.max(0, Math.min(frames.length - 1, Math.floor((simulatedTime - frames[0].t) / frameS + 1e-9)));
    if (timestamp - lastDraw > 60 || simulatedTime >= frames.at(-1).t) { draw(); lastDraw = timestamp; }
    if (simulatedTime >= frames.at(-1).t) { setPlaying(false); return; }
    rafId = requestAnimationFrame(animate);
  }

  $("play-button").addEventListener("click", () => {
    if (!playing && simulatedTime >= frames.at(-1).t) { index = 0; simulatedTime = frames[0].t; }
    setPlaying(!playing);
  });
  $("reset-button").addEventListener("click", () => { setPlaying(false); index = 0; simulatedTime = frames[0].t; draw(); });
  $("time-slider").addEventListener("input", () => { setPlaying(false); index = Number($("time-slider").value); simulatedTime = frames[index].t; draw(); });
  $("case-select").addEventListener("change", () => {
    setPlaying(false);
    const t = simulatedTime;
    loadCase($("case-select").value);
    index = Math.max(0, Math.min(frames.length - 1, Math.round((t - frames[0].t) / frameS)));
    simulatedTime = frames[index].t;
    draw();
  });
  window.addEventListener("resize", () => draw());

  initialize3d();
  populateForm(baselineParameters);
  loadCase(caseId);
  const startIndex = Math.min(frames.length - 1, Math.round(1200 / frameS));
  index = startIndex;
  simulatedTime = frames[startIndex].t;
  ["reset-button", "play-button", "time-slider", "playback-speed"].forEach((id) => { $(id).disabled = false; });
  $("play-button").textContent = "▶";
  $("play-button").setAttribute("aria-label", "Play simulation");
  annotateCamera();
  draw();

  window.UAM_MOTION_QA = {
    state: () => ({
      case: caseId, t: simulatedTime, frame: index, active: statesAt(simulatedTime).length,
      selected: selectedIdx === null ? null : aircraft[selectedIdx].id, engine: viewer3d ? "cesium" : "none",
      markers: markers.size, recomputed,
      shares: active[caseId].stats.policy_shares,
      window: [active[caseId].stats.window_mean_uam_h, active[caseId].stats.window_q95_uam_h],
    }),
    setCase: (id) => { $("case-select").value = id; $("case-select").dispatchEvent(new Event("change")); },
    seek: (t) => { setPlaying(false); simulatedTime = t; index = Math.max(0, Math.min(frames.length - 1, Math.floor((t - frames[0].t) / frameS))); draw(); },
    select: (id) => { const found = aircraft.findIndex((row) => row.id === id); if (found >= 0) select(found); return found; },
    recompute: (overrides = {}) => {
      populateForm({ ...currentParameters, ...overrides });
      $("experiment-form").dispatchEvent(new Event("submit"));
    },
    reset: () => $("reset-experiment").click(),
  };
})();
