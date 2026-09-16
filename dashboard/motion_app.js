(() => {
  "use strict";

  const data = window.UAM_MOTION_DATA;
  const engine = window.UAM_TRAFFIC_ENGINE;
  const errorPanel = document.getElementById("error-panel");
  if (!data || !window.L || !engine) {
    errorPanel.hidden = false;
    errorPanel.textContent = !data
      ? "Motion data bundle is missing. Rebuild it with scripts/build_motion_dashboard.py."
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
  const controllerLabels = {
    cruise: "Cruise · no binding leader",
    ks2_tracker: "AKS reference tracking",
    feedback_fallback: "ACC feedback spacing",
  };
  const $ = (id) => document.getElementById(id);
  const fmt = (value, digits = 1) => Number(value).toFixed(digits);

  function formatTime(seconds) {
    const value = Math.max(0, Math.round(seconds));
    return `${String(Math.floor(value / 60)).padStart(2, "0")}:${String(value % 60).padStart(2, "0")}`;
  }

  // ------------------------------------------------------------------ settings
  const spacing = summary.spacing_m;
  const settings = [
    ["SINR threshold Θ", `${fmt(parameters.threshold_db)} dB`],
    ["Radio sampling", `${fmt(parameters.radio_s, 0)} s`],
    ["Policy update", `${fmt(parameters.policy_s, 0)} s`],
    ["Assessment window", `${fmt(parameters.window_s, 0)} s`],
    ["Persistence k", String(parameters.persistence_k)],
    ["Exposure C / R", `${fmt(100 * parameters.exposure_c, 0)}% / ${fmt(100 * parameters.exposure_r, 0)}%`],
    ["Exposure group", `${parameters.group_mode} · 5 aircraft`],
    ["Spacing C / R / F", `${fmt(spacing.C, 0)} / ${fmt(spacing.R, 0)} / ${fmt(spacing.F, 0)} m`],
    ["Spacing law", `${fmt(parameters.d0_m)} + τ·v + ${parameters.buffer_s2_per_m}·v²`],
    ["Speed", `${fmt(parameters.speed_min_mps, 0)}–${fmt(parameters.speed_max_mps, 0)} m/s`],
    ["Longitudinal accel.", `+${fmt(parameters.acceleration_limit_mps2)} / −${fmt(parameters.deceleration_limit_mps2)} m/s²`],
    ["Lane-change accel.", `${fmt(summary.envelope.lateral_accel_max_mps2, 3)} m/s² (0.1 g)`],
    ["Demand", `1 request / ${fmt(summary.global_headway_s, 0)} s · ${summary.requests} requests`],
  ];
  $("settings-list").replaceChildren(...settings.map(([label, value]) => {
    const item = document.createElement("div");
    item.innerHTML = `<dt>${label}</dt><dd>${value}</dd>`;
    return item;
  }));
  $("run-note").textContent = `Run ${summary.run_id} · validation ${summary.validation} · ${summary.display.route_label || summary.scenario_id}`;
  $("footer-run").textContent = `Precomputed from verified run ${summary.run_id} · frames every ${fmt(frameS, 0)} s, interpolated for playback`;

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

  function decodeRow(row) {
    return {
      idx: row[field.aircraft], lat: row[field.lat_e5] / 1e5, lon: row[field.lon_e5] / 1e5,
      x: row[field.x_m], y: row[field.y_m], altitude: row[field.altitude_m], offset: row[field.offset_m],
      speed: row[field.speed_dmps] / 10, policy: POLICIES[row[field.policy]],
      controller: summary.controllers[row[field.controller]], gap: row[field.gap_m],
      moving: row[field.moving] === 1, cell: row[field.cell],
    };
  }

  function loadCase(nextCase) {
    caseId = nextCase;
    caseData = data.cases[caseId];
    frames = caseData.frames;
    aircraft = caseData.aircraft;
    rowsByFrame = frames.map((frame) => new Map(frame.rows.map((row) => [row[field.aircraft], decodeRow(row)])));
    index = 0;
    simulatedTime = frames[0].t;
    seriesCache = null;
    if (selectedIdx !== null && !rowsByFrame.some((rows) => rows.has(selectedIdx))) selectedIdx = null;
    $("case-select").value = caseId;
    $("time-slider").max = String(frames.length - 1);
    $("total-time").textContent = `/ ${formatTime(frames.at(-1).t)}`;
    const stats = caseData.stats;
    $("window-capacity").textContent = `${fmt(stats.window_mean_uam_h)} / ${fmt(stats.window_q95_uam_h)}`;
    const [lo, hi] = summary.compared_window_s;
    $("window-caption").textContent = `mean / 95% reliable · ${fmt(lo / 60)}–${fmt(hi / 60)} min`;
    renderCompareTable();
    clearAircraftLayers();
    const url = new URL(window.location.href);
    url.searchParams.set("case", caseId);
    window.history.replaceState(null, "", url);
  }

  // interpolated aircraft states at simulated time t
  function statesAt(t) {
    const k = Math.max(0, Math.min(frames.length - 1, Math.floor((t - frames[0].t) / frameS + 1e-9)));
    const a = rowsByFrame[k];
    const b = rowsByFrame[Math.min(frames.length - 1, k + 1)];
    const span = k + 1 < frames.length ? frames[k + 1].t - frames[k].t : 1;
    const f = Math.max(0, Math.min(1, (t - frames[k].t) / span));
    const out = [];
    for (const [idx, row] of a) {
      const next = b.get(idx);
      if (!next || f === 0) { out.push(row); continue; }
      out.push({
        ...row,
        lat: row.lat + f * (next.lat - row.lat), lon: row.lon + f * (next.lon - row.lon),
        x: row.x + f * (next.x - row.x), y: row.y + f * (next.y - row.y),
        altitude: row.altitude + f * (next.altitude - row.altitude), offset: row.offset + f * (next.offset - row.offset),
        speed: row.speed + f * (next.speed - row.speed),
      });
    }
    return out;
  }

  function activeChange(idx, t) {
    return caseData.changes.find((change) => change[1] === idx && change[0] <= t + 1e-9 && t < change[0] + change[4]);
  }

  function cellLabel(cellIndex) {
    const cell = summary.grid[cellIndex];
    return `${cell.offset_m > 0 ? "+" : ""}${fmt(cell.offset_m, 0)} m · ${fmt(cell.altitude_m, 0)} m`;
  }

  function progressM(state) {
    let best = null;
    for (let i = 0; i + 1 < route.length; i += 1) {
      const a = route[i]; const b = route[i + 1];
      const dx = b.x_m - a.x_m; const dy = b.y_m - a.y_m;
      const len2 = dx * dx + dy * dy || 1;
      const u = Math.max(0, Math.min(1, ((state.x - a.x_m) * dx + (state.y - a.y_m) * dy) / len2));
      const d2 = (a.x_m + u * dx - state.x) ** 2 + (a.y_m + u * dy - state.y) ** 2;
      if (!best || d2 < best.d2) best = { d2, s: a.s_m + u * Math.sqrt(len2) };
    }
    return best ? best.s : 0;
  }

  // ------------------------------------------------------------------ map
  const map = L.map("motion-map", { preferCanvas: true });
  L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", { maxZoom: 19, attribution: "&copy; OpenStreetMap contributors" }).addTo(map);
  const laneLines = offsets.map((offset) => L.polyline(
    route.map((row) => { const p = engine.interpolateRoute(route, corridorLengthM, row.s_m, offset); return [p.lat, p.lon]; }),
    offset === 0 ? { color: "#168c85", weight: 4, opacity: .85 } : { color: "#607177", weight: 2, opacity: .7, dashArray: "6 6" },
  ).addTo(map));
  // The pane can still be laying out when the script runs, so keep fitting the corridor until the
  // container settles. Any pointer or wheel gesture on the map hands control to the viewer.
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

  function markerIcon(state, selected) {
    const classes = ["motion-ac"];
    if (state.moving) classes.push("motion-ac--moving");
    if (selected) classes.push("motion-ac--selected");
    return L.divIcon({
      className: "",
      html: `<div class="${classes.join(" ")}" style="background:${colors[state.policy]}">${Math.round(state.altitude / 100)}</div>`,
      iconSize: selected ? [28, 28] : [20, 20], iconAnchor: selected ? [14, 14] : [10, 10],
    });
  }

  function update2d(states) {
    const active = new Set(states.map((state) => state.idx));
    for (const [idx, marker] of markers) {
      if (!active.has(idx)) { map.removeLayer(marker); markers.delete(idx); }
    }
    states.forEach((state) => {
      const selected = state.idx === selectedIdx;
      const key = `${state.policy}${state.moving}${selected}${Math.round(state.altitude / 100)}`;
      let marker = markers.get(state.idx);
      if (!marker) {
        marker = L.marker([state.lat, state.lon], { title: `Select ${aircraft[state.idx].id}`, keyboard: true })
          .on("click", () => select(state.idx)).addTo(map);
        markers.set(state.idx, marker);
      }
      marker.setLatLng([state.lat, state.lon]);
      if (marker._motionKey !== key) {
        marker.setIcon(markerIcon(state, selected));
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
      const positions = route.flatMap((row) => { const p = engine.interpolateRoute(route, corridorLengthM, row.s_m, offset); return [p.lon, p.lat, altitude]; });
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
    const active = new Set(states.map((state) => state.idx));
    for (const [idx, entity] of aircraft3d) {
      if (!active.has(idx)) { viewer3d.entities.remove(entity); aircraft3d.delete(idx); }
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

  function drawGrid(states, selected) {
    const { ctx, width, height } = setupCanvas($("grid-chart"), 240);
    const pad = { left: 52, right: 14, top: 14, bottom: 30 };
    const cellW = (width - pad.left - pad.right) / offsets.length;
    const cellH = (height - pad.top - pad.bottom) / altitudes.length;
    const xAt = (offset) => pad.left + (offsets.indexOf(offset) + .5) * cellW;
    const xOf = (offset) => pad.left + ((offset - offsets[0]) / (offsets.at(-1) - offsets[0]) * (offsets.length - 1) + .5) * cellW;
    const yOf = (altitude) => pad.top + ((altitudes.at(-1) - altitude) / (altitudes.at(-1) - altitudes[0]) * (altitudes.length - 1) + .5) * cellH;
    const counts = summary.grid.map(() => ({ C: 0, R: 0, F: 0 }));
    states.forEach((state) => { counts[state.cell][state.policy] += 1; });
    summary.grid.forEach((cell, cellIndex) => {
      const cx = xAt(cell.offset_m); const cy = yOf(cell.altitude_m);
      const total = counts[cellIndex].C + counts[cellIndex].R + counts[cellIndex].F;
      const entry = cellIndex === summary.entry_flow_index;
      ctx.fillStyle = total ? "#eef4f1" : "#f7f7f5";
      ctx.strokeStyle = entry ? "#17364a" : "#d7ddd9";
      ctx.lineWidth = entry ? 2 : 1;
      ctx.fillRect(cx - cellW / 2 + 4, cy - cellH / 2 + 4, cellW - 8, cellH - 8);
      ctx.strokeRect(cx - cellW / 2 + 4, cy - cellH / 2 + 4, cellW - 8, cellH - 8);
      let dot = 0;
      POLICIES.split("").forEach((policy) => {
        for (let n = 0; n < counts[cellIndex][policy] && dot < 18; n += 1, dot += 1) {
          ctx.fillStyle = colors[policy];
          ctx.beginPath();
          ctx.arc(cx - cellW / 2 + 16 + (dot % 6) * 11, cy - cellH / 2 + 18 + Math.floor(dot / 6) * 11, 4, 0, Math.PI * 2);
          ctx.fill();
        }
      });
      ctx.fillStyle = total ? "#17364a" : "#9aa5a9";
      ctx.font = "700 13px system-ui";
      ctx.fillText(String(total), cx + cellW / 2 - 24, cy + cellH / 2 - 12);
      ctx.font = "10px system-ui";
      if (entry) { ctx.fillStyle = "#17364a"; ctx.fillText("entry", cx - cellW / 2 + 10, cy + cellH / 2 - 12); }
    });
    ctx.fillStyle = "#64747c";
    offsets.forEach((offset) => ctx.fillText(`${offset > 0 ? "+" : ""}${offset} m`, xAt(offset) - 16, height - 10));
    altitudes.forEach((altitude) => ctx.fillText(`${altitude} m`, 6, yOf(altitude) + 3));
    if (selected) {
      ctx.strokeStyle = "#17364a"; ctx.lineWidth = 3;
      ctx.beginPath(); ctx.arc(xOf(selected.offset), yOf(selected.altitude), 9, 0, Math.PI * 2); ctx.stroke();
      const change = activeChange(selected.idx, simulatedTime);
      if (change) {
        const target = summary.grid[change[3]];
        ctx.setLineDash([4, 3]);
        ctx.beginPath(); ctx.moveTo(xOf(selected.offset), yOf(selected.altitude)); ctx.lineTo(xAt(target.offset_m), yOf(target.altitude_m)); ctx.stroke();
        ctx.setLineDash([]);
      }
    }
  }

  function aircraftSeries(idx) {
    if (seriesCache && seriesCache.idx === idx && seriesCache.caseId === caseId) return seriesCache.rows;
    const rows = [];
    rowsByFrame.forEach((frameRows, k) => { const row = frameRows.get(idx); if (row) rows.push({ t: frames[k].t, ...row }); });
    seriesCache = { idx, caseId, rows };
    return rows;
  }

  function drawAircraftChart(selected) {
    const { ctx, width, height } = setupCanvas($("aircraft-chart"), 230);
    if (!selected) { ctx.fillStyle = "#64747c"; ctx.fillText("Select an aircraft on the map.", 14, 24); return; }
    const rows = aircraftSeries(selected.idx);
    const pad = { left: 46, right: 12 };
    const t0 = rows[0].t; const t1 = rows.at(-1).t;
    const xAt = (t) => pad.left + (t - t0) / Math.max(1, t1 - t0) * (width - pad.left - pad.right);
    const panels = [
      { top: 14, h: 72, min: parameters.speed_min_mps - 2, max: parameters.speed_max_mps + 2, label: "speed (m/s)" },
      { top: 116, h: 84, min: 0, max: spacing.F * 1.35, label: "gap to leader vs spacing target (m, capped)" },
    ];
    panels.forEach((panel) => {
      const yAt = (value) => panel.top + (panel.max - value) / (panel.max - panel.min) * panel.h;
      panel.yAt = yAt;
      ctx.strokeStyle = "#d7ddd9"; ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(pad.left, panel.top); ctx.lineTo(pad.left, panel.top + panel.h); ctx.lineTo(width - pad.right, panel.top + panel.h); ctx.stroke();
      ctx.fillStyle = "#64747c";
      ctx.fillText(panel.label, pad.left + 4, panel.top - 3);
      ctx.fillText(fmt(panel.max, 0), 4, panel.top + 8);
      ctx.fillText(fmt(panel.min, 0), 4, panel.top + panel.h);
    });
    const [speedPanel, gapPanel] = panels;
    ctx.strokeStyle = "#294f70"; ctx.lineWidth = 2; ctx.beginPath();
    rows.forEach((row, i) => { const x = xAt(row.t); const y = speedPanel.yAt(row.speed); if (i) ctx.lineTo(x, y); else ctx.moveTo(x, y); });
    ctx.stroke();
    for (let i = 0; i + 1 < rows.length; i += 1) {
      ctx.strokeStyle = colors[rows[i].policy]; ctx.lineWidth = 3;
      ctx.beginPath(); ctx.moveTo(xAt(rows[i].t), gapPanel.yAt(spacing[rows[i].policy])); ctx.lineTo(xAt(rows[i + 1].t), gapPanel.yAt(spacing[rows[i].policy])); ctx.stroke();
    }
    ctx.strokeStyle = "#17364a"; ctx.lineWidth = 1.6; ctx.beginPath();
    let open = false;
    rows.forEach((row) => {
      if (row.gap < 0) { open = false; return; }
      const x = xAt(row.t); const y = gapPanel.yAt(Math.min(row.gap, gapPanel.max));
      if (open) ctx.lineTo(x, y); else { ctx.moveTo(x, y); open = true; }
    });
    ctx.stroke();
    ctx.strokeStyle = "#17364a"; ctx.setLineDash([3, 3]);
    ctx.beginPath(); ctx.moveTo(xAt(simulatedTime), 8); ctx.lineTo(xAt(simulatedTime), 204); ctx.stroke(); ctx.setLineDash([]);
    ctx.fillStyle = "#64747c";
    ctx.fillText(formatTime(t0), pad.left, 222);
    ctx.fillText(formatTime(t1), width - pad.right - 32, 222);
    ctx.fillText("thick = target S(policy) · thin = actual gap (blank when no leader in the group)", pad.left + 60, 222);
  }

  function drawCapacity() {
    const { ctx, width, height } = setupCanvas($("capacity-chart"), 190);
    const pad = { left: 40, right: 12, top: 18, bottom: 24 };
    const all = Object.values(data.cases).flatMap((c) => c.capacity.map((row) => row[1]));
    const tMax = Math.max(...Object.values(data.cases).map((c) => c.capacity.at(-1)[0]));
    const minY = Math.floor(Math.min(...all) / 10) * 10; const maxY = Math.ceil(Math.max(...all) / 10) * 10;
    const xAt = (t) => pad.left + t / tMax * (width - pad.left - pad.right);
    const yAt = (v) => pad.top + (maxY - v) / (maxY - minY) * (height - pad.top - pad.bottom);
    const [lo, hi] = summary.compared_window_s;
    ctx.fillStyle = "#eef2f7"; ctx.fillRect(xAt(lo), pad.top, xAt(hi) - xAt(lo), height - pad.top - pad.bottom);
    ctx.fillStyle = "#17364a"; ctx.fillText("compared window", (xAt(lo) + xAt(hi)) / 2 - 40, 12);
    ctx.strokeStyle = "#d7ddd9"; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(pad.left, pad.top); ctx.lineTo(pad.left, height - pad.bottom); ctx.lineTo(width - pad.right, height - pad.bottom); ctx.stroke();
    Object.entries(data.cases).forEach(([id, c]) => {
      const current = id === caseId;
      ctx.strokeStyle = current ? "#168c85" : "#9aa5a9"; ctx.lineWidth = current ? 2 : 1.2;
      ctx.beginPath();
      c.capacity.forEach((row, i) => { const x = xAt(row[0]); const y = yAt(row[1]); if (i) ctx.lineTo(x, y); else ctx.moveTo(x, y); });
      ctx.stroke();
    });
    ctx.strokeStyle = "#17364a"; ctx.setLineDash([3, 3]);
    ctx.beginPath(); ctx.moveTo(xAt(simulatedTime), pad.top); ctx.lineTo(xAt(simulatedTime), height - pad.bottom); ctx.stroke(); ctx.setLineDash([]);
    ctx.fillStyle = "#64747c";
    ctx.fillText(String(maxY), 6, pad.top + 8); ctx.fillText(String(minY), 6, height - pad.bottom);
    ctx.fillText(`teal = ${data.cases[caseId].label.split(" · ")[0].toLowerCase()} · grey = other case · UAM/h`, pad.left + 6, height - 6);
  }

  function renderCompareTable() {
    const ids = Object.keys(data.cases);
    const stat = (id) => data.cases[id].stats;
    const pct = (value) => `${fmt(100 * value)}%`;
    const rows = [
      ["C / R / F time", (s) => `${pct(s.policy_shares.C)} / ${pct(s.policy_shares.R)} / ${pct(s.policy_shares.F)}`],
      ["Planning rate, mean", (s) => `${fmt(s.window_mean_uam_h)} UAM/h`],
      ["Planning rate, 95% reliable", (s) => `${fmt(s.window_q95_uam_h)} UAM/h`],
      ["Completed lane changes", (s) => String(s.completed_lane_changes)],
      ["Aircraft held at entry", (s) => String(s.held_at_entry)],
      ["Longest entry hold", (s) => `${fmt(s.longest_entry_hold_s)} s`],
      ["Speed reversals per flight", (s) => fmt(s.speed_reversals_per_flight)],
      ["Last aircraft out", (s) => `${fmt(s.end_s / 60)} min`],
      ["Completed / sampled NMAC", (s) => `${s.completed}/${s.scheduled} · ${s.sampled_nmac ? "yes" : "none"}`],
    ];
    const header = `<thead><tr><th></th>${ids.map((id) => `<th class="${id === caseId ? "motion-table__current" : ""}">${id === "spatial_grid" ? "Move 3 × 3" : "Stay"}</th>`).join("")}</tr></thead>`;
    const body = rows.map(([label, value]) => `<tr><td>${label}</td>${ids.map((id) => `<td class="${id === caseId ? "motion-table__current" : ""}">${value(stat(id))}</td>`).join("")}</tr>`).join("");
    $("compare-table").innerHTML = header + `<tbody>${body}</tbody>`;
    const [lo, hi] = summary.compared_window_s;
    $("compare-window").textContent = `window ${fmt(lo / 60)}–${fmt(hi / 60)} min`;
  }

  // ------------------------------------------------------------------ frame update
  function select(idx) {
    setPlaying(false);
    selectedIdx = idx;
    cameraMode = "follow";
    annotateCamera();
    draw();
  }

  function draw() {
    const states = statesAt(simulatedTime);
    if (selectedIdx === null || !states.some((state) => state.idx === selectedIdx)) {
      const moving = states.find((state) => state.moving);
      selectedIdx = (moving || states[0] || { idx: null }).idx;
    }
    const selected = states.find((state) => state.idx === selectedIdx) || null;
    const link = selected ? engine.evaluateRadio(data.stations, summary.radio, { x_m: selected.x, y_m: selected.y }, selected.altitude) : null;
    update2d(states);
    update3d(states, selected, link);
    const frame = frames[index];
    const capacityRow = caseData.capacity.reduce((best, row) => (Math.abs(row[0] - frame.t) < Math.abs(best[0] - frame.t) ? row : best), caseData.capacity[0]);
    $("time-slider").value = String(index);
    $("current-time").textContent = formatTime(simulatedTime);
    $("active-count").textContent = String(states.length);
    $("current-capacity").textContent = fmt(capacityRow[1]);
    $("current-counts").textContent = `C/R/F now · ${capacityRow[2]}/${capacityRow[3]}/${capacityRow[4]}`;
    const started = caseData.changes.filter((change) => change[0] <= simulatedTime + 1e-9).length;
    $("lane-changes").textContent = caseData.lane_change_allowed ? `${started} / ${caseData.changes.length}` : "off";
    $("lane-change-caption").textContent = caseData.lane_change_allowed ? "started so far / total" : "stay case: aircraft keep the centre cell";
    const movingNow = states.filter((state) => state.moving).length;
    $("grid-note").textContent = caseData.lane_change_allowed ? `${movingNow} changing lane now` : "all at the entry cell";
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
      const s = progressM(selected);
      $("selected-progress").textContent = `${fmt(s / 1000, 2)} km · ${fmt(100 * s / corridorLengthM)}%`;
      $("selected-move").textContent = change
        ? `${cellLabel(change[2])} → ${cellLabel(change[3])} · ${fmt(simulatedTime - change[0], 0)} of ${fmt(change[4], 1)} s`
        : `none now · ${caseData.changes.filter((c) => c[1] === selected.idx).length} in this flight · entry delay ${fmt(record.entry_delay_s)} s`;
      $("aircraft-note").textContent = record.id;
      servingLine.setLatLngs([[selected.lat, selected.lon], [link.site.lat, link.site.lon]]);
    } else {
      servingLine.setLatLngs([]);
    }
    drawGrid(states, selected);
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
      markers: markers.size,
    }),
    setCase: (id) => { $("case-select").value = id; $("case-select").dispatchEvent(new Event("change")); },
    seek: (t) => { setPlaying(false); simulatedTime = t; index = Math.max(0, Math.min(frames.length - 1, Math.floor((t - frames[0].t) / frameS))); draw(); },
  };
})();
