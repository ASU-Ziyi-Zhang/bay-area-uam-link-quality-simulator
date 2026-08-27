(() => {
  "use strict";

  const data = window.UAM_TRAFFIC_DATA;
  const engine = window.UAM_TRAFFIC_ENGINE;
  const errorPanel = document.getElementById("error-panel");
  if (!data || !window.L || !engine) {
    errorPanel.hidden = false;
    errorPanel.textContent = !data
      ? "Traffic data bundle is missing. Rebuild it with scripts/build_traffic_dashboard.py."
      : !window.L ? "The bundled Leaflet map library did not load." : "The deterministic traffic engine did not load.";
    return;
  }

  const colors = { C: "#238b57", R: "#e3b735", F: "#d65353" };
  let entrants = [...data.entrants].sort((left, right) => left.index - right.index);
  function expandFrame(frame) {
    if (frame.policies) return frame;
    const activeIds = entrants
      .filter((uam) => uam.entry <= frame.t + 1e-9 && uam.exit >= frame.t - 1e-9)
      .map((uam) => uam.id);
    if (activeIds.length !== frame.policy_codes.length || activeIds.length !== frame.exposure_values.length) {
      throw new Error(`Compact traffic frame mismatch at ${frame.t}s`);
    }
    const policies = Object.fromEntries(activeIds.map((uamId, index) => [uamId, frame.policy_codes[index]]));
    const exposure = Object.fromEntries(activeIds.map((uamId, index) => [uamId, frame.exposure_values[index]]));
    const groups = Object.fromEntries(activeIds.map((uamId, index) => [
      uamId,
      activeIds.slice(Math.max(0, index - 2), Math.min(activeIds.length, index + 3)),
    ]));
    return { ...frame, policies, exposure, groups };
  }
  let frames = data.frames.map(expandFrame);
  const route = data.route_metric;
  const summary = data.summary;
  const display = summary.display || {};
  let speedMps = Number(summary.trajectory.speed_mps);
  let altitudeM = Number(summary.trajectory.altitude_m);
  let lateralOffsetM = Number(summary.trajectory.lateral_offset_m || 0);
  const corridorLengthM = Number(summary.corridor_length_km) * 1000;
  const stationById = new Map(data.stations.map((site) => [site.id, site]));
  const baselineParameters = {
    speedMps,
    altitudeM,
    lateralOffsetM,
    departureIntervalS: Number(summary.traffic.entry_interval_s),
    sinrThresholdDb: Number(summary.policy.sinr_threshold_db),
    groupSize: Number(summary.policy.maximum_group_size),
    exposureWindowS: Number(summary.policy.window_s),
    policyIntervalS: Number(summary.clock.dt_control_s),
    coordinatedTolerance: Number(summary.policy.coordinated_exposure_tolerance),
    reactiveTolerance: Number(summary.policy.reactive_exposure_tolerance),
    reliabilityRho: Number(summary.capacity.reliability_rho),
  };
  let currentParameters = { ...baselineParameters };
  let currentResults = {
    offeredDemand: Number(summary.traffic.entry_demand_uam_h),
    transitTimeS: Number(summary.transit_time_s),
    expectedOccupancy: Number(summary.traffic.expected_steady_occupancy),
    reliabilityCapacity: Number(summary.capacity.q_mix_rho_uam_h),
    demandSupported: Boolean(summary.capacity.demand_supported_by_q_mix_rho),
    observationCount: Number(summary.policy.observation_count),
    policyShares: { ...summary.policy.shares },
  };

  const ids = [
    "traffic-title", "traffic-subtitle",
    "play-button", "reset-button", "time-slider", "playback-speed", "current-time", "total-time",
    "active-count", "expected-count", "current-capacity", "reliability-capacity", "demand-status",
    "policy-observations", "share-c", "share-r", "share-f", "share-c-bar", "share-r-bar", "share-f-bar",
    "selected-uam", "selected-policy", "selected-exposure", "selected-site", "selected-rsrp", "selected-sinr", "selected-progress", "selected-group",
    "current-counts", "link-current", "three-warning", "three-free-view", "three-recenter", "three-camera-state", "three-camera-note",
    "reliability-caption", "demand-caption", "policy-description", "flight-altitude-legend", "footer-policy-description",
    "experiment-form", "experiment-status", "run-experiment", "reset-experiment", "input-altitude", "input-offset",
    "input-speed", "input-departure", "input-theta", "input-group-size", "input-window", "input-policy-interval",
    "input-c-tolerance", "input-r-tolerance", "input-reliability",
  ];
  const el = Object.fromEntries(ids.map((id) => [id, document.getElementById(id)]));
  const capacityCanvas = document.getElementById("capacity-chart");
  const linkCanvas = document.getElementById("link-quality-chart");

  el["traffic-title"].textContent = `${display.route_label || summary.scenario_id} · Multi-UAM Policy`;
  el["traffic-subtitle"].textContent = "Individual link quality → available local-neighbor group policy → reliability-qualified capacity";
  function populateParameterForm(parameters) {
    el["input-altitude"].value = String(parameters.altitudeM);
    el["input-offset"].value = String(parameters.lateralOffsetM);
    el["input-speed"].value = String(parameters.speedMps);
    el["input-departure"].value = String(parameters.departureIntervalS);
    el["input-theta"].value = String(parameters.sinrThresholdDb);
    el["input-group-size"].value = String(parameters.groupSize);
    el["input-window"].value = String(parameters.exposureWindowS);
    el["input-policy-interval"].value = String(parameters.policyIntervalS);
    el["input-c-tolerance"].value = String(100 * parameters.coordinatedTolerance);
    el["input-r-tolerance"].value = String(100 * parameters.reactiveTolerance);
    el["input-reliability"].value = String(100 * parameters.reliabilityRho);
  }

  function readParameterForm() {
    return {
      altitudeM: Number(el["input-altitude"].value),
      lateralOffsetM: Number(el["input-offset"].value),
      speedMps: Number(el["input-speed"].value),
      departureIntervalS: Number(el["input-departure"].value),
      sinrThresholdDb: Number(el["input-theta"].value),
      groupSize: Number(el["input-group-size"].value),
      exposureWindowS: Number(el["input-window"].value),
      policyIntervalS: Number(el["input-policy-interval"].value),
      coordinatedTolerance: Number(el["input-c-tolerance"].value) / 100,
      reactiveTolerance: Number(el["input-r-tolerance"].value) / 100,
      reliabilityRho: Number(el["input-reliability"].value) / 100,
    };
  }

  function updateScenarioSummary(results, parameters) {
    el["expected-count"].textContent = `steady-state expectation ${results.expectedOccupancy.toFixed(1)}`;
    el["reliability-capacity"].textContent = results.reliabilityCapacity.toFixed(1);
    const tailPercent = 100 * (1 - parameters.reliabilityRho);
    const qLabel = `Q${parameters.reliabilityRho.toFixed(2)}`;
    el["reliability-caption"].innerHTML = `${qLabel} · lower ${tailPercent.toFixed(0)}th percentile`;
    el["demand-caption"].innerHTML = `offered demand ≤ ${qLabel}`;
    el["policy-observations"].textContent = `${results.observationCount.toLocaleString()} observations`;
    ["C", "R", "F"].forEach((policy) => {
      const share = Number(results.policyShares[policy] || 0);
      el[`share-${policy.toLowerCase()}`].textContent = `${(100 * share).toFixed(1)}%`;
      el[`share-${policy.toLowerCase()}-bar`].style.width = `${100 * share}%`;
    });
    el["demand-status"].textContent = results.demandSupported ? "Yes" : "No";
    el["demand-status"].className = results.demandSupported ? "supported" : "unsupported";
    const half = Math.floor(parameters.groupSize / 2);
    el["policy-description"].textContent = `Fractions use all active-aircraft time observations. Each aircraft uses up to ${half} ahead and ${half} behind (maximum group ${parameters.groupSize}), updated every ${parameters.policyIntervalS.toFixed(0)} s.`;
    el["flight-altitude-legend"].textContent = `${parameters.altitudeM.toFixed(0)} m flight path`;
    el["footer-policy-description"].textContent = `Individual radio · overlapping maximum-${parameters.groupSize}-UAM group policy · deterministic planning capacity`;
  }

  populateParameterForm(baselineParameters);
  updateScenarioSummary(currentResults, currentParameters);

  function formatTime(seconds) {
    const value = Math.max(0, Math.round(seconds));
    const minutes = Math.floor(value / 60);
    return `${String(minutes).padStart(2, "0")}:${String(value % 60).padStart(2, "0")}`;
  }

  function interpolateRoute(sM) {
    return engine.interpolateRoute(route, corridorLengthM, sM, lateralOffsetM);
  }

  function evaluateRadio(position) {
    return engine.evaluateRadio(data.stations, summary.radio, position, altitudeM);
  }

  function buildLinkProfile() {
    return Array.from({ length: 241 }, (_unused, sampleIndex) => {
      const sM = corridorLengthM * sampleIndex / 240;
      const link = evaluateRadio(interpolateRoute(sM));
      return { sM, rsrp: link.rsrp, sinr: link.sinr, siteId: link.site.id };
    });
  }
  let linkProfile = buildLinkProfile();

  function drawLinkQualityChart(selected) {
    const ratio = window.devicePixelRatio || 1;
    const width = Math.max(320, linkCanvas.clientWidth);
    const height = 220;
    linkCanvas.width = width * ratio;
    linkCanvas.height = height * ratio;
    const ctx = linkCanvas.getContext("2d");
    ctx.scale(ratio, ratio);
    ctx.clearRect(0, 0, width, height);
    const pad = { left: 44, right: 12 };
    const plotWidth = width - pad.left - pad.right;
    const xAt = (sM) => pad.left + sM / corridorLengthM * plotWidth;
    const cursorS = selected ? selected.sM : 0;

    const panels = [
      {
        top: 13, height: 78, key: "rsrp", color: "#294f70", label: "RSRP (dBm/RE)",
        min: Math.floor(Math.min(...linkProfile.map((row) => row.rsrp)) / 5) * 5 - 5,
        max: Math.ceil(Math.max(...linkProfile.map((row) => row.rsrp)) / 5) * 5 + 5,
      },
      {
        top: 116, height: 78, key: "sinr", color: "#168c85", label: "SINR (dB)",
        min: Math.floor(Math.min(...linkProfile.map((row) => row.sinr), currentParameters.sinrThresholdDb) / 5) * 5 - 5,
        max: Math.ceil(Math.max(...linkProfile.map((row) => row.sinr), currentParameters.sinrThresholdDb) / 5) * 5 + 5,
      },
    ];

    panels.forEach((panel) => {
      const bottom = panel.top + panel.height;
      const yAt = (value) => panel.top + (panel.max - value) / (panel.max - panel.min) * panel.height;
      ctx.strokeStyle = "#d7ddd9";
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(pad.left, panel.top);
      ctx.lineTo(pad.left, bottom);
      ctx.lineTo(width - pad.right, bottom);
      ctx.stroke();
      ctx.fillStyle = "#64747c";
      ctx.font = "10px system-ui";
      ctx.fillText(panel.label, pad.left + 4, panel.top - 3);
      ctx.fillText(panel.max.toFixed(0), 5, panel.top + 4);
      ctx.fillText(panel.min.toFixed(0), 5, bottom);
      if (panel.key === "sinr") {
        const threshold = currentParameters.sinrThresholdDb;
        ctx.strokeStyle = "#d65353";
        ctx.setLineDash([5, 4]);
        ctx.beginPath();
        ctx.moveTo(pad.left, yAt(threshold));
        ctx.lineTo(width - pad.right, yAt(threshold));
        ctx.stroke();
        ctx.setLineDash([]);
        ctx.fillStyle = "#d65353";
        ctx.fillText(`Θ ${threshold.toFixed(1)} dB`, width - pad.right - 67, yAt(threshold) - 3);
      }
      ctx.strokeStyle = panel.color;
      ctx.lineWidth = 2;
      ctx.beginPath();
      linkProfile.forEach((row, rowIndex) => {
        const x = xAt(row.sM);
        const y = yAt(row[panel.key]);
        if (rowIndex === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
      });
      ctx.stroke();
      const current = evaluateRadio(interpolateRoute(cursorS));
      ctx.fillStyle = panel.color;
      ctx.beginPath();
      ctx.arc(xAt(cursorS), yAt(current[panel.key]), 4, 0, Math.PI * 2);
      ctx.fill();
    });
    ctx.strokeStyle = "#17364a";
    ctx.setLineDash([3, 3]);
    ctx.beginPath();
    ctx.moveTo(xAt(cursorS), 10);
    ctx.lineTo(xAt(cursorS), 198);
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle = "#64747c";
    ctx.font = "10px system-ui";
    ctx.fillText("0 km", pad.left, 215);
    ctx.fillText(`${(corridorLengthM / 1000).toFixed(1)} km`, width - pad.right - 42, 215);
  }

  const map = L.map("traffic-map", { preferCanvas: true });
  L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
    maxZoom: 19,
    attribution: "&copy; OpenStreetMap contributors",
  }).addTo(map);
  function flightRoutePositions() {
    return route.map((row) => interpolateRoute(row.s_m));
  }
  const groundRoute = L.polyline(flightRoutePositions().map((row) => [row.lat, row.lon]), { color: "#168c85", weight: 4, opacity: .85 }).addTo(map);
  map.fitBounds(groundRoute.getBounds(), { padding: [55, 55] });
  data.stations.forEach((site) => {
    L.circleMarker([site.lat, site.lon], { radius: 4, color: "#fff", weight: 1.2, fillColor: "#17364a", fillOpacity: .85 })
      .bindTooltip(`${site.id} · ${site.physical_form}`)
      .addTo(map);
  });
  const servingLine = L.polyline([], { color: "#e46f51", weight: 2, opacity: .7, dashArray: "5 5" }).addTo(map);
  const markers = new Map();
  let selectedUamId = null;
  let visibleAircraftRows = [];

  function markerIcon(policy, headingDeg, selected, groupMember) {
    const color = colors[policy] || colors.F;
    return L.divIcon({
      className: "",
      html: `<div class="uam-traffic-hit"><div class="uam-traffic-marker${groupMember ? " uam-traffic-marker--group" : ""}${selected ? " uam-traffic-marker--selected" : ""}" style="background:${color};transform:rotate(${headingDeg + 45}deg)" aria-label="${policy} policy aircraft">✈</div></div>`,
      iconSize: selected ? [36, 36] : [30, 30],
      iconAnchor: selected ? [18, 18] : [15, 15],
    });
  }

  let viewer3d = null;
  const aircraft3d = new Map();
  let fallbackMap = null;
  const fallbackAircraft = new Map();
  let fallbackServingLine = null;
  let fallbackFlightRoute = null;
  let servingLink3d = null;
  let servingSite3d = null;
  let altitudeLine3d = null;
  let flightPath3d = null;
  let cameraMode = "follow";
  const CAMERA_HEADING_DEG = 315;
  const CAMERA_PITCH_DEG = -25;
  const CAMERA_RANGE_M = 3300;

  function planeSvg(policy) {
    const color = colors[policy] || colors.F;
    return `data:image/svg+xml;charset=utf-8,${encodeURIComponent(`<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 64 64"><circle cx="32" cy="32" r="29" fill="${color}" stroke="white" stroke-width="4"/><path d="M29 11h6l4 17 15 8v5l-16-3-3 15h-6l-3-15-16 3v-5l15-8z" fill="white"/></svg>`)}`;
  }

  function fallbackAircraftIcon(row, selected) {
    const size = selected ? 36 : 25;
    return L.divIcon({
      className: "",
      html: `<div class="fallback-aircraft" style="width:${size}px"><img src="${planeSvg(row.policy)}" style="width:${size}px;height:${size}px;transform:rotate(${-row.heading}deg)" alt="${row.policy} policy aircraft" /></div>`,
      iconSize: [size, size + 24],
      iconAnchor: [size / 2, size + 18],
    });
  }

  function initializeFallback3d() {
    if (fallbackMap) return;
    if (viewer3d) {
      try { viewer3d.destroy(); } catch (_error) { /* Cesium already stopped. */ }
      viewer3d = null;
    }
    const container = document.getElementById("traffic-3d");
    container.replaceChildren();
    const fallbackNode = document.createElement("div");
    fallbackNode.className = "traffic-3d-fallback";
    container.appendChild(fallbackNode);
    fallbackMap = L.map(fallbackNode, { preferCanvas: true, zoomControl: true });
    L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
      maxZoom: 19,
      attribution: "&copy; OpenStreetMap contributors",
    }).addTo(fallbackMap);
    fallbackFlightRoute = L.polyline(flightRoutePositions().map((row) => [row.lat, row.lon]), { color: "#168c85", weight: 4, opacity: .9 }).addTo(fallbackMap);
    data.stations.forEach((site) => {
      L.marker([site.lat, site.lon], {
        icon: L.divIcon({ className: "", html: `<div class="fallback-station" aria-label="${site.id} base station"></div>`, iconSize: [22, 34], iconAnchor: [11, 32] }),
      }).bindTooltip(`${site.id} · ${site.physical_form}`).addTo(fallbackMap);
    });
    fallbackServingLine = L.polyline([], { color: "#e46f51", weight: 3, dashArray: "7 6" }).addTo(fallbackMap);
    el["three-warning"].hidden = false;
    el["three-camera-note"].textContent = "3D engine unavailable · selected-aircraft map fallback.";
    window.setTimeout(() => updateFrame(index), 0);
  }

  function addVertiport3d(lon, lat, text, colorHex) {
    const color = Cesium.Color.fromCssColorString(colorHex);
    viewer3d.entities.add({
      position: Cesium.Cartesian3.fromDegrees(lon, lat, 3),
      ellipse: { semiMajorAxis: 145, semiMinorAxis: 145, material: color.withAlpha(.78), outline: true, outlineColor: Cesium.Color.WHITE },
      label: { text, font: "700 12px system-ui", fillColor: Cesium.Color.WHITE, showBackground: true, backgroundColor: color.withAlpha(.9), pixelOffset: new Cesium.Cartesian2(0, -24), distanceDisplayCondition: new Cesium.DistanceDisplayCondition(0, 50000), disableDepthTestDistance: Number.POSITIVE_INFINITY },
    });
  }

  function stationLabel(site) {
    return { text: site.id, font: "700 11px system-ui", fillColor: Cesium.Color.WHITE, showBackground: true, backgroundColor: Cesium.Color.fromCssColorString("#17364a").withAlpha(.82), pixelOffset: new Cesium.Cartesian2(0, -17), distanceDisplayCondition: new Cesium.DistanceDisplayCondition(0, 9500), disableDepthTestDistance: Number.POSITIVE_INFINITY };
  }

  function addAntennaCrown3d(site, color) {
    [0, 120, 240].forEach((angleDeg, panelIndex) => {
      const angle = Cesium.Math.toRadians(angleDeg);
      const lat = site.lat + Math.cos(angle) * 2.2 / 111320;
      const lon = site.lon + Math.sin(angle) * 2.2 / (111320 * Math.cos(Cesium.Math.toRadians(site.lat)));
      const position = Cesium.Cartesian3.fromDegrees(lon, lat, site.height_m - 1.8);
      viewer3d.entities.add({
        position,
        orientation: Cesium.Transforms.headingPitchRollQuaternion(position, new Cesium.HeadingPitchRoll(angle, 0, 0)),
        box: { dimensions: new Cesium.Cartesian3(.55, .22, 3.6), material: color, outline: true, outlineColor: Cesium.Color.fromCssColorString("#627177") },
        label: panelIndex === 0 ? stationLabel(site) : undefined,
      });
    });
  }

  function stationSvg() {
    return `data:image/svg+xml;charset=utf-8,${encodeURIComponent(`<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 44 56"><path d="M20 12h4v38h-4z" fill="#17364a"/><path d="M8 15h28v5H8zm4 10h20v5H12z" rx="2" fill="#17364a"/><path d="M22 4c8 0 15 4 20 10M22 4C14 4 7 8 2 14" fill="none" stroke="#e46f51" stroke-width="3" stroke-linecap="round"/><circle cx="22" cy="7" r="4" fill="#e46f51" stroke="white" stroke-width="2"/></svg>`)}`;
  }

  function addStation3d(site) {
    const form = site.physical_form.toLowerCase();
    if (site.site_class === "Macro_Building") {
      const height = Math.max(9, site.height_m - 3.5);
      viewer3d.entities.add({ position: Cesium.Cartesian3.fromDegrees(site.lon, site.lat, height / 2), box: { dimensions: new Cesium.Cartesian3(25, 19, height), material: Cesium.Color.fromCssColorString("#b8b1a6").withAlpha(.92), outline: true, outlineColor: Cesium.Color.fromCssColorString("#7b746a") } });
      addAntennaCrown3d(site, Cesium.Color.fromCssColorString("#e4e8e8"));
    } else if (site.site_class === "Macro_Other") {
      const stem = site.height_m * .68;
      viewer3d.entities.add({ position: Cesium.Cartesian3.fromDegrees(site.lon, site.lat, stem / 2), cylinder: { length: stem, topRadius: 2.1, bottomRadius: 3.4, material: Cesium.Color.fromCssColorString("#9fa8aa") } });
      viewer3d.entities.add({ position: Cesium.Cartesian3.fromDegrees(site.lon, site.lat, stem + 3.8), cylinder: { length: 7.6, topRadius: 6.2, bottomRadius: 4.8, material: Cesium.Color.fromCssColorString("#c8d0d1") } });
      addAntennaCrown3d(site, Cesium.Color.fromCssColorString("#e4e8e8"));
    } else if (form.includes("tree") || form.includes("monopine") || form.includes("eucalyptus")) {
      const crown = Math.max(8, site.height_m * .44);
      viewer3d.entities.add({ position: Cesium.Cartesian3.fromDegrees(site.lon, site.lat, site.height_m / 2), cylinder: { length: site.height_m, topRadius: .45, bottomRadius: .72, material: Cesium.Color.fromCssColorString("#665746") } });
      viewer3d.entities.add({ position: Cesium.Cartesian3.fromDegrees(site.lon, site.lat, site.height_m - crown / 2), cylinder: { length: crown, topRadius: 1.1, bottomRadius: 5.8, material: Cesium.Color.fromCssColorString("#315c45").withAlpha(.88) }, label: stationLabel(site) });
    } else {
      const height = Math.max(8, site.height_m - 2.5);
      viewer3d.entities.add({ position: Cesium.Cartesian3.fromDegrees(site.lon, site.lat, height / 2), cylinder: { length: height, topRadius: .48, bottomRadius: .85, material: Cesium.Color.fromCssColorString("#58656c") } });
      addAntennaCrown3d(site, Cesium.Color.fromCssColorString("#dce4e5"));
    }
    viewer3d.entities.add({
      name: `${site.id} base-station symbol`,
      position: Cesium.Cartesian3.fromDegrees(site.lon, site.lat, site.height_m),
      billboard: {
        image: stationSvg(),
        width: 22,
        height: 28,
        pixelOffset: new Cesium.Cartesian2(0, -18),
        verticalOrigin: Cesium.VerticalOrigin.BOTTOM,
        disableDepthTestDistance: Number.POSITIVE_INFINITY,
        distanceDisplayCondition: new Cesium.DistanceDisplayCondition(0, 18000),
      },
    });
  }

  function initialize3d() {
    if (window.location.protocol === "file:" || !window.Cesium) { initializeFallback3d(); return; }
    try {
      viewer3d = new Cesium.Viewer("traffic-3d", {
        baseLayer: new Cesium.ImageryLayer(new Cesium.OpenStreetMapImageryProvider({ url: "https://tile.openstreetmap.org/", credit: "© OpenStreetMap contributors" })),
        terrainProvider: new Cesium.EllipsoidTerrainProvider(),
        animation: false, timeline: false, geocoder: false, homeButton: false,
        sceneModePicker: false, baseLayerPicker: false, navigationHelpButton: false,
        fullscreenButton: false, selectionIndicator: true, infoBox: true,
      });
    } catch (_error) { initializeFallback3d(); return; }
    viewer3d.scene.renderError.addEventListener(() => initializeFallback3d());
    viewer3d.scene.globe.depthTestAgainstTerrain = false;
    viewer3d.scene.globe.enableLighting = true;
    viewer3d.scene.fog.enabled = true;
    viewer3d.scene.fog.density = .00018;
    const controller = viewer3d.scene.screenSpaceCameraController;
    controller.enableCollisionDetection = false;
    controller.enableRotate = true;
    controller.enableTranslate = true;
    controller.enableZoom = true;
    controller.enableTilt = true;
    controller.enableLook = true;
    viewer3d.entities.add({ polyline: { positions: Cesium.Cartesian3.fromDegreesArray(data.route.flat()), width: 3, material: new Cesium.PolylineDashMaterialProperty({ color: Cesium.Color.fromCssColorString("#607177"), dashLength: 14 }), clampToGround: true } });
    flightPath3d = viewer3d.entities.add({ polyline: { positions: Cesium.Cartesian3.fromDegreesArrayHeights(flightRoutePositions().flatMap((row) => [row.lon, row.lat, altitudeM])), width: 5, material: Cesium.Color.fromCssColorString("#21b7aa").withAlpha(.94), arcType: Cesium.ArcType.GEODESIC } });
    const start = data.route[0];
    const end = data.route.at(-1);
    addVertiport3d(start[0], start[1], `START · ${display.origin_label || "Corridor origin"}`, "#17364a");
    addVertiport3d(end[0], end[1], `END · ${display.destination_label || "Corridor destination"}`, "#e46f51");
    data.stations.forEach(addStation3d);
    servingLink3d = viewer3d.entities.add({ polyline: { positions: [], width: 3, material: new Cesium.PolylineDashMaterialProperty({ color: Cesium.Color.fromCssColorString("#ff7658"), dashLength: 18 }), arcType: Cesium.ArcType.NONE } });
    altitudeLine3d = viewer3d.entities.add({ polyline: { positions: [], width: 2, material: new Cesium.PolylineDashMaterialProperty({ color: Cesium.Color.fromCssColorString("#17364a").withAlpha(.72), dashLength: 12 }), arcType: Cesium.ArcType.NONE } });
    servingSite3d = viewer3d.entities.add({
      position: Cesium.Cartesian3.fromDegrees(start[0], start[1], 0),
      point: { pixelSize: 16, color: Cesium.Color.fromCssColorString("#ff7658").withAlpha(.3), outlineColor: Cesium.Color.fromCssColorString("#ff7658"), outlineWidth: 3, disableDepthTestDistance: Number.POSITIVE_INFINITY },
    });
    const canvas = viewer3d.scene.canvas;
    canvas.addEventListener("pointerdown", releaseCameraToFree, { capture: true });
    canvas.addEventListener("wheel", releaseCameraToFree, { capture: true, passive: true });
  }
  initialize3d();

  function annotateCamera() {
    el["three-camera-state"].textContent = cameraMode === "follow" ? `${CAMERA_HEADING_DEG}° bearing · fixed pitch` : "Free view";
    el["three-free-view"].setAttribute("aria-pressed", String(cameraMode === "free"));
    el["three-recenter"].setAttribute("aria-pressed", String(cameraMode === "follow"));
    el["three-camera-note"].textContent = cameraMode === "follow"
      ? `Following ${selectedUamId || "selected aircraft"} · fixed bearing and pitch.`
      : "Free view · drag to orbit, scroll to zoom · Recenter restores focal-aircraft follow.";
  }

  function applyFollowCamera(row) {
    if (!viewer3d || !row) return;
    const target = Cesium.Cartesian3.fromDegrees(row.position.lon, row.position.lat, Math.max(60, altitudeM * .55));
    viewer3d.camera.lookAt(target, new Cesium.HeadingPitchRange(Cesium.Math.toRadians(CAMERA_HEADING_DEG), Cesium.Math.toRadians(CAMERA_PITCH_DEG), CAMERA_RANGE_M));
  }

  function setFocalCamera(row) {
    cameraMode = "follow";
    applyFollowCamera(row);
    annotateCamera();
  }

  function releaseCameraToFree() {
    if (cameraMode === "free") return;
    cameraMode = "free";
    if (viewer3d) viewer3d.camera.lookAtTransform(Cesium.Matrix4.IDENTITY);
    annotateCamera();
  }

  function recenter3d() {
    const focal = activeAircraft(frames[index]).find((row) => row.id === selectedUamId);
    if (!focal) return;
    if (fallbackMap) {
      cameraMode = "follow";
      fallbackMap.setView([focal.position.lat, focal.position.lon], 13);
      annotateCamera();
      return;
    }
    setFocalCamera(focal);
  }
  el["three-free-view"].addEventListener("click", releaseCameraToFree);
  el["three-recenter"].addEventListener("click", recenter3d);

  function updateFlightRouteGeometry() {
    const positions = flightRoutePositions();
    groundRoute.setLatLngs(positions.map((row) => [row.lat, row.lon]));
    if (fallbackFlightRoute) fallbackFlightRoute.setLatLngs(positions.map((row) => [row.lat, row.lon]));
    if (viewer3d && flightPath3d) {
      flightPath3d.polyline.positions = Cesium.Cartesian3.fromDegreesArrayHeights(
        positions.flatMap((row) => [row.lon, row.lat, altitudeM]),
      );
    }
  }

  function setExperimentBusy(busy) {
    el["run-experiment"].disabled = busy;
    el["reset-experiment"].disabled = busy;
    el["run-experiment"].textContent = busy ? "Running…" : "Run experiment";
  }

  function applyExperiment(parameters, statusLabel) {
    setPlaying(false);
    setExperimentBusy(true);
    el["experiment-status"].textContent = "Running deterministic one-second simulation…";
    window.setTimeout(() => {
      try {
        const simulation = engine.simulate(data, parameters);
        currentParameters = simulation.parameters;
        currentResults = simulation.results;
        entrants = simulation.entrants;
        frames = simulation.frames;
        speedMps = currentParameters.speedMps;
        altitudeM = currentParameters.altitudeM;
        lateralOffsetM = currentParameters.lateralOffsetM;
        selectedUamId = null;
        cameraMode = "follow";
        index = 0;
        simulatedTime = frames[0].t;
        linkProfile = buildLinkProfile();
        updateFlightRouteGeometry();
        populateParameterForm(currentParameters);
        updateScenarioSummary(currentResults, currentParameters);
        el["time-slider"].max = String(frames.length - 1);
        el["time-slider"].value = "0";
        el["total-time"].textContent = `/ ${formatTime(frames.at(-1).t)}`;
        updateFrame(0);
        el["experiment-status"].textContent = `${statusLabel} · ${frames.length.toLocaleString()} one-second snapshots`;
      } catch (error) {
        el["experiment-status"].textContent = `Input error: ${error.message}`;
      } finally {
        setExperimentBusy(false);
      }
    }, 20);
  }

  el["experiment-form"].addEventListener("submit", (event) => {
    event.preventDefault();
    applyExperiment(readParameterForm(), "Experiment complete");
  });
  el["reset-experiment"].addEventListener("click", () => {
    populateParameterForm(baselineParameters);
    applyExperiment({ ...baselineParameters }, "Baseline restored");
  });

  function activeAircraft(frame) {
    return entrants.filter((uam) => uam.entry <= frame.t + 1e-9 && uam.exit >= frame.t - 1e-9).map((uam) => {
      const sM = Math.max(0, Math.min(corridorLengthM, speedMps * (frame.t - uam.entry)));
      const position = interpolateRoute(sM);
      const heading = Math.atan2(position.tangentX, position.tangentY) * 180 / Math.PI;
      const policy = frame.policies[uam.id];
      if (!colors[policy]) throw new Error(`Missing C/R/F policy for active ${uam.id} at ${frame.t}s`);
      return { ...uam, sM, position, heading, policy, exposure: frame.exposure[uam.id] };
    });
  }

  function setSelected(uamId) {
    setPlaying(false);
    selectedUamId = uamId;
    cameraMode = "follow";
    updateFrame(index);
  }

  map.on("click", (event) => {
    const clickPoint = map.latLngToContainerPoint(event.latlng);
    const nearest = visibleAircraftRows
      .map((row) => ({ row, distance: clickPoint.distanceTo(map.latLngToContainerPoint([row.position.lat, row.position.lon])) }))
      .sort((left, right) => left.distance - right.distance)[0];
    if (nearest && nearest.distance <= 34) setSelected(nearest.row.id);
  });

  function update2d(aircraftRows, groups) {
    visibleAircraftRows = aircraftRows;
    const focal = aircraftRows.find((row) => row.id === selectedUamId);
    const groupIds = new Set(focal ? groups[focal.id] : []);
    const activeIds = new Set(aircraftRows.map((row) => row.id));
    for (const [uamId, marker] of markers) {
      if (!activeIds.has(uamId)) { map.removeLayer(marker); markers.delete(uamId); }
    }
    aircraftRows.forEach((row) => {
      const selected = row.id === selectedUamId;
      let marker = markers.get(row.id);
      if (!marker) {
        marker = L.marker([row.position.lat, row.position.lon], {
          zIndexOffset: selected ? 1300 : 900,
          interactive: true,
          keyboard: true,
          riseOnHover: true,
          bubblingMouseEvents: true,
          title: `Select ${row.id}`,
        })
          .on("click", () => setSelected(row.id))
          .addTo(map);
        markers.set(row.id, marker);
      }
      marker.setLatLng([row.position.lat, row.position.lon]);
      marker.setIcon(markerIcon(row.policy, row.heading, selected, groupIds.has(row.id)));
      marker.bindTooltip(`${row.id} · ${row.policy}`, { direction: "top" });
    });
  }

  function update3d(aircraftRows, selected, selectedLink) {
    if (fallbackMap) {
      const activeIds = new Set(aircraftRows.map((row) => row.id));
      for (const [uamId, marker] of fallbackAircraft) {
        if (!activeIds.has(uamId)) {
          fallbackMap.removeLayer(marker);
          fallbackAircraft.delete(uamId);
        }
      }
      aircraftRows.forEach((row) => {
        const isSelected = row.id === selected?.id;
        let marker = fallbackAircraft.get(row.id);
        if (!marker) {
          marker = L.marker([row.position.lat, row.position.lon], {
            icon: fallbackAircraftIcon(row, isSelected),
            zIndexOffset: isSelected ? 1200 : 900,
          }).addTo(fallbackMap);
          fallbackAircraft.set(row.id, marker);
        }
        marker.setLatLng([row.position.lat, row.position.lon]);
        marker.setIcon(fallbackAircraftIcon(row, isSelected));
        marker.bindTooltip(`${row.id} · ${row.policy}`);
      });
      fallbackServingLine.setLatLngs(selected && selectedLink ? [
        [selected.position.lat, selected.position.lon],
        [selectedLink.site.lat, selectedLink.site.lon],
      ] : []);
      if (selected && cameraMode === "follow") {
        fallbackMap.setView([selected.position.lat, selected.position.lon], 13, { animate: false });
        annotateCamera();
      }
      return;
    }
    if (!viewer3d) return;
    const activeIds = new Set(aircraftRows.map((row) => row.id));
    for (const [uamId, entity] of aircraft3d) {
      if (!activeIds.has(uamId)) { viewer3d.entities.remove(entity); aircraft3d.delete(uamId); }
    }
    aircraftRows.forEach((row) => {
      const isSelected = row.id === selected?.id;
      const position = Cesium.Cartesian3.fromDegrees(row.position.lon, row.position.lat, altitudeM);
      let entity = aircraft3d.get(row.id);
      if (!entity) {
        entity = viewer3d.entities.add({
          name: row.id,
          position,
          orientation: Cesium.Transforms.headingPitchRollQuaternion(position, new Cesium.HeadingPitchRoll(Cesium.Math.toRadians(row.heading), 0, 0)),
          model: {
            uri: "assets/models/cesium-drone/CesiumDrone.glb",
            scale: 6,
            minimumPixelSize: 28,
            maximumScale: 120,
            silhouetteColor: Cesium.Color.fromCssColorString(colors[row.policy]),
            silhouetteSize: 1,
          },
          billboard: {
            image: planeSvg(row.policy),
            width: 24,
            height: 24,
            pixelOffset: new Cesium.Cartesian2(0, -34),
            verticalOrigin: Cesium.VerticalOrigin.BOTTOM,
            disableDepthTestDistance: Number.POSITIVE_INFINITY,
            distanceDisplayCondition: new Cesium.DistanceDisplayCondition(0, 18000),
          },
          label: {
            text: row.id,
            font: "700 12px system-ui",
            fillColor: Cesium.Color.WHITE,
            showBackground: true,
            backgroundColor: Cesium.Color.fromCssColorString(colors[row.policy]).withAlpha(.88),
            pixelOffset: new Cesium.Cartesian2(0, -78),
            distanceDisplayCondition: new Cesium.DistanceDisplayCondition(0, 16000),
            disableDepthTestDistance: Number.POSITIVE_INFINITY,
          },
        });
        entity._policyCode = row.policy;
        aircraft3d.set(row.id, entity);
      }
      entity.position = position;
      entity.orientation = Cesium.Transforms.headingPitchRollQuaternion(position, new Cesium.HeadingPitchRoll(Cesium.Math.toRadians(row.heading), 0, 0));
      entity.model.scale = isSelected ? 10 : 6;
      entity.model.minimumPixelSize = isSelected ? 76 : 28;
      entity.model.silhouetteColor = Cesium.Color.fromCssColorString(colors[row.policy]);
      entity.model.silhouetteSize = isSelected ? 2 : 1;
      if (entity._policyCode !== row.policy) {
        entity.billboard.image = planeSvg(row.policy);
        entity._policyCode = row.policy;
      }
      entity.billboard.width = isSelected ? 38 : 24;
      entity.billboard.height = isSelected ? 38 : 24;
      entity.billboard.pixelOffset = new Cesium.Cartesian2(0, isSelected ? -58 : -34);
      entity.label.show = isSelected;
      entity.label.text = `${row.id} · ${altitudeM.toFixed(0)} m · ${row.policy}`;
      entity.label.backgroundColor = Cesium.Color.fromCssColorString(colors[row.policy]).withAlpha(.88);
    });
    if (selected && selectedLink) {
      const aircraftPosition = Cesium.Cartesian3.fromDegrees(selected.position.lon, selected.position.lat, altitudeM);
      const siteTop = Cesium.Cartesian3.fromDegrees(selectedLink.site.lon, selectedLink.site.lat, selectedLink.site.height_m);
      altitudeLine3d.polyline.positions = [Cesium.Cartesian3.fromDegrees(selected.position.lon, selected.position.lat, 0), aircraftPosition];
      servingLink3d.polyline.positions = [aircraftPosition, siteTop];
      servingSite3d.position = siteTop;
      servingSite3d.name = `Serving base station · ${selectedLink.site.id}`;
      if (cameraMode === "follow") {
        applyFollowCamera(selected);
        annotateCamera();
      }
    }
  }

  function drawCapacityChart(cursor) {
    const canvas = capacityCanvas;
    const ratio = window.devicePixelRatio || 1;
    const width = Math.max(320, canvas.clientWidth);
    const height = 150;
    canvas.width = width * ratio;
    canvas.height = height * ratio;
    const ctx = canvas.getContext("2d");
    ctx.scale(ratio, ratio);
    ctx.clearRect(0, 0, width, height);
    const pad = { left: 38, right: 10, top: 10, bottom: 22 };
    const values = frames.map((frame) => frame.q_mix);
    const demand = currentResults.offeredDemand;
    const q95 = currentResults.reliabilityCapacity;
    const minY = Math.min(...values, demand, q95) - 5;
    const maxY = Math.max(...values, demand, q95) + 5;
    const xAt = (i) => pad.left + i / (frames.length - 1) * (width - pad.left - pad.right);
    const yAt = (v) => pad.top + (maxY - v) / (maxY - minY) * (height - pad.top - pad.bottom);
    ctx.strokeStyle = "#d7ddd9"; ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(pad.left, pad.top); ctx.lineTo(pad.left, height - pad.bottom); ctx.lineTo(width - pad.right, height - pad.bottom); ctx.stroke();
    [[demand, "#d65353"], [q95, "#17364a"]].forEach(([value, color]) => { ctx.strokeStyle = color; ctx.setLineDash([5, 4]); ctx.beginPath(); ctx.moveTo(pad.left, yAt(value)); ctx.lineTo(width - pad.right, yAt(value)); ctx.stroke(); });
    ctx.setLineDash([]); ctx.strokeStyle = "#168c85"; ctx.lineWidth = 2; ctx.beginPath(); values.forEach((value, i) => { if (i === 0) ctx.moveTo(xAt(i), yAt(value)); else ctx.lineTo(xAt(i), yAt(value)); }); ctx.stroke();
    ctx.fillStyle = "#168c85"; ctx.beginPath(); ctx.arc(xAt(cursor), yAt(values[cursor]), 4, 0, Math.PI * 2); ctx.fill();
    ctx.fillStyle = "#64747c"; ctx.font = "10px system-ui"; ctx.fillText(`${maxY.toFixed(0)}`, 4, pad.top + 4); ctx.fillText(`${minY.toFixed(0)}`, 4, height - pad.bottom); ctx.fillText("mixed capacity", pad.left + 4, 12);
  }

  let index = 0;
  let playing = false;
  let lastFrameTime = null;
  let simulatedTime = frames[0].t;
  let rafId = null;
  el["time-slider"].max = String(frames.length - 1);
  el["total-time"].textContent = `/ ${formatTime(frames.at(-1).t)}`;
  el["reset-button"].disabled = false;
  el["play-button"].disabled = false;
  el["play-button"].textContent = "▶";
  el["play-button"].setAttribute("aria-label", "Play simulation");
  el["time-slider"].disabled = false;
  el["playback-speed"].disabled = false;

  function updateFrame(nextIndex) {
    index = Math.max(0, Math.min(frames.length - 1, Math.round(nextIndex)));
    const frame = frames[index];
    const aircraftRows = activeAircraft(frame);
    const previousSelection = selectedUamId;
    if (!selectedUamId || !aircraftRows.some((row) => row.id === selectedUamId)) {
      selectedUamId = aircraftRows[0]?.id || null;
    }
    if (selectedUamId !== previousSelection) cameraMode = "follow";
    const selected = aircraftRows.find((row) => row.id === selectedUamId);
    const selectedLink = selected ? evaluateRadio(selected.position) : null;
    update2d(aircraftRows, frame.groups);
    update3d(aircraftRows, selected, selectedLink);
    el["time-slider"].value = String(index);
    el["current-time"].textContent = formatTime(frame.t);
    el["active-count"].textContent = String(frame.active_count);
    el["current-capacity"].textContent = frame.q_mix.toFixed(1);
    el["current-counts"].textContent = `Current C/R/F · ${frame.n_C}/${frame.n_R}/${frame.n_F}`;
    if (selected) {
      el["selected-uam"].textContent = selected.id;
      el["selected-policy"].textContent = selected.policy;
      el["selected-policy"].style.color = colors[selected.policy];
      el["selected-exposure"].textContent = `${(100 * selected.exposure).toFixed(1)}% below Θ`;
      el["selected-site"].textContent = selectedLink.site.id;
      el["selected-rsrp"].textContent = `${selectedLink.rsrp.toFixed(1)} dBm/RE`;
      el["selected-sinr"].textContent = `${selectedLink.sinr.toFixed(1)} dB`;
      el["selected-progress"].textContent = `${(selected.sM / 1000).toFixed(2)} km · ${(100 * selected.sM / corridorLengthM).toFixed(1)}%`;
      el["selected-group"].textContent = frame.groups[selected.id].join(" · ");
      el["link-current"].textContent = `${selected.id} · ${(selected.sM / 1000).toFixed(2)} km`;
      servingLine.setLatLngs([[selected.position.lat, selected.position.lon], [selectedLink.site.lat, selectedLink.site.lon]]);
      drawLinkQualityChart(selected);
    } else {
      servingLine.setLatLngs([]);
      el["link-current"].textContent = "—";
      drawLinkQualityChart(null);
    }
    drawCapacityChart(index);
  }

  function setPlaying(next) {
    playing = next;
    el["play-button"].textContent = next ? "❚❚" : "▶";
    el["play-button"].setAttribute("aria-label", next ? "Pause simulation" : "Play simulation");
    if (next) { lastFrameTime = null; rafId = requestAnimationFrame(animationFrame); }
    else if (rafId !== null) { cancelAnimationFrame(rafId); rafId = null; }
  }

  function animationFrame(timestamp) {
    if (!playing) return;
    if (lastFrameTime === null) lastFrameTime = timestamp;
    const elapsed = (timestamp - lastFrameTime) / 1000;
    lastFrameTime = timestamp;
    simulatedTime += elapsed * Number(el["playback-speed"].value);
    while (index < frames.length - 1 && frames[index + 1].t <= simulatedTime) index += 1;
    updateFrame(index);
    if (index >= frames.length - 1) { setPlaying(false); return; }
    rafId = requestAnimationFrame(animationFrame);
  }

  el["play-button"].addEventListener("click", () => {
    if (!playing && index >= frames.length - 1) { index = 0; simulatedTime = frames[0].t; }
    setPlaying(!playing);
  });
  el["reset-button"].addEventListener("click", () => { setPlaying(false); index = 0; simulatedTime = frames[0].t; updateFrame(0); });
  el["time-slider"].addEventListener("input", () => { setPlaying(false); index = Number(el["time-slider"].value); simulatedTime = frames[index].t; updateFrame(index); });
  window.addEventListener("resize", () => {
    drawCapacityChart(index);
    const selected = activeAircraft(frames[index]).find((row) => row.id === selectedUamId);
    drawLinkQualityChart(selected || null);
  });
  window.UAM_TRAFFIC_QA = {
    camera: () => ({
      mode: cameraMode,
      selected_uam: selectedUamId,
      heading_deg: CAMERA_HEADING_DEG,
      pitch_deg: CAMERA_PITCH_DEG,
      range_m: CAMERA_RANGE_M,
    }),
    scene: () => ({
      engine: viewer3d ? "cesium" : "fallback",
      aircraft_count: viewer3d ? aircraft3d.size : fallbackAircraft.size,
      aircraft_symbol_count: viewer3d ? [...aircraft3d.values()].filter((entity) => Boolean(entity.billboard)).length : fallbackAircraft.size,
      station_symbol_count: viewer3d ? viewer3d.entities.values.filter((entity) => entity.name?.endsWith("base-station symbol")).length : data.stations.length,
      serving_link: Boolean(viewer3d ? servingLink3d : fallbackServingLine),
      altitude_reference: Boolean(viewer3d && altitudeLine3d),
    }),
  };
  updateFrame(0);
})();
