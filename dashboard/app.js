(() => {
  "use strict";

  const data = window.UAM_DASHBOARD_DATA;
  const errorPanel = document.getElementById("error-panel");
  if (!data || !window.L) {
    errorPanel.hidden = false;
    errorPanel.textContent = "Dashboard data or map library could not be loaded. Regenerate the data bundle and check the map connection.";
    return;
  }

  const baseTrace = data.trace;
  const stationById = new Map(data.stations.map((site) => [site.id, site]));
  const defaults = {
    speed: Number(data.summary.defaults.speed_mps),
    altitude: Number(data.summary.defaults.altitude_m),
    offset: Number(data.summary.defaults.lateral_offset_m),
  };
  const radio = data.summary.radio;
  const corridorLengthM = data.summary.corridor_length_km * 1000;

  const map = L.map("map", { zoomControl: true, preferCanvas: true });
  L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
    maxZoom: 19,
    attribution: "&copy; OpenStreetMap contributors",
  }).addTo(map);
  const groundRoute = L.polyline(data.route.map(([lon, lat]) => [lat, lon]), {
    color: "#718187", weight: 2.5, opacity: 0.58, dashArray: "6 5",
  }).addTo(map);
  let flightRoute = L.polyline(baseTrace.map((row) => [row.lat, row.lon]), {
    color: "#168c85", weight: 4, opacity: 0.9,
  }).addTo(map);
  map.fitBounds(groundRoute.getBounds(), { padding: [25, 25] });

  const routeStart = data.route[0];
  const routeEnd = data.route[data.route.length - 1];
  L.circleMarker([routeStart[1], routeStart[0]], { radius: 6, color: "#fff", weight: 2, fillColor: "#17364a", fillOpacity: 1 }).bindTooltip("San Francisco origin").addTo(map);
  L.circleMarker([routeEnd[1], routeEnd[0]], { radius: 6, color: "#fff", weight: 2, fillColor: "#e46f51", fillOpacity: 1 }).bindTooltip("San Jose destination").addTo(map);

  const stationMarkers = new Map();
  data.stations.forEach((site) => {
    const marker = L.circleMarker([site.lat, site.lon], {
      radius: 5, color: "#fffdfa", weight: 1.5,
      fillColor: site.site_class === "Macro_Building" ? "#d6a545" : "#17364a", fillOpacity: 0.92,
    }).addTo(map);
    marker.bindTooltip(site.id, { direction: "top", className: "bs-tooltip" });
    marker.bindPopup(`<strong>${site.id}</strong><br>${site.physical_form}<br>${site.address}<br><small>${site.operator}</small>`);
    stationMarkers.set(site.id, marker);
  });

  const aircraft = L.marker([baseTrace[0].lat, baseTrace[0].lon], {
    icon: L.divIcon({ className: "", html: '<div class="uam-marker" aria-label="UAM aircraft">✈</div>', iconSize: [34, 34], iconAnchor: [17, 17] }),
    zIndexOffset: 1000,
  }).addTo(map);
  const servingLine = L.polyline([], { color: "#e46f51", weight: 2, opacity: 0.75, dashArray: "5 5" }).addTo(map);

  const ids = [
    "play-button", "reset-button", "time-slider", "speed-select", "current-time", "total-time",
    "progress-km", "progress-percent", "serving-site", "serving-form", "rsrp-value", "sinr-value",
    "handoff-count", "site-address", "site-class", "site-operator", "site-height", "run-id", "run-status",
    "clock-summary", "rsrp-range", "sinr-range", "flight-speed", "flight-height", "flight-offset",
    "flight-speed-value", "flight-height-value", "flight-offset-value", "apply-parameters", "reset-parameters",
    "preview-status", "three-parameter-label", "reset-3d-camera", "three-file-warning",
  ];
  const el = Object.fromEntries(ids.map((id) => [id, document.getElementById(id)]));
  const rsrpCanvas = document.getElementById("rsrp-chart");
  const sinrCanvas = document.getElementById("sinr-chart");

  el["run-id"].textContent = data.summary.run_id;
  el["clock-summary"].textContent = `Motion ${data.summary.clock.dt_motion_s} s · Radio ${data.summary.clock.dt_radio_s} s · Control ${data.summary.clock.dt_control_s} s`;
  el["flight-speed"].value = String(defaults.speed);
  el["flight-height"].value = String(defaults.altitude);
  el["flight-offset"].value = String(defaults.offset);

  let trace = baseTrace.map((row) => ({ ...row }));
  let cumulativeHandoffs = buildHandoffCounts(trace);
  let rsrpValues = trace.map((row) => row.rsrp);
  let sinrValues = trace.map((row) => row.sinr);
  let rsrpDomain = numericDomain(rsrpValues);
  let sinrDomain = numericDomain(sinrValues);
  let activeParameters = { ...defaults };
  let index = 0;
  let playing = false;
  let lastFrameTime = null;
  let simulatedClock = 0;
  let lastServingId = null;
  let viewer3d = null;
  let aircraft3d = null;
  let flightRoute3d = null;
  let servingLink3d = null;
  let servingSite3d = null;
  let altitudeLine3d = null;

  initialize3DScene();

  function initialize3DScene() {
    if (window.location.protocol === "file:") {
      el["three-file-warning"].hidden = false;
      return;
    }
    if (!window.Cesium) {
      el["three-file-warning"].hidden = false;
      el["three-file-warning"].textContent = "The 3D geographic library could not be loaded. The 2D map and radio traces remain available.";
      return;
    }

    viewer3d = new Cesium.Viewer("corridor-3d", {
      baseLayer: new Cesium.ImageryLayer(new Cesium.OpenStreetMapImageryProvider({
        url: "https://tile.openstreetmap.org/",
        credit: "© OpenStreetMap contributors",
      })),
      terrainProvider: new Cesium.EllipsoidTerrainProvider(),
      animation: false,
      timeline: false,
      baseLayerPicker: false,
      geocoder: false,
      homeButton: false,
      sceneModePicker: false,
      navigationHelpButton: false,
      fullscreenButton: false,
      selectionIndicator: true,
      infoBox: true,
      shouldAnimate: false,
    });
    viewer3d.scene.globe.depthTestAgainstTerrain = false;
    viewer3d.scene.globe.enableLighting = true;
    viewer3d.scene.fog.enabled = true;
    viewer3d.scene.fog.density = 0.00018;
    viewer3d.scene.skyAtmosphere.show = true;
    viewer3d.scene.screenSpaceCameraController.enableCollisionDetection = false;

    const groundPositions = Cesium.Cartesian3.fromDegreesArray(data.route.flatMap(([lon, lat]) => [lon, lat]));
    viewer3d.entities.add({
      name: "Caltrain-referenced ground corridor",
      polyline: {
        positions: groundPositions,
        width: 3,
        material: new Cesium.PolylineDashMaterialProperty({ color: Cesium.Color.fromCssColorString("#607177"), dashLength: 14 }),
        clampToGround: true,
      },
    });

    flightRoute3d = viewer3d.entities.add({
      name: "Selected UAM flight path",
      polyline: {
        positions: Cesium.Cartesian3.fromDegreesArrayHeights(baseTrace.flatMap((row) => [row.lon, row.lat, row.altitude_m])),
        width: 5,
        material: Cesium.Color.fromCssColorString("#21b7aa").withAlpha(0.94),
        arcType: Cesium.ArcType.GEODESIC,
      },
    });

    addVertiport3D(routeStart[0], routeStart[1], "San Francisco", "Origin vertiport");
    addVertiport3D(routeEnd[0], routeEnd[1], "San Jose Diridon", "Destination vertiport");
    addGeographicLabels();
    data.stations.forEach(addStation3D);

    const initial = baseTrace[0];
    const initialPosition = Cesium.Cartesian3.fromDegrees(initial.lon, initial.lat, initial.altitude_m);
    aircraft3d = viewer3d.entities.add({
      name: "UAM aircraft",
      description: "Representative UAM aircraft geometry. Position and altitude are data-driven; rendered model size is enhanced for legibility.",
      position: initialPosition,
      orientation: aircraftOrientation(initialPosition, 0),
      model: {
        uri: "assets/models/cesium-drone/CesiumDrone.glb",
        scale: 9,
        minimumPixelSize: 70,
        maximumScale: 120,
        silhouetteColor: Cesium.Color.fromCssColorString("#fff7ed"),
        silhouetteSize: 1.2,
        shadows: Cesium.ShadowMode.ENABLED,
      },
      label: {
        text: "UAM",
        font: "700 13px system-ui",
        fillColor: Cesium.Color.WHITE,
        showBackground: true,
        backgroundColor: Cesium.Color.fromCssColorString("#e46f51").withAlpha(0.88),
        pixelOffset: new Cesium.Cartesian2(0, -38),
        distanceDisplayCondition: new Cesium.DistanceDisplayCondition(0, 16000),
      },
    });

    servingLink3d = viewer3d.entities.add({
      name: "Serving radio link",
      polyline: {
        positions: [],
        width: 3,
        material: new Cesium.PolylineDashMaterialProperty({ color: Cesium.Color.fromCssColorString("#ff7658"), dashLength: 18 }),
        arcType: Cesium.ArcType.NONE,
      },
    });
    altitudeLine3d = viewer3d.entities.add({
      name: "Aircraft altitude reference",
      polyline: {
        positions: [Cesium.Cartesian3.fromDegrees(initial.lon, initial.lat, 0), initialPosition],
        width: 2,
        material: new Cesium.PolylineDashMaterialProperty({ color: Cesium.Color.fromCssColorString("#17364a").withAlpha(0.72), dashLength: 12 }),
        arcType: Cesium.ArcType.NONE,
      },
    });
    servingSite3d = viewer3d.entities.add({
      name: "Serving base station",
      position: initialPosition,
      point: {
        pixelSize: 16,
        color: Cesium.Color.fromCssColorString("#ff7658").withAlpha(0.3),
        outlineColor: Cesium.Color.fromCssColorString("#ff7658"),
        outlineWidth: 3,
        disableDepthTestDistance: Number.POSITIVE_INFINITY,
      },
    });
    setFixedThirdPersonCamera(initial);
    window.UAM_DASHBOARD_QA = {
      camera: () => ({
        heading_deg: Cesium.Math.toDegrees(viewer3d.camera.heading),
        pitch_deg: Cesium.Math.toDegrees(viewer3d.camera.pitch),
        roll_deg: Cesium.Math.toDegrees(viewer3d.camera.roll),
      }),
      scene: () => ({ stations: data.stations.length, trace_samples: trace.length, index }),
    };
  }

  function addVertiport3D(lon, lat, label, description) {
    viewer3d.entities.add({
      name: label,
      description,
      position: Cesium.Cartesian3.fromDegrees(lon, lat, 3),
      ellipse: {
        semiMajorAxis: 115,
        semiMinorAxis: 115,
        material: Cesium.Color.fromCssColorString("#17364a").withAlpha(0.72),
        outline: true,
        outlineColor: Cesium.Color.WHITE,
      },
      label: {
        text: label,
        font: "700 12px system-ui",
        fillColor: Cesium.Color.WHITE,
        showBackground: true,
        backgroundColor: Cesium.Color.fromCssColorString("#17364a").withAlpha(0.86),
        pixelOffset: new Cesium.Cartesian2(0, -24),
        distanceDisplayCondition: new Cesium.DistanceDisplayCondition(0, 26000),
      },
    });
  }

  function addGeographicLabels() {
    [
      ["South San Francisco", -122.407, 37.655],
      ["San Mateo", -122.326, 37.564],
      ["Redwood City", -122.236, 37.485],
      ["Palo Alto", -122.164, 37.443],
      ["Mountain View", -122.079, 37.395],
      ["Sunnyvale", -122.030, 37.378],
    ].forEach(([name, lon, lat]) => viewer3d.entities.add({
      name,
      position: Cesium.Cartesian3.fromDegrees(lon, lat, 8),
      point: { pixelSize: 5, color: Cesium.Color.WHITE, outlineColor: Cesium.Color.fromCssColorString("#17364a"), outlineWidth: 2 },
      label: {
        text: name,
        font: "600 11px system-ui",
        fillColor: Cesium.Color.fromCssColorString("#17364a"),
        showBackground: true,
        backgroundColor: Cesium.Color.WHITE.withAlpha(0.78),
        pixelOffset: new Cesium.Cartesian2(0, -16),
        distanceDisplayCondition: new Cesium.DistanceDisplayCondition(0, 18000),
      },
    }));
  }

  function addStation3D(site) {
    const form = site.physical_form.toLowerCase();
    if (site.site_class === "Macro_Building") addBuildingMacro3D(site);
    else if (site.site_class === "Macro_Other") addWaterTowerMacro3D(site);
    else if (form.includes("tree") || form.includes("monopine") || form.includes("eucalyptus")) addTreeMacro3D(site);
    else addTowerMacro3D(site);
  }

  function stationDescription(site, renderedForm) {
    return `<strong>${site.id}</strong><br>${site.address}<br>${site.physical_form}<br>${site.height_m.toFixed(2)} m AGL<br>${site.operator}<br><small>${renderedForm}; support dimensions are schematic.</small>`;
  }

  function stationLabel(site) {
    return {
      text: site.id,
      font: "700 11px system-ui",
      fillColor: Cesium.Color.WHITE,
      showBackground: true,
      backgroundColor: Cesium.Color.fromCssColorString("#17364a").withAlpha(0.82),
      pixelOffset: new Cesium.Cartesian2(0, -17),
      distanceDisplayCondition: new Cesium.DistanceDisplayCondition(0, 9500),
      disableDepthTestDistance: Number.POSITIVE_INFINITY,
    };
  }

  function addTowerMacro3D(site) {
    const mastHeight = Math.max(8, site.height_m - 2.5);
    viewer3d.entities.add({
      name: `${site.id} · tower macro`,
      description: stationDescription(site, "tower and antenna geometry"),
      position: Cesium.Cartesian3.fromDegrees(site.lon, site.lat, mastHeight / 2),
      cylinder: { length: mastHeight, topRadius: 0.48, bottomRadius: 0.85, material: Cesium.Color.fromCssColorString("#58656c"), shadows: Cesium.ShadowMode.ENABLED },
    });
    addAntennaCrown3D(site, Cesium.Color.fromCssColorString("#dce4e5"));
  }

  function addTreeMacro3D(site) {
    const crownHeight = Math.max(8, site.height_m * 0.44);
    viewer3d.entities.add({
      name: `${site.id} · concealed tree macro`,
      description: stationDescription(site, "concealed-tree macro geometry"),
      position: Cesium.Cartesian3.fromDegrees(site.lon, site.lat, site.height_m / 2),
      cylinder: { length: site.height_m, topRadius: 0.45, bottomRadius: 0.72, material: Cesium.Color.fromCssColorString("#665746"), shadows: Cesium.ShadowMode.ENABLED },
    });
    viewer3d.entities.add({
      position: Cesium.Cartesian3.fromDegrees(site.lon, site.lat, site.height_m - crownHeight / 2),
      cylinder: { length: crownHeight, topRadius: 1.1, bottomRadius: 5.8, material: Cesium.Color.fromCssColorString("#315c45").withAlpha(0.88), shadows: Cesium.ShadowMode.ENABLED },
      label: stationLabel(site),
    });
  }

  function addBuildingMacro3D(site) {
    const buildingHeight = Math.max(9, site.height_m - 3.5);
    viewer3d.entities.add({
      name: `${site.id} · building macro`,
      description: stationDescription(site, "rooftop/building support proxy and antenna geometry"),
      position: Cesium.Cartesian3.fromDegrees(site.lon, site.lat, buildingHeight / 2),
      box: {
        dimensions: new Cesium.Cartesian3(25, 19, buildingHeight),
        material: Cesium.Color.fromCssColorString("#b8b1a6").withAlpha(0.92),
        outline: true,
        outlineColor: Cesium.Color.fromCssColorString("#7b746a"),
        shadows: Cesium.ShadowMode.ENABLED,
      },
    });
    addAntennaCrown3D(site, Cesium.Color.fromCssColorString("#e4e8e8"));
  }

  function addWaterTowerMacro3D(site) {
    const stemHeight = site.height_m * 0.68;
    viewer3d.entities.add({
      name: `${site.id} · water-tower macro`,
      description: stationDescription(site, "water-tower support and antenna geometry"),
      position: Cesium.Cartesian3.fromDegrees(site.lon, site.lat, stemHeight / 2),
      cylinder: { length: stemHeight, topRadius: 2.1, bottomRadius: 3.4, material: Cesium.Color.fromCssColorString("#9fa8aa"), shadows: Cesium.ShadowMode.ENABLED },
    });
    viewer3d.entities.add({
      position: Cesium.Cartesian3.fromDegrees(site.lon, site.lat, stemHeight + 3.8),
      cylinder: { length: 7.6, topRadius: 6.2, bottomRadius: 4.8, material: Cesium.Color.fromCssColorString("#c8d0d1"), shadows: Cesium.ShadowMode.ENABLED },
    });
    addAntennaCrown3D(site, Cesium.Color.fromCssColorString("#e4e8e8"));
  }

  function addAntennaCrown3D(site, color) {
    const radiusM = 2.2;
    [0, 120, 240].forEach((angleDeg, panelIndex) => {
      const angle = Cesium.Math.toRadians(angleDeg);
      const eastM = Math.sin(angle) * radiusM;
      const northM = Math.cos(angle) * radiusM;
      const lat = site.lat + northM / 111320;
      const lon = site.lon + eastM / (111320 * Math.cos(Cesium.Math.toRadians(site.lat)));
      const position = Cesium.Cartesian3.fromDegrees(lon, lat, site.height_m - 1.8);
      viewer3d.entities.add({
        name: panelIndex === 0 ? `${site.id} antenna sector` : undefined,
        position,
        orientation: Cesium.Transforms.headingPitchRollQuaternion(position, new Cesium.HeadingPitchRoll(angle, 0, 0)),
        box: { dimensions: new Cesium.Cartesian3(0.55, 0.22, 3.6), material: color, outline: true, outlineColor: Cesium.Color.fromCssColorString("#627177") },
        label: panelIndex === 0 ? stationLabel(site) : undefined,
      });
    });
  }

  function aircraftOrientation(position, cursor) {
    return Cesium.Transforms.headingPitchRollQuaternion(position, new Cesium.HeadingPitchRoll(Cesium.Math.toRadians(headingAt(cursor)), 0, 0));
  }

  function setFixedThirdPersonCamera(row) {
    if (!viewer3d) return;
    const target = Cesium.Cartesian3.fromDegrees(row.lon, row.lat, Math.max(60, row.altitude_m * 0.55));
    viewer3d.camera.lookAt(target, new Cesium.HeadingPitchRange(Cesium.Math.toRadians(315), Cesium.Math.toRadians(-25), 3300));
    const viewport = document.getElementById("corridor-3d");
    viewport.dataset.cameraHeading = "315";
    viewport.dataset.cameraPitch = "-25";
    viewport.dataset.cameraMode = "translate-only";
  }

  function updateFlightRoute3D() {
    if (!flightRoute3d) return;
    flightRoute3d.polyline.positions = Cesium.Cartesian3.fromDegreesArrayHeights(trace.flatMap((row) => [row.lon, row.lat, row.altitude_m]));
  }

  function updateFrame3D(cursor) {
    if (!viewer3d || !aircraft3d) return;
    const row = trace[cursor];
    const site = stationById.get(row.serving);
    const aircraftPosition = Cesium.Cartesian3.fromDegrees(row.lon, row.lat, row.altitude_m);
    aircraft3d.position = aircraftPosition;
    aircraft3d.orientation = aircraftOrientation(aircraftPosition, cursor);
    aircraft3d.label.text = `UAM · ${row.altitude_m.toFixed(0)} m`;
    altitudeLine3d.polyline.positions = [Cesium.Cartesian3.fromDegrees(row.lon, row.lat, 0), aircraftPosition];
    if (site) {
      const siteTop = Cesium.Cartesian3.fromDegrees(site.lon, site.lat, site.height_m);
      servingLink3d.polyline.positions = [aircraftPosition, siteTop];
      servingSite3d.position = siteTop;
      servingSite3d.name = `Serving base station · ${site.id}`;
    }
    setFixedThirdPersonCamera(row);
  }

  function formatTime(seconds) {
    const whole = Math.max(0, Math.round(seconds));
    return `${String(Math.floor(whole / 60)).padStart(2, "0")}:${String(whole % 60).padStart(2, "0")}`;
  }

  function numericDomain(values) {
    return [Math.floor(Math.min(...values) - 1), Math.ceil(Math.max(...values) + 1)];
  }

  function buildHandoffCounts(rows) {
    let count = 0;
    return rows.map((row) => { if (row.handoff) count += 1; return count; });
  }

  function parametersFromControls() {
    return {
      speed: Number(el["flight-speed"].value),
      altitude: Number(el["flight-height"].value),
      offset: Number(el["flight-offset"].value),
    };
  }

  function updateParameterLabels() {
    const params = parametersFromControls();
    el["flight-speed-value"].textContent = `${params.speed} m/s`;
    el["flight-height-value"].textContent = `${params.altitude} m`;
    el["flight-offset-value"].textContent = `${params.offset > 0 ? "+" : ""}${params.offset} m`;
  }

  function isDefault(params) {
    return params.speed === defaults.speed && params.altitude === defaults.altitude && params.offset === defaults.offset;
  }

  function interpolateCenterline(sM) {
    if (sM <= 0) return { ...baseTrace[0], tangentX: baseTrace[1].x_m - baseTrace[0].x_m, tangentY: baseTrace[1].y_m - baseTrace[0].y_m };
    if (sM >= corridorLengthM) {
      const last = baseTrace.length - 1;
      return { ...baseTrace[last], tangentX: baseTrace[last].x_m - baseTrace[last - 1].x_m, tangentY: baseTrace[last].y_m - baseTrace[last - 1].y_m };
    }
    let low = 0;
    let high = baseTrace.length - 1;
    while (high - low > 1) {
      const middle = Math.floor((low + high) / 2);
      if (baseTrace[middle].s_km * 1000 <= sM) low = middle; else high = middle;
    }
    const a = baseTrace[low];
    const b = baseTrace[high];
    const span = b.s_km * 1000 - a.s_km * 1000;
    const fraction = span > 0 ? (sM - a.s_km * 1000) / span : 0;
    return {
      x_m: a.x_m + fraction * (b.x_m - a.x_m),
      y_m: a.y_m + fraction * (b.y_m - a.y_m),
      lat: a.lat + fraction * (b.lat - a.lat),
      lon: a.lon + fraction * (b.lon - a.lon),
      tangentX: b.x_m - a.x_m,
      tangentY: b.y_m - a.y_m,
    };
  }

  function evaluateRadio(xM, yM, zM) {
    const received = [];
    let servingIndex = 0;
    let strongest = -Infinity;
    data.stations.forEach((site, i) => {
      const dx = xM - site.x_m;
      const dy = yM - site.y_m;
      const dz = zM - site.height_m;
      const d2 = Math.max(dx * dx + dy * dy + dz * dz, Number.MIN_VALUE);
      const rx = Number(radio.eirp_dbm) + Number(radio.receiver_gain_db) - 28 - 11 * Math.log10(d2) - 20 * Math.log10(Number(radio.frequency_ghz));
      received.push(rx);
      if (rx > strongest) { strongest = rx; servingIndex = i; }
    });
    const powersMw = received.map((value) => 10 ** (value / 10));
    const desiredMw = powersMw[servingIndex];
    const interferenceMw = Math.max(powersMw.reduce((sum, value) => sum + value, 0) - desiredMw, Number.MIN_VALUE);
    const noiseMw = 10 ** (Number(radio.noise_dbm) / 10);
    return {
      serving: data.stations[servingIndex].id,
      rsrp: strongest - 10 * Math.log10(Number(radio.resource_elements)),
      sinr: 10 * Math.log10(desiredMw / (interferenceMw + noiseMw)),
    };
  }

  function buildPreview(params) {
    if (isDefault(params)) return baseTrace.map((row) => ({ ...row }));
    const dt = Number(data.summary.clock.dt_radio_s);
    const duration = corridorLengthM / params.speed;
    const times = [];
    for (let t = 0; t <= duration + 1e-9; t += dt) times.push(Math.min(t, duration));
    if (times.at(-1) < duration - 1e-9) times.push(duration);
    const rows = [];
    let previousServing = null;
    times.forEach((t) => {
      const sM = Math.min(params.speed * t, corridorLengthM);
      const center = interpolateCenterline(sM);
      const norm = Math.hypot(center.tangentX, center.tangentY) || 1;
      const normalX = -center.tangentY / norm;
      const normalY = center.tangentX / norm;
      const terminalFactorRaw = Math.max(0, Math.min(1, sM / 2000, (corridorLengthM - sM) / 2000));
      const terminalFactor = terminalFactorRaw * terminalFactorRaw * (3 - 2 * terminalFactorRaw);
      const localOffset = params.offset * terminalFactor;
      const xM = center.x_m + localOffset * normalX;
      const yM = center.y_m + localOffset * normalY;
      const lat = center.lat + localOffset * normalY / 111320;
      const lon = center.lon + localOffset * normalX / (111320 * Math.cos(center.lat * Math.PI / 180));
      const link = evaluateRadio(xM, yM, params.altitude);
      rows.push({
        t, s_km: sM / 1000, x_m: xM, y_m: yM, lat, lon,
        altitude_m: params.altitude, serving: link.serving, rsrp: link.rsrp, sinr: link.sinr,
        handoff: previousServing !== null && link.serving !== previousServing,
        control_tick: Math.abs(t / Number(data.summary.clock.dt_control_s) - Math.round(t / Number(data.summary.clock.dt_control_s))) < 1e-7,
      });
      previousServing = link.serving;
    });
    return rows;
  }

  function headingAt(i) {
    const a = trace[Math.max(0, i - 1)];
    const b = trace[Math.min(trace.length - 1, i + 1)];
    return Math.atan2(b.lon - a.lon, b.lat - a.lat) * 180 / Math.PI;
  }

  function updateStationStyle(servingId) {
    if (lastServingId && stationMarkers.has(lastServingId)) {
      const oldSite = stationById.get(lastServingId);
      stationMarkers.get(lastServingId).setStyle({ radius: 5, weight: 1.5, fillColor: oldSite.site_class === "Macro_Building" ? "#d6a545" : "#17364a" });
    }
    const marker = stationMarkers.get(servingId);
    if (marker) marker.setStyle({ radius: 9, weight: 3, fillColor: "#e46f51" }).bringToFront();
    lastServingId = servingId;
  }

  function applyParameters(params) {
    playing = false;
    el["play-button"].textContent = "▶";
    el["play-button"].setAttribute("aria-label", "Play simulation");
    activeParameters = params;
    trace = buildPreview(params);
    cumulativeHandoffs = buildHandoffCounts(trace);
    rsrpValues = trace.map((row) => row.rsrp);
    sinrValues = trace.map((row) => row.sinr);
    rsrpDomain = numericDomain(rsrpValues);
    sinrDomain = numericDomain(sinrValues);
    flightRoute.setLatLngs(trace.map((row) => [row.lat, row.lon]));
    updateFlightRoute3D();
    el["time-slider"].max = String(trace.length - 1);
    el["total-time"].textContent = `/ ${formatTime(trace.at(-1).t)}`;
    el["rsrp-range"].textContent = `${Math.min(...rsrpValues).toFixed(1)} to ${Math.max(...rsrpValues).toFixed(1)} dBm/RE`;
    el["sinr-range"].textContent = `${Math.min(...sinrValues).toFixed(1)} to ${Math.max(...sinrValues).toFixed(1)} dB`;
    el["three-parameter-label"].textContent = `${params.altitude} m level · ${params.offset > 0 ? "+" : ""}${params.offset} m offset`;
    el["preview-status"].textContent = isDefault(params) ? "Baseline parameters active" : `Interactive preview · ${trace.length.toLocaleString()} radio samples`;
    el["run-status"].textContent = isDefault(params) ? "Validated baseline" : "Interactive preview";
    simulatedClock = 0;
    updateFrame(0);
  }

  function updateFrame(nextIndex) {
    index = Math.max(0, Math.min(trace.length - 1, Math.round(nextIndex)));
    const row = trace[index];
    const site = stationById.get(row.serving);
    const position = [row.lat, row.lon];
    aircraft.setLatLng(position);
    const glyph = aircraft.getElement()?.querySelector(".uam-marker");
    if (glyph) glyph.style.transform = `rotate(${headingAt(index)}deg)`;
    if (site) servingLine.setLatLngs([position, [site.lat, site.lon]]);
    updateStationStyle(row.serving);
    el["time-slider"].value = String(index);
    el["current-time"].textContent = formatTime(row.t);
    el["progress-km"].textContent = `${row.s_km.toFixed(2)} km`;
    el["progress-percent"].textContent = `${(100 * row.s_km / data.summary.corridor_length_km).toFixed(1)}%`;
    el["serving-site"].textContent = row.serving;
    el["serving-form"].textContent = site?.physical_form || "Unknown form";
    el["rsrp-value"].textContent = row.rsrp.toFixed(1);
    el["sinr-value"].textContent = row.sinr.toFixed(1);
    el["handoff-count"].textContent = String(cumulativeHandoffs[index]);
    if (site) {
      el["site-address"].textContent = site.address;
      el["site-class"].textContent = `${site.site_class} · ${site.physical_form}`;
      el["site-operator"].textContent = site.operator;
      el["site-height"].textContent = `${site.height_m.toFixed(2)} m AGL (${site.height_basis})`;
    }
    drawChart(rsrpCanvas, rsrpValues, rsrpDomain, index, "#168c85", "dBm/RE");
    drawChart(sinrCanvas, sinrValues, sinrDomain, index, "#e46f51", "dB");
    updateFrame3D(index);
  }

  function drawChart(canvas, values, domain, cursor, color, unit) {
    const ratio = window.devicePixelRatio || 1;
    const width = Math.max(280, canvas.clientWidth);
    const height = 130;
    canvas.width = Math.round(width * ratio); canvas.height = Math.round(height * ratio);
    const ctx = canvas.getContext("2d");
    ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
    const pad = { left: 34, right: 8, top: 8, bottom: 20 };
    const [min, max] = domain;
    const xAt = (i) => pad.left + i / Math.max(1, values.length - 1) * (width - pad.left - pad.right);
    const yAt = (value) => pad.top + (max - value) / Math.max(1e-9, max - min) * (height - pad.top - pad.bottom);
    ctx.strokeStyle = "#e3e6e3"; ctx.fillStyle = "#6c7880"; ctx.font = "10px system-ui"; ctx.textAlign = "right";
    for (let step = 0; step <= 2; step += 1) {
      const value = min + (max - min) * step / 2; const y = yAt(value);
      ctx.beginPath(); ctx.moveTo(pad.left, y); ctx.lineTo(width - pad.right, y); ctx.stroke(); ctx.fillText(value.toFixed(0), pad.left - 5, y + 3);
    }
    ctx.beginPath();
    values.forEach((value, i) => { const x = xAt(i); const y = yAt(value); if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y); });
    ctx.strokeStyle = color; ctx.lineWidth = 1.7; ctx.stroke();
    const cursorX = xAt(cursor);
    ctx.beginPath(); ctx.moveTo(cursorX, pad.top); ctx.lineTo(cursorX, height - pad.bottom);
    ctx.strokeStyle = "#17364a"; ctx.lineWidth = 1; ctx.setLineDash([3, 3]); ctx.stroke(); ctx.setLineDash([]);
    ctx.beginPath(); ctx.arc(cursorX, yAt(values[cursor]), 3.5, 0, Math.PI * 2); ctx.fillStyle = color; ctx.fill();
    ctx.fillStyle = "#6c7880"; ctx.textAlign = "left"; ctx.fillText("SF", pad.left, height - 4);
    ctx.textAlign = "right"; ctx.fillText(`SJ · ${unit}`, width - pad.right, height - 4);
  }

  function animationFrame(timestamp) {
    if (!playing) return;
    if (lastFrameTime === null) lastFrameTime = timestamp;
    simulatedClock += (timestamp - lastFrameTime) / 1000 * Number(el["speed-select"].value);
    lastFrameTime = timestamp;
    while (index < trace.length - 1 && trace[index + 1].t <= simulatedClock) index += 1;
    updateFrame(index);
    if (index >= trace.length - 1) {
      playing = false; el["play-button"].textContent = "▶"; el["play-button"].setAttribute("aria-label", "Play simulation"); return;
    }
    requestAnimationFrame(animationFrame);
  }

  ["flight-speed", "flight-height", "flight-offset"].forEach((id) => el[id].addEventListener("input", updateParameterLabels));
  el["apply-parameters"].addEventListener("click", () => applyParameters(parametersFromControls()));
  el["reset-parameters"].addEventListener("click", () => {
    el["flight-speed"].value = String(defaults.speed);
    el["flight-height"].value = String(defaults.altitude);
    el["flight-offset"].value = String(defaults.offset);
    updateParameterLabels(); applyParameters({ ...defaults });
  });
  el["play-button"].addEventListener("click", () => {
    playing = !playing;
    el["play-button"].textContent = playing ? "❚❚" : "▶";
    el["play-button"].setAttribute("aria-label", playing ? "Pause simulation" : "Play simulation");
    if (playing) {
      if (index >= trace.length - 1) { index = 0; simulatedClock = 0; }
      lastFrameTime = null; requestAnimationFrame(animationFrame);
    }
  });
  el["reset-button"].addEventListener("click", () => { playing = false; el["play-button"].textContent = "▶"; simulatedClock = 0; updateFrame(0); });
  el["time-slider"].addEventListener("input", () => { index = Number(el["time-slider"].value); simulatedClock = trace[index].t; updateFrame(index); });
  el["reset-3d-camera"].addEventListener("click", () => setFixedThirdPersonCamera(trace[index]));
  window.addEventListener("resize", () => { map.invalidateSize(); updateFrame(index); });

  updateParameterLabels();
  applyParameters({ ...defaults });
})();
