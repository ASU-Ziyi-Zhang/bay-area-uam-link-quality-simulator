(() => {
  "use strict";

  const data = window.UAM_DASHBOARD_DATA;
  const errorPanel = document.getElementById("error-panel");
  if (!data || !window.L) {
    errorPanel.hidden = false;
    // Leaflet is vendored, so a failure here is a packaging problem rather than
    // a network one; keep the two causes distinguishable.
    errorPanel.textContent = !data
      ? "Dashboard data bundle is missing. Run scripts/build_dashboard.py to regenerate dashboard/data/dashboard_data.js."
      : "The bundled map library (vendor/leaflet/leaflet.js) did not load. Serve the whole dashboard directory rather than the HTML file on its own.";
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
  const servedSetSize = radio.served_set_size === null || radio.served_set_size === undefined
    ? null
    : Number(radio.served_set_size);
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
    "three-camera-state", "three-camera-note",
    "site-source", "model-parameters", "model-caveat", "sinr-caption",
  ];
  const el = Object.fromEntries(ids.map((id) => [id, document.getElementById(id)]));
  const rsrpCanvas = document.getElementById("rsrp-chart");
  const sinrCanvas = document.getElementById("sinr-chart");

  // Everything here is derived from the same summary.radio block that drives
  // evaluateRadio(), so the printed assumptions cannot drift from the model.
  function renderModelParameters() {
    const reOffsetDb = 10 * Math.log10(Number(radio.resource_elements));
    const rows = [
      ["Carrier", `${Number(radio.frequency_ghz).toFixed(1)} GHz`],
      ["Site EIRP", `${Number(radio.eirp_dbm).toFixed(0)} dBm (full carrier)`],
      ["Receiver gain", `${Number(radio.receiver_gain_db).toFixed(0)} dB`],
      ["Noise", `${Number(radio.noise_dbm).toFixed(0)} dBm (full carrier)`],
      ["Resource elements", `${radio.resource_elements} · RSRP = full carrier − ${reOffsetDb.toFixed(2)} dB`],
      ["Path loss", "28.0 + 22·log₁₀(d₃D / m) + 20·log₁₀(f / GHz)"],
      ["Reference", "3GPP TR 36.777 UMa-AV LOS"],
      ["Served set", servedSetSize === null
        ? `all ${data.stations.length} sites`
        : `${servedSetSize} nearest sites (equivalently the ${servedSetSize} strongest)`],
      ["Association", "strongest of the served set, no hysteresis or offset"],
      ["Interference", servedSetSize === null
        ? `all ${data.stations.length} sites co-channel at full EIRP (${data.stations.length - 1} interferers)`
        : `remaining ${servedSetSize - 1} served sites, co-channel at full EIRP`],
      ["Antenna heights", "per site; see the serving-site record above"],
    ];
    el["model-parameters"].replaceChildren(...rows.map(([term, value]) => {
      const wrap = document.createElement("div");
      const dt = document.createElement("dt");
      dt.textContent = term;
      const dd = document.createElement("dd");
      dd.textContent = value;
      wrap.append(dt, dd);
      return wrap;
    }));
    el["model-caveat"].textContent =
      "Deterministic LOS planning kernel. No antenna pattern, sector orientation, "
      + "downtilt, frequency reuse, traffic loading, shadow fading, or terrain "
      + "diffraction is applied. SINR is a planning estimate, not a predicted "
      + "service level.";
    const interferers = servedSetSize === null ? data.stations.length - 1 : servedSetSize - 1;
    el["sinr-caption"].textContent = `Co-channel · ${interferers} interferer${interferers === 1 ? "" : "s"}`;
  }

  renderModelParameters();
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
  let rafId = null;

  const CAMERA_HEADING_DEG = 315;
  const CAMERA_PITCH_DEG = -25;
  const CAMERA_RANGE_M = 3300;
  let cameraMode = "follow";

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

    const canvas3d = viewer3d.scene.canvas;
    canvas3d.addEventListener("pointerdown", releaseCameraToFree);
    canvas3d.addEventListener("wheel", releaseCameraToFree, { passive: true });

    window.UAM_DASHBOARD_QA = {
      camera: () => {
        const carto = Cesium.Cartographic.fromCartesian(viewer3d.camera.positionWC);
        return {
          mode: cameraMode,
          heading_deg: Cesium.Math.toDegrees(viewer3d.camera.heading),
          pitch_deg: Cesium.Math.toDegrees(viewer3d.camera.pitch),
          roll_deg: Cesium.Math.toDegrees(viewer3d.camera.roll),
          lat: Cesium.Math.toDegrees(carto.latitude),
          lon: Cesium.Math.toDegrees(carto.longitude),
          height_m: carto.height,
        };
      },
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

  function annotateCameraViewport() {
    const viewport = document.getElementById("corridor-3d");
    viewport.dataset.cameraMode = cameraMode === "follow" ? "translate-only" : "free";
    if (cameraMode === "follow") {
      viewport.dataset.cameraHeading = String(CAMERA_HEADING_DEG);
      viewport.dataset.cameraPitch = String(CAMERA_PITCH_DEG);
    } else {
      delete viewport.dataset.cameraHeading;
      delete viewport.dataset.cameraPitch;
    }
    el["three-camera-state"].textContent = cameraMode === "follow"
      ? `${CAMERA_HEADING_DEG}° bearing · fixed pitch`
      : "Free view";
    el["three-camera-note"].textContent = cameraMode === "follow"
      ? "Fixed bearing and pitch · camera translates with position only."
      : "Free view · drag to orbit, scroll to zoom · Recenter restores the fixed bearing.";
  }

  // Follow mode reasserts the fixed third-person framing on every frame. Any
  // user gesture on the canvas releases the lookAt transform so the camera can
  // be driven freely; Recenter puts it back under follow control.
  function applyFollowCamera(row) {
    const target = Cesium.Cartesian3.fromDegrees(row.lon, row.lat, Math.max(60, row.altitude_m * 0.55));
    viewer3d.camera.lookAt(target, new Cesium.HeadingPitchRange(
      Cesium.Math.toRadians(CAMERA_HEADING_DEG), Cesium.Math.toRadians(CAMERA_PITCH_DEG), CAMERA_RANGE_M,
    ));
  }

  function setFixedThirdPersonCamera(row) {
    if (!viewer3d) return;
    cameraMode = "follow";
    applyFollowCamera(row);
    annotateCameraViewport();
  }

  function releaseCameraToFree() {
    if (!viewer3d || cameraMode === "free") return;
    cameraMode = "free";
    // Detach from the aircraft-local reference frame without moving the camera.
    viewer3d.camera.lookAtTransform(Cesium.Matrix4.IDENTITY);
    annotateCameraViewport();
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
    if (cameraMode === "follow") applyFollowCamera(row);
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

  // Mirrors compute_link_state() in src/capacity_policy/radio.py, including the
  // served-set restriction: the UAM associates with and receives from only the
  // nearest summary.radio.served_set_size sites.
  function evaluateRadio(xM, yM, zM) {
    const links = data.stations.map((site, i) => {
      const dx = xM - site.x_m;
      const dy = yM - site.y_m;
      const dz = zM - site.height_m;
      const d2 = Math.max(dx * dx + dy * dy + dz * dz, Number.MIN_VALUE);
      const rx = Number(radio.eirp_dbm) + Number(radio.receiver_gain_db) - 28 - 11 * Math.log10(d2) - 20 * Math.log10(Number(radio.frequency_ghz));
      return { i, d2, rx };
    });

    const served = servedSetSize !== null && servedSetSize < links.length
      ? [...links].sort((a, b) => a.d2 - b.d2).slice(0, servedSetSize)
      : links;

    let serving = served[0];
    served.forEach((link) => { if (link.rx > serving.rx) serving = link; });
    const desiredMw = 10 ** (serving.rx / 10);
    const totalMw = served.reduce((sum, link) => sum + 10 ** (link.rx / 10), 0);
    const interferenceMw = Math.max(totalMw - desiredMw, Number.MIN_VALUE);
    const noiseMw = 10 ** (Number(radio.noise_dbm) / 10);
    return {
      serving: data.stations[serving.i].id,
      rsrp: serving.rx - 10 * Math.log10(Number(radio.resource_elements)),
      sinr: 10 * Math.log10(desiredMw / (interferenceMw + noiseMw)),
    };
  }

  // Every trace shown by the dashboard — including the default one — comes out
  // of this function. tests/test_dashboard_preview.py pins it against the
  // Python kernel in src/capacity_policy/radio.py so the default path stays
  // numerically identical to the validated baseline.
  function buildPreview(params) {
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
    // A degree of longitude spans cos(latitude) of a degree of latitude on the
    // ground, so the east component must be scaled before taking the bearing.
    const midLat = (a.lat + b.lat) / 2 * Math.PI / 180;
    return Math.atan2((b.lon - a.lon) * Math.cos(midLat), b.lat - a.lat) * 180 / Math.PI;
  }

  // The register keeps an official source URL per site; surface it as a link so
  // the on-screen claim can be traced without opening the CSV.
  function renderSiteSource(site) {
    const url = site.official_source_url;
    if (!url) {
      el["site-source"].textContent = "no public source URL retained";
      return;
    }
    let host;
    try {
      const parsed = new URL(url);
      if (parsed.protocol !== "http:" && parsed.protocol !== "https:") throw new Error("scheme");
      host = parsed.hostname.replace(/^www\./, "");
    } catch {
      el["site-source"].textContent = "source URL is not resolvable";
      return;
    }
    const link = document.createElement("a");
    link.href = url;
    link.target = "_blank";
    link.rel = "noopener noreferrer";
    link.textContent = host;
    link.title = url;
    el["site-source"].replaceChildren(link);
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
    setPlaying(false);
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
      renderSiteSource(site);
    }
    drawChart(rsrpCanvas, rsrpValues, rsrpDomain, index, "#168c85", "dBm/RE");
    drawChart(sinrCanvas, sinrValues, sinrDomain, index, "#e46f51", "dB");
    updateFrame3D(index);
  }

  const CHART_HEIGHT = 130;
  const CHART_PAD = { left: 34, right: 8, top: 8, bottom: 20 };
  const chartLayers = new Map();

  // The trace, gridlines, and axis labels are static between parameter changes,
  // so they are rendered once into an offscreen surface and reused.
  function chartLayer(canvas, values, domain, color, unit) {
    const ratio = window.devicePixelRatio || 1;
    const width = Math.max(280, canvas.clientWidth);
    const cached = chartLayers.get(canvas);
    if (cached && cached.values === values && cached.domain === domain
      && cached.width === width && cached.ratio === ratio) return cached;

    const [min, max] = domain;
    const xAt = (i) => CHART_PAD.left + i / Math.max(1, values.length - 1) * (width - CHART_PAD.left - CHART_PAD.right);
    const yAt = (value) => CHART_PAD.top + (max - value) / Math.max(1e-9, max - min) * (CHART_HEIGHT - CHART_PAD.top - CHART_PAD.bottom);

    const surface = cached?.surface || document.createElement("canvas");
    surface.width = Math.round(width * ratio);
    surface.height = Math.round(CHART_HEIGHT * ratio);
    const ctx = surface.getContext("2d");
    ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
    ctx.clearRect(0, 0, width, CHART_HEIGHT);
    ctx.strokeStyle = "#e3e6e3"; ctx.fillStyle = "#6c7880"; ctx.font = "10px system-ui"; ctx.textAlign = "right";
    for (let step = 0; step <= 2; step += 1) {
      const value = min + (max - min) * step / 2; const y = yAt(value);
      ctx.beginPath(); ctx.moveTo(CHART_PAD.left, y); ctx.lineTo(width - CHART_PAD.right, y); ctx.stroke();
      ctx.fillText(value.toFixed(0), CHART_PAD.left - 5, y + 3);
    }
    ctx.beginPath();
    values.forEach((value, i) => { const x = xAt(i); const y = yAt(value); if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y); });
    ctx.strokeStyle = color; ctx.lineWidth = 1.7; ctx.stroke();
    ctx.fillStyle = "#6c7880"; ctx.textAlign = "left"; ctx.fillText("SF", CHART_PAD.left, CHART_HEIGHT - 4);
    ctx.textAlign = "right"; ctx.fillText(`SJ · ${unit}`, width - CHART_PAD.right, CHART_HEIGHT - 4);

    const layer = { surface, values, domain, width, ratio, xAt, yAt };
    chartLayers.set(canvas, layer);
    return layer;
  }

  function drawChart(canvas, values, domain, cursor, color, unit) {
    const layer = chartLayer(canvas, values, domain, color, unit);
    if (canvas.width !== layer.surface.width || canvas.height !== layer.surface.height) {
      canvas.width = layer.surface.width;
      canvas.height = layer.surface.height;
    }
    const ctx = canvas.getContext("2d");
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.drawImage(layer.surface, 0, 0);
    ctx.setTransform(layer.ratio, 0, 0, layer.ratio, 0, 0);
    const cursorX = layer.xAt(cursor);
    ctx.beginPath(); ctx.moveTo(cursorX, CHART_PAD.top); ctx.lineTo(cursorX, CHART_HEIGHT - CHART_PAD.bottom);
    ctx.strokeStyle = "#17364a"; ctx.lineWidth = 1; ctx.setLineDash([3, 3]); ctx.stroke(); ctx.setLineDash([]);
    ctx.beginPath(); ctx.arc(cursorX, layer.yAt(values[cursor]), 3.5, 0, Math.PI * 2); ctx.fillStyle = color; ctx.fill();
  }

  // Single source of truth for the transport state, so the button glyph and its
  // accessible name can never drift apart and only one rAF loop can be live.
  function setPlaying(next) {
    playing = next;
    el["play-button"].textContent = next ? "❚❚" : "▶";
    el["play-button"].setAttribute("aria-label", next ? "Pause simulation" : "Play simulation");
    if (rafId !== null) { cancelAnimationFrame(rafId); rafId = null; }
    if (next) { lastFrameTime = null; rafId = requestAnimationFrame(animationFrame); }
  }

  function animationFrame(timestamp) {
    rafId = null;
    if (!playing) return;
    if (lastFrameTime === null) lastFrameTime = timestamp;
    simulatedClock += (timestamp - lastFrameTime) / 1000 * Number(el["speed-select"].value);
    lastFrameTime = timestamp;
    while (index < trace.length - 1 && trace[index + 1].t <= simulatedClock) index += 1;
    updateFrame(index);
    if (index >= trace.length - 1) { setPlaying(false); return; }
    rafId = requestAnimationFrame(animationFrame);
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
    if (!playing && index >= trace.length - 1) { index = 0; simulatedClock = 0; }
    setPlaying(!playing);
  });
  el["reset-button"].addEventListener("click", () => { setPlaying(false); simulatedClock = 0; updateFrame(0); });
  el["time-slider"].addEventListener("input", () => { index = Number(el["time-slider"].value); simulatedClock = trace[index].t; updateFrame(index); });
  el["reset-3d-camera"].addEventListener("click", () => setFixedThirdPersonCamera(trace[index]));

  let resizeTimer = null;
  window.addEventListener("resize", () => {
    if (resizeTimer !== null) clearTimeout(resizeTimer);
    resizeTimer = setTimeout(() => { resizeTimer = null; map.invalidateSize(); updateFrame(index); }, 120);
  });

  updateParameterLabels();
  applyParameters({ ...defaults });
})();
