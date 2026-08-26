(() => {
  "use strict";

  const data = window.UAM_TRAFFIC_DATA;
  const errorPanel = document.getElementById("error-panel");
  if (!data || !window.L) {
    errorPanel.hidden = false;
    errorPanel.textContent = !data
      ? "Traffic data bundle is missing. Rebuild it with scripts/build_traffic_dashboard.py."
      : "The bundled Leaflet map library did not load.";
    return;
  }

  const colors = { C: "#238b57", R: "#e3b735", F: "#d65353", U: "#829095" };
  const frames = data.frames;
  const entrants = data.entrants;
  const route = data.route_metric;
  const summary = data.summary;
  const display = summary.display || {};
  const speedMps = Number(summary.trajectory.speed_mps);
  const altitudeM = Number(summary.trajectory.altitude_m);
  const corridorLengthM = Number(summary.corridor_length_km) * 1000;
  const stationById = new Map(data.stations.map((site) => [site.id, site]));

  const ids = [
    "traffic-title", "traffic-subtitle", "offered-demand", "entry-interval", "traffic-speed", "traffic-altitude",
    "play-button", "reset-button", "time-slider", "playback-speed", "current-time", "total-time",
    "active-count", "expected-count", "current-capacity", "reliability-capacity", "demand-status",
    "policy-observations", "share-c", "share-r", "share-f", "share-c-bar", "share-r-bar", "share-f-bar",
    "selected-uam", "selected-policy", "selected-exposure", "selected-site", "selected-rsrp", "selected-sinr", "selected-progress", "selected-group",
    "current-counts", "three-warning",
  ];
  const el = Object.fromEntries(ids.map((id) => [id, document.getElementById(id)]));
  const capacityCanvas = document.getElementById("capacity-chart");

  el["traffic-title"].textContent = `${display.route_label || summary.scenario_id} · Multi-UAM Policy`;
  el["traffic-subtitle"].textContent = "Individual link quality → centered five-aircraft group policy → reliability-qualified capacity";
  el["offered-demand"].textContent = Number(summary.traffic.entry_demand_uam_h).toFixed(1);
  el["entry-interval"].textContent = Number(summary.traffic.entry_interval_s).toFixed(1);
  el["traffic-speed"].textContent = speedMps.toFixed(0);
  el["traffic-altitude"].textContent = altitudeM.toFixed(0);
  el["expected-count"].textContent = `steady-state expectation ${Number(summary.traffic.expected_steady_occupancy).toFixed(1)}`;
  el["reliability-capacity"].textContent = Number(summary.capacity.q_mix_rho_uam_h).toFixed(1);
  el["policy-observations"].textContent = `${Number(summary.policy.observation_count).toLocaleString()} observations`;

  ["C", "R", "F"].forEach((policy) => {
    const share = Number(summary.policy.shares[policy]);
    el[`share-${policy.toLowerCase()}`].textContent = `${(100 * share).toFixed(1)}%`;
    el[`share-${policy.toLowerCase()}-bar`].style.width = `${100 * share}%`;
  });
  const supported = Boolean(summary.capacity.demand_supported_by_q_mix_rho);
  el["demand-status"].textContent = supported ? "Yes" : "No";
  el["demand-status"].className = supported ? "supported" : "unsupported";

  function formatTime(seconds) {
    const value = Math.max(0, Math.round(seconds));
    const minutes = Math.floor(value / 60);
    return `${String(minutes).padStart(2, "0")}:${String(value % 60).padStart(2, "0")}`;
  }

  function interpolateRoute(sM) {
    if (sM <= 0) return { ...route[0], tangentX: route[1].x_m - route[0].x_m, tangentY: route[1].y_m - route[0].y_m };
    if (sM >= corridorLengthM) {
      const last = route.length - 1;
      return { ...route[last], tangentX: route[last].x_m - route[last - 1].x_m, tangentY: route[last].y_m - route[last - 1].y_m };
    }
    let low = 0;
    let high = route.length - 1;
    while (high - low > 1) {
      const middle = Math.floor((low + high) / 2);
      if (route[middle].s_m <= sM) low = middle; else high = middle;
    }
    const a = route[low];
    const b = route[high];
    const fraction = (sM - a.s_m) / (b.s_m - a.s_m);
    return {
      s_m: sM,
      x_m: a.x_m + fraction * (b.x_m - a.x_m),
      y_m: a.y_m + fraction * (b.y_m - a.y_m),
      lon: a.lon + fraction * (b.lon - a.lon),
      lat: a.lat + fraction * (b.lat - a.lat),
      tangentX: b.x_m - a.x_m,
      tangentY: b.y_m - a.y_m,
    };
  }

  function evaluateRadio(position) {
    const radio = summary.radio;
    const servedSetSize = radio.served_set_size == null ? null : Number(radio.served_set_size);
    const links = data.stations.map((site, index) => {
      const dx = position.x_m - site.x_m;
      const dy = position.y_m - site.y_m;
      const dz = altitudeM - site.height_m;
      const d2 = Math.max(dx * dx + dy * dy + dz * dz, Number.MIN_VALUE);
      const rx = Number(radio.eirp_dbm) + Number(radio.receiver_gain_db) - 28 - 11 * Math.log10(d2) - 20 * Math.log10(Number(radio.frequency_ghz));
      return { index, d2, rx };
    });
    const served = servedSetSize !== null && servedSetSize < links.length
      ? [...links].sort((a, b) => a.d2 - b.d2).slice(0, servedSetSize)
      : links;
    let serving = served[0];
    served.forEach((link) => { if (link.rx > serving.rx) serving = link; });
    const desiredMw = 10 ** (serving.rx / 10);
    const interferenceMw = Math.max(served.reduce((total, link) => total + 10 ** (link.rx / 10), 0) - desiredMw, Number.MIN_VALUE);
    const noiseMw = 10 ** (Number(radio.noise_dbm) / 10);
    return {
      site: data.stations[serving.index],
      rsrp: serving.rx - 10 * Math.log10(Number(radio.resource_elements)),
      sinr: 10 * Math.log10(desiredMw / (interferenceMw + noiseMw)),
    };
  }

  const map = L.map("traffic-map", { preferCanvas: true });
  L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
    maxZoom: 19,
    attribution: "&copy; OpenStreetMap contributors",
  }).addTo(map);
  const groundRoute = L.polyline(data.route.map(([lon, lat]) => [lat, lon]), { color: "#168c85", weight: 4, opacity: .85 }).addTo(map);
  map.fitBounds(groundRoute.getBounds(), { padding: [55, 55] });
  data.stations.forEach((site) => {
    L.circleMarker([site.lat, site.lon], { radius: 4, color: "#fff", weight: 1.2, fillColor: "#17364a", fillOpacity: .85 })
      .bindTooltip(`${site.id} · ${site.physical_form}`)
      .addTo(map);
  });
  const servingLine = L.polyline([], { color: "#e46f51", weight: 2, opacity: .7, dashArray: "5 5" }).addTo(map);
  const markers = new Map();
  let selectedUamId = null;

  function markerIcon(policy, headingDeg, selected, groupMember) {
    const color = colors[policy] || colors.U;
    return L.divIcon({
      className: "",
      html: `<div class="uam-traffic-marker${groupMember ? " uam-traffic-marker--group" : ""}${selected ? " uam-traffic-marker--selected" : ""}" style="background:${color};transform:rotate(${headingDeg + 45}deg)" aria-label="${policy} policy aircraft">✈</div>`,
      iconSize: selected ? [28, 28] : [22, 22],
      iconAnchor: selected ? [14, 14] : [11, 11],
    });
  }

  let viewer3d = null;
  const aircraft3d = new Map();
  function planeSvg(policy) {
    const color = colors[policy] || colors.U;
    return `data:image/svg+xml;charset=utf-8,${encodeURIComponent(`<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 64 64"><circle cx="32" cy="32" r="29" fill="${color}" stroke="white" stroke-width="4"/><path d="M29 11h6l4 17 15 8v5l-16-3-3 15h-6l-3-15-16 3v-5l15-8z" fill="white"/></svg>`)}`;
  }

  function initialize3d() {
    if (window.location.protocol === "file:" || !window.Cesium) {
      el["three-warning"].hidden = false;
      return;
    }
    viewer3d = new Cesium.Viewer("traffic-3d", {
      animation: false, timeline: false, geocoder: false, homeButton: false,
      sceneModePicker: false, baseLayerPicker: false, navigationHelpButton: false,
      fullscreenButton: false, selectionIndicator: false, infoBox: false,
      baseLayer: new Cesium.ImageryLayer(new Cesium.OpenStreetMapImageryProvider({ url: "https://tile.openstreetmap.org/" })),
    });
    viewer3d.scene.globe.depthTestAgainstTerrain = false;
    viewer3d.entities.add({
      polyline: { positions: Cesium.Cartesian3.fromDegreesArray(data.route.flat()), width: 3, material: Cesium.Color.fromCssColorString("#168c85") },
    });
    data.stations.forEach((site) => {
      viewer3d.entities.add({
        position: Cesium.Cartesian3.fromDegrees(site.lon, site.lat, site.height_m / 2),
        cylinder: { length: site.height_m, topRadius: 1.4, bottomRadius: 2.2, material: Cesium.Color.fromCssColorString("#17364a").withAlpha(.85) },
        point: { pixelSize: 7, color: Cesium.Color.fromCssColorString("#17364a"), outlineColor: Cesium.Color.WHITE, outlineWidth: 1, disableDepthTestDistance: Number.POSITIVE_INFINITY },
      });
    });
    const minLon = Math.min(...data.route.map((p) => p[0]));
    const minLat = Math.min(...data.route.map((p) => p[1]));
    const maxLon = Math.max(...data.route.map((p) => p[0]));
    const maxLat = Math.max(...data.route.map((p) => p[1]));
    const center = Cesium.Cartesian3.fromDegrees((minLon + maxLon) / 2, (minLat + maxLat) / 2, 0);
    const diagonal = Cesium.Cartesian3.distance(
      Cesium.Cartesian3.fromDegrees(minLon, minLat, 0),
      Cesium.Cartesian3.fromDegrees(maxLon, maxLat, 0),
    );
    viewer3d.camera.lookAt(
      center,
      new Cesium.HeadingPitchRange(
        Cesium.Math.toRadians(315),
        Cesium.Math.toRadians(-48),
        Math.max(28000, diagonal * 1.05),
      ),
    );
  }
  initialize3d();

  function activeAircraft(frame) {
    return entrants.filter((uam) => uam.entry <= frame.t + 1e-9 && uam.exit >= frame.t - 1e-9).map((uam) => {
      const sM = Math.max(0, Math.min(corridorLengthM, speedMps * (frame.t - uam.entry)));
      const position = interpolateRoute(sM);
      const heading = Math.atan2(position.tangentX, position.tangentY) * 180 / Math.PI;
      return { ...uam, sM, position, heading, policy: frame.policies[uam.id] || "U", exposure: frame.exposure[uam.id] };
    });
  }

  function setSelected(uamId) {
    selectedUamId = uamId;
    updateFrame(index);
  }

  function update2d(aircraftRows) {
    const focal = aircraftRows.find((row) => row.id === selectedUamId);
    const groupIds = focal && focal.policy !== "U"
      ? new Set(entrants.slice(focal.index - 2, focal.index + 3).map((row) => row.id))
      : new Set();
    const activeIds = new Set(aircraftRows.map((row) => row.id));
    for (const [uamId, marker] of markers) {
      if (!activeIds.has(uamId)) { map.removeLayer(marker); markers.delete(uamId); }
    }
    aircraftRows.forEach((row) => {
      const selected = row.id === selectedUamId;
      let marker = markers.get(row.id);
      if (!marker) {
        marker = L.marker([row.position.lat, row.position.lon], { zIndexOffset: selected ? 1300 : 900 })
          .on("click", () => setSelected(row.id))
          .addTo(map);
        markers.set(row.id, marker);
      }
      marker.setLatLng([row.position.lat, row.position.lon]);
      marker.setIcon(markerIcon(row.policy, row.heading, selected, groupIds.has(row.id)));
      marker.bindTooltip(`${row.id} · ${row.policy === "U" ? "unclassified" : row.policy}`, { direction: "top" });
    });
  }

  function update3d(aircraftRows) {
    if (!viewer3d) return;
    const activeIds = new Set(aircraftRows.map((row) => row.id));
    for (const [uamId, entity] of aircraft3d) {
      if (!activeIds.has(uamId)) { viewer3d.entities.remove(entity); aircraft3d.delete(uamId); }
    }
    aircraftRows.forEach((row) => {
      let entity = aircraft3d.get(row.id);
      if (!entity) {
        entity = viewer3d.entities.add({
          position: Cesium.Cartesian3.fromDegrees(row.position.lon, row.position.lat, altitudeM),
          billboard: { image: planeSvg(row.policy), width: 32, height: 32, verticalOrigin: Cesium.VerticalOrigin.CENTER, disableDepthTestDistance: Number.POSITIVE_INFINITY },
        });
        aircraft3d.set(row.id, entity);
      }
      entity.position = Cesium.Cartesian3.fromDegrees(row.position.lon, row.position.lat, altitudeM);
      entity.billboard.image = planeSvg(row.policy);
      entity.billboard.width = row.id === selectedUamId ? 44 : 32;
      entity.billboard.height = row.id === selectedUamId ? 44 : 32;
      entity.billboard.rotation = Cesium.Math.toRadians(-row.heading);
    });
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
    const demand = Number(summary.traffic.entry_demand_uam_h);
    const q95 = Number(summary.capacity.q_mix_rho_uam_h);
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

  function updateFrame(nextIndex) {
    index = Math.max(0, Math.min(frames.length - 1, Math.round(nextIndex)));
    const frame = frames[index];
    const aircraftRows = activeAircraft(frame);
    if (!selectedUamId || !aircraftRows.some((row) => row.id === selectedUamId)) {
      selectedUamId = aircraftRows.find((row) => row.policy !== "U")?.id || aircraftRows[0]?.id || null;
    }
    update2d(aircraftRows);
    update3d(aircraftRows);
    el["time-slider"].value = String(index);
    el["current-time"].textContent = formatTime(frame.t);
    el["active-count"].textContent = String(frame.active_count);
    el["current-capacity"].textContent = frame.q_mix.toFixed(1);
    el["current-counts"].textContent = `Current C/R/F · ${frame.n_C}/${frame.n_R}/${frame.n_F}`;
    const selected = aircraftRows.find((row) => row.id === selectedUamId);
    if (selected) {
      const link = evaluateRadio(selected.position);
      el["selected-uam"].textContent = selected.id;
      el["selected-policy"].textContent = selected.policy === "U" ? "Unclassified" : selected.policy;
      el["selected-policy"].style.color = colors[selected.policy];
      el["selected-exposure"].textContent = selected.exposure == null ? "Not yet valid" : `${(100 * selected.exposure).toFixed(1)}% below Θ`;
      el["selected-site"].textContent = link.site.id;
      el["selected-rsrp"].textContent = `${link.rsrp.toFixed(1)} dBm/RE`;
      el["selected-sinr"].textContent = `${link.sinr.toFixed(1)} dB`;
      el["selected-progress"].textContent = `${(selected.sM / 1000).toFixed(2)} km · ${(100 * selected.sM / corridorLengthM).toFixed(1)}%`;
      el["selected-group"].textContent = selected.policy === "U"
        ? "Not yet valid"
        : entrants.slice(selected.index - 2, selected.index + 3).map((row) => row.id).join(" · ");
      servingLine.setLatLngs([[selected.position.lat, selected.position.lon], [link.site.lat, link.site.lon]]);
    } else {
      servingLine.setLatLngs([]);
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
  window.addEventListener("resize", () => drawCapacityChart(index));
  updateFrame(0);
})();
