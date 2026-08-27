(function attachTrafficEngine(root, factory) {
  const api = factory();
  if (typeof module === "object" && module.exports) module.exports = api;
  root.UAM_TRAFFIC_ENGINE = api;
})(typeof globalThis !== "undefined" ? globalThis : this, function buildTrafficEngine() {
  "use strict";

  const EPSILON = 1e-9;

  function interpolateRoute(route, corridorLengthM, sM, lateralM = 0) {
    const clipped = Math.max(0, Math.min(corridorLengthM, sM));
    let low = 0;
    let high = route.length - 1;
    while (high - low > 1) {
      const middle = Math.floor((low + high) / 2);
      if (route[middle].s_m <= clipped) low = middle; else high = middle;
    }
    const a = route[low];
    const b = route[high];
    const segmentM = Math.max(EPSILON, b.s_m - a.s_m);
    const fraction = (clipped - a.s_m) / segmentM;
    const tangentX = b.x_m - a.x_m;
    const tangentY = b.y_m - a.y_m;
    const tangentLength = Math.hypot(tangentX, tangentY);
    const normalX = -tangentY / tangentLength;
    const normalY = tangentX / tangentLength;
    const centerLon = a.lon + fraction * (b.lon - a.lon);
    const centerLat = a.lat + fraction * (b.lat - a.lat);
    return {
      s_m: clipped,
      x_m: a.x_m + fraction * tangentX + lateralM * normalX,
      y_m: a.y_m + fraction * tangentY + lateralM * normalY,
      lon: centerLon + lateralM * normalX / (111320 * Math.cos(centerLat * Math.PI / 180)),
      lat: centerLat + lateralM * normalY / 111320,
      tangentX,
      tangentY,
    };
  }

  function evaluateRadio(stations, radio, position, altitudeM) {
    const servedSetSize = radio.served_set_size == null ? null : Number(radio.served_set_size);
    const links = stations.map((site, index) => {
      const dx = position.x_m - site.x_m;
      const dy = position.y_m - site.y_m;
      const dz = altitudeM - site.height_m;
      const d2 = Math.max(dx * dx + dy * dy + dz * dz, Number.MIN_VALUE);
      const rx = Number(radio.eirp_dbm) + Number(radio.receiver_gain_db) - 28
        - 11 * Math.log10(d2) - 20 * Math.log10(Number(radio.frequency_ghz));
      return { index, d2, rx };
    });
    const served = servedSetSize !== null && servedSetSize < links.length
      ? [...links].sort((left, right) => left.d2 - right.d2).slice(0, servedSetSize)
      : links;
    let serving = served[0];
    served.forEach((link) => { if (link.rx > serving.rx) serving = link; });
    const desiredMw = 10 ** (serving.rx / 10);
    const interferenceMw = Math.max(
      served.reduce((total, link) => total + 10 ** (link.rx / 10), 0) - desiredMw,
      Number.MIN_VALUE,
    );
    const noiseMw = 10 ** (Number(radio.noise_dbm) / 10);
    return {
      site: stations[serving.index],
      rsrp: serving.rx - 10 * Math.log10(Number(radio.resource_elements)),
      sinr: 10 * Math.log10(desiredMw / (interferenceMw + noiseMw)),
    };
  }

  function reliabilityFloor(values, rho) {
    const ordered = [...values].sort((left, right) => left - right);
    const index = Math.floor((1 - rho) * ordered.length + 1e-12);
    return ordered[Math.max(0, Math.min(ordered.length - 1, index))];
  }

  function validateParameters(parameters) {
    const values = Object.fromEntries(Object.entries(parameters).map(([key, value]) => [key, Number(value)]));
    const positive = ["speedMps", "altitudeM", "departureIntervalS", "exposureWindowS", "policyIntervalS"];
    positive.forEach((key) => { if (!(values[key] > 0)) throw new Error(`${key} must be positive`); });
    if (!Number.isInteger(values.groupSize) || values.groupSize < 1 || values.groupSize % 2 !== 1) {
      throw new Error("Local group size must be a positive odd integer");
    }
    if (!(0 <= values.coordinatedTolerance && values.coordinatedTolerance <= values.reactiveTolerance && values.reactiveTolerance <= 1)) {
      throw new Error("Exposure limits must satisfy 0 ≤ C ≤ R ≤ 1");
    }
    if (!(values.reliabilityRho > 0 && values.reliabilityRho <= 1)) {
      throw new Error("Reliability rho must lie in (0, 1]");
    }
    return values;
  }

  function makeTimeGrid(durationS, dtS) {
    const count = Math.floor(durationS / dtS);
    const grid = Array.from({ length: count + 1 }, (_unused, index) => index * dtS);
    if (grid.at(-1) < durationS - EPSILON) grid.push(durationS); else grid[grid.length - 1] = durationS;
    return grid;
  }

  function simulate(data, rawParameters) {
    const parameters = validateParameters(rawParameters);
    const route = data.route_metric;
    const corridorLengthM = Number(data.summary.corridor_length_km) * 1000;
    const durationS = Number(data.summary.simulation_duration_s);
    const dtS = 1;
    const transitTimeS = corridorLengthM / parameters.speedMps;
    const entrantCount = Math.ceil(durationS / parameters.departureIntervalS - EPSILON);
    const entrants = Array.from({ length: entrantCount }, (_unused, index) => {
      const entry = index * parameters.departureIntervalS;
      return { id: `UAM${String(index + 1).padStart(3, "0")}`, index, entry, exit: entry + transitTimeS };
    });
    const capacity = data.summary.model_config.capacity;
    const spacingByPolicy = Object.fromEntries(["C", "R", "F"].map((policy) => [
      policy,
      Number(capacity.standstill_distance_m)
        + Number(capacity.response_time_s[policy]) * parameters.speedMps
        + Number(capacity.braking_buffer_s2_per_m) * parameters.speedMps ** 2,
    ]));
    const histories = new Map();
    const heldPolicies = new Map();
    const frames = [];
    const policyCounts = { C: 0, R: 0, F: 0 };
    let observationCount = 0;

    makeTimeGrid(durationS, dtS).forEach((timestamp) => {
      const activeEntrants = entrants.filter((uam) => uam.entry <= timestamp + EPSILON && uam.exit >= timestamp - EPSILON);
      const activeIds = activeEntrants.map((uam) => uam.id);
      const linkOk = activeEntrants.map((uam) => {
        const sM = Math.max(0, Math.min(corridorLengthM, parameters.speedMps * (timestamp - uam.entry)));
        const position = interpolateRoute(route, corridorLengthM, sM, parameters.lateralOffsetM);
        return evaluateRadio(data.stations, data.summary.radio, position, parameters.altitudeM).sinr >= parameters.sinrThresholdDb;
      });
      const half = Math.floor(parameters.groupSize / 2);
      const policies = {};
      const exposure = {};
      const groups = {};
      activeIds.forEach((uamId, index) => {
        const start = Math.max(0, index - half);
        const end = Math.min(activeIds.length, index + half + 1);
        const members = activeIds.slice(start, end);
        groups[uamId] = members;
        const support = linkOk.slice(start, end).filter(Boolean).length / members.length;
        const history = histories.get(uamId) || [];
        history.push({ t: timestamp, support });
        while (history.length && history[0].t < timestamp - parameters.exposureWindowS - EPSILON) history.shift();
        histories.set(uamId, history);
        exposure[uamId] = 1 - history.reduce((total, row) => total + row.support, 0) / history.length;
        const isPolicyTick = parameters.policyIntervalS <= dtS + EPSILON
          || Math.abs(timestamp / parameters.policyIntervalS - Math.round(timestamp / parameters.policyIntervalS)) < EPSILON;
        if (isPolicyTick || !heldPolicies.has(uamId)) {
          heldPolicies.set(
            uamId,
            exposure[uamId] <= parameters.coordinatedTolerance + 1e-12
              ? "C"
              : exposure[uamId] <= parameters.reactiveTolerance + 1e-12 ? "R" : "F",
          );
        }
        policies[uamId] = heldPolicies.get(uamId);
        policyCounts[policies[uamId]] += 1;
        observationCount += 1;
      });
      const policyValues = Object.values(policies);
      const nC = policyValues.filter((value) => value === "C").length;
      const nR = policyValues.filter((value) => value === "R").length;
      const nF = policyValues.filter((value) => value === "F").length;
      const meanSpacing = policyValues.reduce((total, policy) => total + spacingByPolicy[policy], 0) / policyValues.length;
      const localFlows = policyValues.map((policy) => 3600 * parameters.speedMps / spacingByPolicy[policy]);
      frames.push({
        t: timestamp,
        active_count: activeIds.length,
        classified_count: activeIds.length,
        n_C: nC,
        n_R: nR,
        n_F: nF,
        mean_spacing_m: meanSpacing,
        q_mix: 3600 * parameters.speedMps / meanSpacing,
        q_bottleneck: Math.min(...localFlows),
        policies,
        exposure,
        groups,
      });
    });
    const qMixRho = reliabilityFloor(frames.map((frame) => frame.q_mix), parameters.reliabilityRho);
    const offeredDemand = 3600 / parameters.departureIntervalS;
    return {
      parameters,
      entrants,
      frames,
      results: {
        transitTimeS,
        offeredDemand,
        expectedOccupancy: transitTimeS / parameters.departureIntervalS,
        reliabilityCapacity: qMixRho,
        demandSupported: offeredDemand <= qMixRho + 1e-12,
        observationCount,
        policyCounts,
        policyShares: Object.fromEntries(Object.entries(policyCounts).map(([key, value]) => [key, value / observationCount])),
      },
    };
  }

  return { evaluateRadio, interpolateRoute, reliabilityFloor, simulate, validateParameters };
});
