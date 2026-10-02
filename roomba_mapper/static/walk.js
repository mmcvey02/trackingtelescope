"use strict";

/*
 * Walk-to-map: build room outlines by walking along the walls with a phone.
 *
 * The map frame is the robot's: the dock is the origin and +x points straight
 * out of the dock into the room. You start standing at the dock facing into
 * the room, then walk along walls in straight lines; at each corner you tell
 * the app which way you turned. Rooms are almost always square-cornered, so
 * every leg runs along one of four directions; the length of each leg comes
 * from counting steps (and can be typed in instead).
 *
 * Pure functions here (no DOM) so they can be unit-tested with node.
 */
(function (root) {
  // heading index: 0 = +x (out of the dock), 1 = +y (left), 2 = -x, 3 = -y
  const DIRS = [[1, 0], [0, 1], [-1, 0], [0, -1]];

  const turn = (q, which) => (q + ({ left: 1, right: 3, around: 2 }[which] || 0)) % 4;

  function polygonArea(pts) {
    let a = 0;
    for (let k = 0; k < pts.length; k++) {
      const [x1, y1] = pts[k], [x2, y2] = pts[(k + 1) % pts.length];
      a += x1 * y2 - x2 * y1;
    }
    return a / 2;
  }

  /** Merge consecutive legs in the same direction and drop zero-length ones. */
  function mergeLegs(legs) {
    const out = [];
    for (const leg of legs) {
      if (!(leg.dist > 0)) continue;
      const last = out[out.length - 1];
      if (last && last.q === leg.q) last.dist += leg.dist;
      else out.push({ q: leg.q, dist: leg.dist });
    }
    return out;
  }

  /** Corner points visited when walking `legs` from `start`. */
  function walkPoints(start, legs) {
    const pts = [start.slice()];
    for (const leg of legs) {
      const [dx, dy] = DIRS[leg.q];
      const [x, y] = pts[pts.length - 1];
      pts.push([x + dx * leg.dist, y + dy * leg.dist]);
    }
    return pts;
  }

  /**
   * Close a walked loop. Step counting is never exact, so the walk rarely
   * ends where it began; the gap is shared out over the legs in proportion
   * to their length (x-gap over the x legs, y-gap over the y legs), keeping
   * every wall square. Returns the corrected legs.
   */
  function closeLoop(legs) {
    const merged = mergeLegs(legs);
    const end = walkPoints([0, 0], merged).pop();
    const fixed = merged.map((l) => ({ q: l.q, dist: l.dist }));
    for (const axis of [0, 1]) {
      const along = fixed.filter((l) => DIRS[l.q][axis] !== 0);
      const total = along.reduce((s, l) => s + l.dist, 0);
      const gap = end[axis];
      if (!total || Math.abs(gap) < 1e-9) continue;
      for (const l of along) {
        // a leg in the + direction shrinks by its share of a positive gap, a - leg grows
        l.dist = Math.max(0, l.dist - DIRS[l.q][axis] * gap * (l.dist / total));
      }
    }
    return mergeLegs(fixed);
  }

  function intersect(p, d, q, e) {
    const den = d[0] * e[1] - d[1] * e[0];
    if (Math.abs(den) < 1e-12) return null;
    const t = ((q[0] - p[0]) * e[1] - (q[1] - p[1]) * e[0]) / den;
    return [p[0] + t * d[0], p[1] + t * d[1]];
  }

  /** Move every edge of a polygon outward by `d` metres (negative = inward). */
  function offsetPolygon(pts, d) {
    const n = pts.length;
    if (n < 3 || !d) return pts.map((p) => p.slice());
    const sign = polygonArea(pts) >= 0 ? 1 : -1;
    const lines = [];
    for (let k = 0; k < n; k++) {
      const a = pts[k], b = pts[(k + 1) % n];
      const dx = b[0] - a[0], dy = b[1] - a[1];
      const len = Math.hypot(dx, dy) || 1;
      const nx = sign * dy / len, ny = -sign * dx / len; // outward normal
      lines.push([[a[0] + nx * d, a[1] + ny * d], [dx, dy]]);
    }
    return pts.map((p, k) => {
      const prev = lines[(k - 1 + n) % n], cur = lines[k];
      return intersect(prev[0], prev[1], cur[0], cur[1]) || cur[0];
    });
  }

  /** Drop corners where the outline just carries straight on (e.g. the dock spot mid-wall). */
  function removeStraight(pts) {
    let out = pts.slice();
    let changed = true;
    while (changed && out.length > 3) {
      changed = false;
      for (let k = 0; k < out.length; k++) {
        const a = out[(k - 1 + out.length) % out.length], b = out[k], c = out[(k + 1) % out.length];
        const cross = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0]);
        const tooClose = Math.hypot(b[0] - a[0], b[1] - a[1]) < 1e-6;
        if (Math.abs(cross) < 1e-9 || tooClose) { out.splice(k, 1); changed = true; break; }
      }
    }
    return out;
  }

  /**
   * Finished shape from a walk: close the loop, then shift the walls out by
   * how far from them you walked (rooms), or in (walking around furniture).
   */
  function shapeFromWalk(start, legs, wallGap, kind) {
    const closed = closeLoop(legs);
    let pts = walkPoints(start, closed);
    pts.pop(); // last point == first point after closing
    pts = removeStraight(pts);
    if (pts.length < 3) return null;
    const gap = kind === "obstacle" ? -wallGap : wallGap;
    pts = offsetPolygon(pts, gap);
    if (polygonArea(pts) < 0) pts.reverse();
    return pts.map(([x, y]) => [Math.round(x * 100) / 100, Math.round(y * 100) / 100]);
  }

  /**
   * Step detector for accelerometer samples (m/s^2, gravity included).
   * Each step shows up as a bump in total acceleration; a slow average
   * tracks the baseline and a hysteresis threshold counts each bump once.
   */
  class StepCounter {
    constructor(opts = {}) {
      this.high = opts.high || 1.2;   // m/s^2 above baseline to count a step
      this.low = opts.low || 0.3;     // must fall back below this before the next one
      this.minGapMs = opts.minGapMs || 280;
      this.reset();
    }

    reset() {
      this.steps = 0;
      this.baseline = null;
      this.smooth = null;
      this.armed = true;
      this.lastStep = -Infinity;
    }

    /** Feed one sample; returns true when a step is counted. */
    add(ax, ay, az, tMs) {
      const mag = Math.hypot(ax, ay, az);
      if (this.baseline === null) { this.baseline = mag; this.smooth = mag; return false; }
      this.baseline += 0.02 * (mag - this.baseline);
      this.smooth += 0.35 * (mag - this.smooth);
      const rel = this.smooth - this.baseline;
      if (this.armed && rel > this.high && tMs - this.lastStep >= this.minGapMs) {
        this.steps += 1;
        this.lastStep = tMs;
        this.armed = false;
        return true;
      }
      if (!this.armed && rel < this.low) this.armed = true;
      return false;
    }
  }

  const api = { DIRS, turn, mergeLegs, walkPoints, closeLoop, offsetPolygon, shapeFromWalk, removeStraight,
                polygonArea, StepCounter };
  if (typeof module !== "undefined" && module.exports) module.exports = api;
  else root.Walk = api;
})(typeof window !== "undefined" ? window : globalThis);
