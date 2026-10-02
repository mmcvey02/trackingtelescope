"use strict";

/*
 * Walk-to-map: trace rooms by walking along their walls with a phone.
 *
 * Map frame = the robot's: the dock is the origin and heading 0 (+x) points
 * straight out of the dock into the room; angles grow counter-clockwise, so
 * turning left is positive. You start at the dock facing into the room.
 *
 *   StepCounter     counts steps from the accelerometer
 *   HeadingTracker  integrates the gyroscope around the vertical axis, so it
 *                   works however the phone is held
 *   WalkTracker     turns steps + heading into straight legs, finding a
 *                   corner whenever you keep walking in a new direction
 *
 * Pure logic (no DOM) so it can be unit-tested with node. The finished path
 * is sent to the server, which closes the loop, squares near-right angles
 * and offsets the walls (geometry.walk_to_polygon).
 */
(function (root) {
  const RAD = Math.PI / 180;

  /** Smallest signed difference a - b, in radians (-pi..pi]. */
  function angleDiff(a, b) {
    let d = (a - b) % (2 * Math.PI);
    if (d > Math.PI) d -= 2 * Math.PI;
    if (d <= -Math.PI) d += 2 * Math.PI;
    return d;
  }

  function meanAngle(list) {
    let s = 0, c = 0;
    for (const a of list) { s += Math.sin(a); c += Math.cos(a); }
    return Math.atan2(s, c);
  }

  /** Step detector for accelerometer samples (m/s^2, gravity included). */
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

  /**
   * Heading from the gyroscope. The phone's rotation rate (deg/s about its
   * own x, y, z axes) is projected onto the vertical, found from gravity, so
   * only turning your body counts - not tilting the phone. Browsers disagree
   * on the sign of the gravity reading, so "up" is taken as whichever way
   * points out of the screen/top edge, as a phone held in front of you does.
   */
  class HeadingTracker {
    constructor() {
      this.heading = 0;      // radians, 0 = facing into the room from the dock
      this.gravity = null;
      this.lastT = null;
      this.reverse = false;  // user setting, if a browser reports turns backwards
      this.samples = 0;
    }

    addGravity(x, y, z) {
      if (this.gravity === null) { this.gravity = [x, y, z]; return; }
      const g = this.gravity;
      for (const [i, v] of [x, y, z].entries()) g[i] += 0.1 * (v - g[i]);
    }

    up() {
      if (!this.gravity) return null;
      let [x, y, z] = this.gravity;
      const n = Math.hypot(x, y, z);
      if (n < 1) return null;
      [x, y, z] = [x / n, y / n, z / n];
      if (y + z < 0) [x, y, z] = [-x, -y, -z];
      return [x, y, z];
    }

    /** alpha/beta/gamma: rotation rate about the device z/x/y axes, deg/s. */
    addRotation(alpha, beta, gamma, tMs) {
      const u = this.up();
      const t0 = this.lastT;
      this.lastT = tMs;
      if (!u || t0 === null || alpha === null) return;
      const dt = Math.min(0.2, Math.max(0, (tMs - t0) / 1000));
      const yawRate = (beta || 0) * u[0] + (gamma || 0) * u[1] + (alpha || 0) * u[2];
      this.heading += (this.reverse ? -1 : 1) * yawRate * RAD * dt;
      this.samples += 1;
    }
  }

  /**
   * Straight legs from steps and heading. A corner is recorded once you have
   * taken `confirmSteps` steps in a direction more than `turnDeg` away from
   * the current wall - so glancing around, or turning back, doesn't count.
   */
  class WalkTracker {
    constructor(opts = {}) {
      this.stepLen = opts.stepLen || 0.7;
      this.turnRad = (opts.turnDeg || 30) * RAD;
      this.confirmSteps = opts.confirmSteps || 2;
      this.start = (opts.start || [0, 0]).slice();
      this.legs = [];        // finished walls: {h, dist}
      this.newLeg(opts.heading || 0);
    }

    newLeg(h, base = 0) {
      this.leg = { h, hs: [], steps: 0, override: null, base };
      this.pending = null;
    }

    legHeading() { return this.leg.hs.length ? meanAngle(this.leg.hs) : this.leg.h; }

    legDist() {
      const l = this.leg;
      return l.base + (l.override !== null ? l.override : l.steps * this.stepLen);
    }

    /** Type the real length of the current wall. Returns the step length learned, if any. */
    setLength(metres) {
      this.leg.override = metres === null ? null : Math.max(0, metres - this.leg.base);
    }

    /** A step was taken while facing `heading`. Returns {angle} when a corner is found. */
    step(heading) {
      if (this.pending) {
        if (Math.abs(angleDiff(heading, this.legHeading())) < this.turnRad) {
          // back on the old line: it was a glance or a wobble, not a corner.
          // Those steps still count, along the wall's direction.
          const h = this.legHeading();
          for (let i = 0; i < this.pending.hs.length; i++) this.leg.hs.push(h);
          this.leg.steps += this.pending.hs.length;
          this.pending = null;
          this.leg.hs.push(heading);
          this.leg.steps += 1;
          return null;
        }
        this.pending.hs.push(heading);
        if (this.pending.hs.length >= this.confirmSteps) return this.corner();
        return null;
      }
      if (this.leg.steps + this.leg.hs.length > 0 || this.leg.base > 0) {
        if (Math.abs(angleDiff(heading, this.legHeading())) >= this.turnRad) {
          this.pending = { hs: [heading] };
          return null;
        }
      } else if (Math.abs(angleDiff(heading, this.leg.h)) >= this.turnRad) {
        // turned before walking (e.g. at the dock): just face the new way
        this.leg.h = heading;
      }
      this.leg.hs.push(heading);
      this.leg.steps += 1;
      return null;
    }

    corner() {
      const old = this.legHeading();
      const learned = this.finishLeg();
      const hs = this.pending ? this.pending.hs : [];
      const h = hs.length ? meanAngle(hs) : old;
      this.newLeg(h);
      this.leg.hs = hs;
      this.leg.steps = hs.length;
      return { angle: angleDiff(h, old) / RAD, learned };
    }

    finishLeg() {
      let learned = null;
      const l = this.leg;
      if (l.override !== null && l.steps >= 5) learned = l.override / l.steps;
      const d = this.legDist();
      if (d > 0) this.legs.push({ h: this.legHeading(), dist: d });
      return learned;
    }

    /** For phones without a gyroscope: the user says how far they turned (left = +). */
    manualTurn(degrees) {
      const h = this.legHeading() + degrees * RAD;
      const learned = this.finishLeg();
      this.newLeg(h);
      return { angle: degrees, learned };
    }

    /** "That wasn't a corner": merge the last wall back into the current one. */
    undoCorner() {
      const prev = this.legs.pop();
      if (!prev) return false;
      const extra = this.legDist();
      this.newLeg(prev.h, prev.dist + extra);
      return true;
    }

    /** Corner points from the start to where you are now. */
    points() {
      const pts = [this.start.slice()];
      let [x, y] = this.start;
      const legs = this.legs.concat([{ h: this.legHeading(), dist: this.legDist() }]);
      for (const leg of legs) {
        if (!(leg.dist > 0)) continue;
        x += Math.cos(leg.h) * leg.dist;
        y += Math.sin(leg.h) * leg.dist;
        pts.push([x, y]);
      }
      if (this.pending) {
        const h = meanAngle(this.pending.hs);
        x += Math.cos(h) * this.pending.hs.length * this.stepLen;
        y += Math.sin(h) * this.pending.hs.length * this.stepLen;
        pts.push([x, y]);
      }
      return pts;
    }

    position() {
      const pts = this.points();
      return pts[pts.length - 1];
    }
  }

  const api = { angleDiff, meanAngle, StepCounter, HeadingTracker, WalkTracker };
  if (typeof module !== "undefined" && module.exports) module.exports = api;
  else root.Walk = api;
})(typeof window !== "undefined" ? window : globalThis);
