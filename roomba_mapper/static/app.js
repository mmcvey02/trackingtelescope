"use strict";

(() => {
  const POLL_MS = 1000;
  const SNAP = 0.05;            // m, drawing snaps to 5 cm
  const HANDLE_PX = 18;
  const KIND_LABEL = { floor: "Room / floor", obstacle: "Obstacle", nogo: "No-go zone" };
  const ACTIVITY_LABEL = {
    cleaning: "Cleaning", returning: "Going home", docked: "Docked", paused: "Paused",
    stuck: "Stuck!", idle: "Idle", emptying: "Emptying bin", unknown: "…",
  };

  const $ = (id) => document.getElementById(id);
  const canvas = $("map");
  const ctx = canvas.getContext("2d");

  let pin = "";
  try { pin = localStorage.getItem("roomba-pin") || ""; } catch (_) { /* storage blocked */ }

  let S = null;              // latest state
  let elements = [];
  let mapVersion = null;
  let covTag = null;
  let raster = null;         // {res,i0,j0,w,h,data}
  let rasterCanvas = null;
  let colors = null;
  let settingsFilled = false;

  const view = { scale: 40, panX: 0, panY: 0, userMoved: false, ready: false };
  let editing = false;
  let tool = "select";
  let kind = "floor";
  let selectedId = null;
  let activeVertex = null;
  let draft = [];
  let rectDraft = null;
  let gesture = null;
  const pointers = new Map();

  // ---- API -----------------------------------------------------------------

  async function api(path, body) {
    const opts = { headers: { "X-Pin": pin } };
    if (body !== undefined) {
      opts.method = "POST";
      opts.headers["Content-Type"] = "application/json";
      opts.body = JSON.stringify(body);
    }
    const res = await fetch(path, opts);
    if (res.status === 401) { askPin(); throw new Error("PIN required"); }
    const data = await res.json().catch(() => ({}));
    if (!res.ok) throw new Error(data.error || res.statusText);
    return data;
  }

  async function post(path, body) {
    try {
      const data = await api(path, body);
      apply(data);
      return data;
    } catch (err) {
      setNotice("⚠ " + err.message);
      return null;
    }
  }

  async function poll() {
    try {
      const q = `?map=${encodeURIComponent(mapVersion ?? "")}&cov=${encodeURIComponent(covTag ?? "")}`;
      apply(await api("/api/state" + q));
    } catch (err) {
      if (err.message !== "PIN required") setNotice("⚠ Can't reach the mapper – retrying…");
    } finally {
      setTimeout(poll, document.hidden ? POLL_MS * 5 : POLL_MS);
    }
  }

  function askPin() {
    const dlg = $("pin-dialog");
    if (!dlg.open) dlg.showModal();
  }
  $("pin-form").addEventListener("submit", () => {
    pin = $("pin-input").value.trim();
    try { localStorage.setItem("roomba-pin", pin); } catch (_) { /* ignore */ }
    mapVersion = covTag = null;
  });

  // ---- state ---------------------------------------------------------------

  function apply(data) {
    const draggingShape = gesture && gesture.type === "vertex";
    if (data.elements && !draggingShape) {
      elements = data.elements;
      mapVersion = String(data.map_version);
      if (selectedId !== null && !elements.some((e) => e.id === selectedId)) select(null);
      if (data.created_id) select(data.created_id);
    }
    if (data.coverage) {
      raster = data.coverage;
      covTag = data.cov_version;
      rasterCanvas = null;
    }
    const prevRot = S && S.settings.view_rotation;
    const prevMirror = S && S.settings.view_mirror;
    S = data;
    if (!settingsFilled) { fillSettings(); settingsFilled = true; }
    if (!view.userMoved || prevRot !== S.settings.view_rotation || prevMirror !== S.settings.view_mirror) fit();
    updatePanel();
    draw();
  }

  function setNotice(text) { $("notice").textContent = text; }

  // ---- view math -----------------------------------------------------------

  function rot() {
    const th = ((S && S.settings.view_rotation) || 0) * Math.PI / 180;
    return { c: Math.cos(th), s: Math.sin(th), m: S && S.settings.view_mirror ? -1 : 1 };
  }
  function toScreen(x, y) {
    const { c, s, m } = rot();
    return [view.panX + m * (c * x - s * y) * view.scale, view.panY - (s * x + c * y) * view.scale];
  }
  function toWorld(px, py) {
    const { c, s, m } = rot();
    const rx = (px - view.panX) / view.scale * m;
    const ry = -(py - view.panY) / view.scale;
    return [c * rx + s * ry, -s * rx + c * ry];
  }
  const snap = (v) => Math.round(v / SNAP) * SNAP;

  function size() {
    const r = canvas.getBoundingClientRect();
    return [r.width, r.height];
  }

  function resizeCanvas() {
    const [w, h] = size();
    const dpr = window.devicePixelRatio || 1;
    canvas.width = Math.max(1, Math.round(w * dpr));
    canvas.height = Math.max(1, Math.round(h * dpr));
  }

  function contentPoints() {
    const pts = [[0, 0]];
    for (const el of elements) pts.push(...el.points);
    if (raster && raster.w) {
      const r = raster.res;
      pts.push([raster.i0 * r, raster.j0 * r], [(raster.i0 + raster.w) * r, (raster.j0 + raster.h) * r]);
    }
    if (S && S.trail) for (let k = 0; k < S.trail.length; k += 10) pts.push(S.trail[k]);
    if (S && S.robot.pose) pts.push(S.robot.pose);
    if (walk.active) pts.push(...walkPathPoints());
    return pts;
  }

  function fit() {
    const [W, H] = size();
    if (!W || !H) return;
    const { c, s, m } = rot();
    let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
    for (const [x, y] of contentPoints()) {
      const vx = m * (c * x - s * y), vy = s * x + c * y;
      x0 = Math.min(x0, vx); x1 = Math.max(x1, vx); y0 = Math.min(y0, vy); y1 = Math.max(y1, vy);
    }
    const w = Math.max(x1 - x0, 2), h = Math.max(y1 - y0, 2);
    view.scale = Math.max(4, Math.min(300, Math.min((W - 40) / w, (H - 40) / h)));
    view.panX = W / 2 - ((x0 + x1) / 2) * view.scale;
    view.panY = H / 2 + ((y0 + y1) / 2) * view.scale;
    view.ready = true;
  }

  // ---- drawing -------------------------------------------------------------

  function readColors() {
    const st = getComputedStyle(document.documentElement);
    const v = (n) => st.getPropertyValue(n).trim();
    const probe = document.createElement("canvas").getContext("2d");
    const rgba = (col) => {
      probe.clearRect(0, 0, 1, 1);
      probe.fillStyle = col;
      probe.fillRect(0, 0, 1, 1);
      return Array.from(probe.getImageData(0, 0, 1, 1).data);
    };
    colors = {
      mapBg: v("--map-bg"), floor: v("--floor"), floorLine: v("--floor-line"),
      obstacle: v("--obstacle"), nogo: v("--nogo"), trail: v("--trail"), robot: v("--robot"),
      select: v("--select"), text: v("--text"), accent: v("--accent"),
      cells: { 1: rgba(v("--needs")), 2: rgba(v("--clean")), 3: rgba(v("--outside")) },
    };
    rasterCanvas = null;
  }

  function buildRaster() {
    if (!raster || !raster.w) return null;
    const off = document.createElement("canvas");
    off.width = raster.w;
    off.height = raster.h;
    const octx = off.getContext("2d");
    const img = octx.createImageData(raster.w, raster.h);
    const d = raster.data;
    for (let k = 0; k < d.length; k++) {
      const col = colors.cells[d.charCodeAt(k) - 48];
      if (!col) continue;
      img.data.set(col, k * 4);
    }
    octx.putImageData(img, 0, 0);
    return off;
  }

  function pathPoly(pts) {
    ctx.beginPath();
    pts.forEach(([x, y], k) => {
      const [sx, sy] = toScreen(x, y);
      if (k) ctx.lineTo(sx, sy); else ctx.moveTo(sx, sy);
    });
    ctx.closePath();
  }

  let hatch = null;
  function nogoPattern() {
    if (hatch) return hatch;
    const p = document.createElement("canvas");
    p.width = p.height = 10;
    const g = p.getContext("2d");
    g.strokeStyle = colors.nogo;
    g.lineWidth = 2;
    g.beginPath(); g.moveTo(0, 10); g.lineTo(10, 0); g.stroke();
    hatch = ctx.createPattern(p, "repeat");
    return hatch;
  }

  function draw() {
    if (!colors) readColors();
    const dpr = window.devicePixelRatio || 1;
    const [W, H] = size();
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.fillStyle = colors.mapBg;
    ctx.fillRect(0, 0, W, H);
    if (!S) return;

    const shown = elements.map((el) => (gesture && gesture.type === "vertex" && el.id === gesture.id)
      ? { ...el, points: gesture.pts } : el);

    // floor
    for (const el of shown) {
      if (el.kind !== "floor") continue;
      pathPoly(el.points);
      ctx.fillStyle = colors.floor;
      ctx.fill();
    }

    // cleaning coverage raster
    if (raster && raster.w) {
      rasterCanvas = rasterCanvas || buildRaster();
      const { c, s, m } = rot();
      const k = view.scale;
      ctx.save();
      ctx.setTransform(dpr * m * k * c, dpr * -k * s, dpr * -m * k * s, dpr * -k * c, dpr * view.panX, dpr * view.panY);
      ctx.transform(raster.res, 0, 0, -raster.res, raster.i0 * raster.res, (raster.j0 + raster.h) * raster.res);
      ctx.imageSmoothingEnabled = false;
      ctx.drawImage(rasterCanvas, 0, 0);
      ctx.restore();
    }

    // walls, obstacles, no-go
    for (const el of shown) {
      pathPoly(el.points);
      if (el.kind === "floor") {
        ctx.strokeStyle = colors.floorLine;
        ctx.lineWidth = 2;
        ctx.setLineDash(el.source === "manual" ? [] : [6, 3]);
        ctx.stroke();
        ctx.setLineDash([]);
      } else if (el.kind === "obstacle") {
        ctx.fillStyle = colors.obstacle;
        ctx.globalAlpha = 0.85;
        ctx.fill();
        ctx.globalAlpha = 1;
      } else {
        ctx.fillStyle = nogoPattern();
        ctx.fill();
        ctx.strokeStyle = colors.nogo;
        ctx.lineWidth = 2;
        ctx.stroke();
      }
    }

    // robot path
    if (S.trail && S.trail.length > 1) {
      ctx.strokeStyle = colors.trail;
      ctx.globalAlpha = S.running ? 0.75 : 0.35;
      ctx.lineWidth = 1.5;
      ctx.lineJoin = "round";
      ctx.beginPath();
      S.trail.forEach(([x, y], k) => {
        const [sx, sy] = toScreen(x, y);
        if (k) ctx.lineTo(sx, sy); else ctx.moveTo(sx, sy);
      });
      ctx.stroke();
      ctx.globalAlpha = 1;
    }

    // missed spots
    ctx.font = "bold 11px system-ui, sans-serif";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    (S.missed || []).forEach((spot, k) => {
      const [sx, sy] = toScreen(spot.x, spot.y);
      const r = Math.max(9, Math.sqrt(spot.area / Math.PI) * view.scale);
      ctx.strokeStyle = "#d9822b";
      ctx.lineWidth = 2;
      ctx.setLineDash([3, 3]);
      ctx.beginPath(); ctx.arc(sx, sy, r, 0, Math.PI * 2); ctx.stroke();
      ctx.setLineDash([]);
      ctx.fillStyle = "#d9822b";
      ctx.fillText(String(k + 1), sx, sy);
    });

    // dock
    {
      const [sx, sy] = toScreen(0, 0);
      ctx.fillStyle = colors.accent;
      ctx.fillRect(sx - 9, sy - 9, 18, 18);
      ctx.fillStyle = "#fff";
      ctx.font = "13px system-ui, sans-serif";
      ctx.fillText("⌂", sx, sy + 1);
    }

    // robot
    if (S.robot.pose) {
      const [x, y, th] = S.robot.pose;
      const [sx, sy] = toScreen(x, y);
      const [hx, hy] = toScreen(x + Math.cos(th) * 0.17, y + Math.sin(th) * 0.17);
      const r = Math.max(7, 0.17 * view.scale);
      ctx.fillStyle = colors.robot;
      ctx.globalAlpha = S.running ? 1 : 0.5;
      ctx.beginPath(); ctx.arc(sx, sy, r, 0, Math.PI * 2); ctx.fill();
      ctx.strokeStyle = "#fff";
      ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(sx, sy); ctx.lineTo(sx + (hx - sx) * Math.max(1, 7 / (0.17 * view.scale)), sy + (hy - sy) * Math.max(1, 7 / (0.17 * view.scale))); ctx.stroke();
      ctx.globalAlpha = 1;
    }

    // selection & handles
    const sel = shown.find((e) => e.id === selectedId);
    if (editing && sel) {
      pathPoly(sel.points);
      ctx.strokeStyle = colors.select;
      ctx.lineWidth = 3;
      ctx.stroke();
      const n = sel.points.length;
      sel.points.forEach(([x, y], k) => {
        const [ax, ay] = toScreen(x, y);
        const [bx, by] = toScreen(...sel.points[(k + 1) % n]);
        ctx.fillStyle = colors.select;
        ctx.globalAlpha = 0.6;
        ctx.beginPath(); ctx.arc((ax + bx) / 2, (ay + by) / 2, 5, 0, Math.PI * 2); ctx.fill();
        ctx.globalAlpha = 1;
        ctx.fillStyle = k === activeVertex ? colors.select : "#fff";
        ctx.strokeStyle = colors.select;
        ctx.lineWidth = 2;
        ctx.beginPath(); ctx.arc(ax, ay, 8, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
      });
    }

    // drafts
    if (draft.length) {
      ctx.strokeStyle = colors.select;
      ctx.lineWidth = 2;
      ctx.setLineDash([6, 4]);
      ctx.beginPath();
      draft.forEach(([x, y], k) => {
        const [sx, sy] = toScreen(x, y);
        if (k) ctx.lineTo(sx, sy); else ctx.moveTo(sx, sy);
      });
      if (draft.length > 2) ctx.closePath();
      ctx.stroke();
      ctx.setLineDash([]);
      draft.forEach(([x, y]) => {
        const [sx, sy] = toScreen(x, y);
        ctx.fillStyle = colors.select;
        ctx.beginPath(); ctx.arc(sx, sy, 5, 0, Math.PI * 2); ctx.fill();
      });
    }
    if (rectDraft) {
      const [a, b] = [rectDraft.a, rectDraft.b];
      ctx.strokeStyle = colors.select;
      ctx.lineWidth = 2;
      ctx.setLineDash([6, 4]);
      ctx.strokeRect(Math.min(a[0], b[0]), Math.min(a[1], b[1]), Math.abs(b[0] - a[0]), Math.abs(b[1] - a[1]));
      ctx.setLineDash([]);
      ctx.fillStyle = colors.text;
      ctx.fillText(rectSize(a, b), (a[0] + b[0]) / 2, (a[1] + b[1]) / 2);
    }

    if (walk.active) drawWalk();
    drawScaleBar(W, H);
  }

  function rectSize(a, b) {
    const w = Math.abs(b[0] - a[0]) / view.scale, h = Math.abs(b[1] - a[1]) / view.scale;
    return `${w.toFixed(2)} × ${h.toFixed(2)} m`;
  }

  function drawScaleBar(W, H) {
    const steps = [0.1, 0.25, 0.5, 1, 2, 5, 10, 20];
    const metres = steps.find((m) => m * view.scale >= 50) || 20;
    const len = metres * view.scale;
    const x = 12, y = H - 14;
    ctx.strokeStyle = colors.text;
    ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(x, y - 4); ctx.lineTo(x, y); ctx.lineTo(x + len, y); ctx.lineTo(x + len, y - 4); ctx.stroke();
    ctx.fillStyle = colors.text;
    ctx.font = "11px system-ui, sans-serif";
    ctx.textAlign = "left";
    ctx.fillText(metres >= 1 ? `${metres} m` : `${metres * 100} cm`, x + len + 6, y - 2);
    ctx.textAlign = "center";
  }

  // ---- panels --------------------------------------------------------------

  function updatePanel() {
    const st = S.stats, r = S.robot;
    $("stat-percent").textContent = st.percent == null ? "–" : st.percent + "%";
    $("stat-remaining").textContent = st.percent == null ? "–" : st.remaining_m2;
    $("stat-floor").textContent = st.floor_m2 || (st.explored_m2 ? "~" + st.explored_m2 : "–");
    $("stat-runs").textContent = S.runs;
    $("progress-bar").style.width = (st.percent || 0) + "%";
    setNotice(S.notice);
    $("link").textContent = r.link + (r.pose_source ? ` · positions via ${r.pose_source}` : "");
    $("robot-name").textContent = r.model || r.name;
    $("activity-pill").textContent = r.connected ? (ACTIVITY_LABEL[r.activity] || r.activity) : "Offline";
    const bat = $("battery");
    bat.hidden = r.battery == null;
    bat.textContent = `🔋 ${r.battery}%`;

    const busy = r.activity === "cleaning" || r.activity === "returning";
    $("btn-clean").disabled = !r.connected || r.activity === "cleaning";
    $("btn-clean").textContent = r.activity === "paused" ? "▶ Resume" : "▶ Clean";
    $("btn-pause").disabled = !r.connected || !busy;
    $("btn-dock").disabled = !r.connected || r.activity === "docked" || r.activity === "returning";
    $("clean-mode").hidden = !r.has_mop;
    $("wet-row").hidden = !r.has_mop;

    const badge = $("map-badge");
    badge.classList.toggle("locked", S.locked);
    badge.textContent = S.locked ? "🔒 Reference map" :
      S.has_floor_plan ? `Learning… (${S.runs} run${S.runs === 1 ? "" : "s"})` : "No map yet";
    $("btn-lock").textContent = S.locked ? "🔓 Unlock to relearn" : "🔒 Lock as reference";
    $("btn-lock").disabled = !S.locked && !S.has_floor_plan;
    $("map-explain").textContent = S.locked
      ? "This map is the robot's reference. Each run is lined up with it and cleaning is measured against it. Edit shapes any time; unlock to let new runs reshape it."
      : S.has_floor_plan
        ? "The floor plan is regenerated after every run from where the robot has been. It locks itself once a run adds almost nothing new – or lock it now."
        : "Start a clean and the map draws itself as the robot drives – or tap ✎ Edit map and draw your rooms by hand.";

    const missed = S.missed || [];
    $("missed-card").hidden = !missed.length || S.stats.percent == null;
    $("missed-list").innerHTML = "";
    missed.forEach((m) => {
      const li = document.createElement("li");
      li.innerHTML = `${m.area} m² <span>— tap to show</span>`;
      li.addEventListener("click", () => {
        const [W, H] = size();
        const [sx, sy] = toScreen(m.x, m.y);
        view.panX += W / 2 - sx; view.panY += H / 2 - sy; view.userMoved = true;
        draw();
        $("map-wrap").scrollIntoView({ behavior: "smooth", block: "center" });
      });
      $("missed-list").appendChild(li);
    });

    const caps = S.capabilities || {};
    const info = [
      ["Model", r.model || "–"], ["Connection", r.link],
      ["Positions", r.pose_source ? r.pose_source : (caps.rrtp === false ? "not available" : "waiting for a run")],
      ["Map", `${elements.length} shapes, ${S.stats.explored_m2} m² explored`],
    ];
    $("robot-info").innerHTML = "";
    for (const [k, v] of info) {
      const dt = document.createElement("dt"); dt.textContent = k;
      const dd = document.createElement("dd"); dd.textContent = v;
      $("robot-info").append(dt, dd);
    }
    updateSelectionPanel();
  }

  function fillSettings() {
    const s = S.settings;
    $("set-stale").value = s.stale_hours;
    $("set-auto-pct").value = s.auto_start_percent;
    $("set-auto-hours").value = s.auto_start_min_hours;
    $("set-wetness").value = String(s.mop_wetness);
    $("set-auto-lock").checked = s.auto_lock;
    $("set-keep-learning").checked = s.keep_learning;
    $("set-align").checked = s.align;
    $("set-mirror").checked = s.view_mirror;
  }

  function polyArea(pts) {
    let a = 0;
    for (let k = 0; k < pts.length; k++) {
      const [x1, y1] = pts[k], [x2, y2] = pts[(k + 1) % pts.length];
      a += x1 * y2 - x2 * y1;
    }
    return Math.abs(a / 2);
  }

  function select(id) {
    selectedId = id;
    activeVertex = null;
    updateSelectionPanel();
    draw();
  }

  function updateSelectionPanel() {
    const sel = elements.find((e) => e.id === selectedId);
    $("selection").hidden = !sel || !editing;
    if (!sel) return;
    $("sel-label").textContent = `${sel.name || KIND_LABEL[sel.kind]} · ${polyArea(sel.points).toFixed(1)} m²` +
      (sel.source === "auto" ? " (auto)" : "");
    $("sel-kind").value = sel.kind;
    $("btn-del-point").hidden = activeVertex === null || sel.points.length <= 3;
  }

  // ---- editing -------------------------------------------------------------

  function setEditing(on) {
    editing = on;
    $("btn-edit").setAttribute("aria-pressed", String(on));
    $("edit-panel").hidden = !on;
    if (!on) { draft = []; rectDraft = null; select(null); }
    updateDrawBar();
    draw();
  }

  function setTool(t) {
    tool = t;
    draft = [];
    rectDraft = null;
    document.querySelectorAll("[data-tool]").forEach((b) => b.classList.toggle("active", b.dataset.tool === t));
    updateDrawBar();
    draw();
  }

  function updateDrawBar() {
    const show = editing && tool === "poly";
    $("draw-bar").hidden = !show;
    $("draw-hint").textContent = `${KIND_LABEL[kind]}: ${draft.length} corner${draft.length === 1 ? "" : "s"}`;
    $("btn-finish").disabled = draft.length < 3;
    $("btn-undo-pt").disabled = !draft.length;
  }

  function hitShape(wx, wy) {
    const inside = (pts) => {
      let c = false;
      for (let i = 0, j = pts.length - 1; i < pts.length; j = i++) {
        const [xi, yi] = pts[i], [xj, yj] = pts[j];
        if ((yi > wy) !== (yj > wy) && wx < (xj - xi) * (wy - yi) / (yj - yi) + xi) c = !c;
      }
      return c;
    };
    const hits = elements.filter((e) => inside(e.points));
    hits.sort((a, b) => polyArea(a.points) - polyArea(b.points)); // smallest (topmost) first
    return hits[0] || null;
  }

  function handleAt(px, py) {
    const sel = elements.find((e) => e.id === selectedId);
    if (!sel) return null;
    const n = sel.points.length;
    for (let k = 0; k < n; k++) {
      const [sx, sy] = toScreen(...sel.points[k]);
      if (Math.hypot(sx - px, sy - py) <= HANDLE_PX) return { type: "vertex", idx: k, el: sel };
    }
    for (let k = 0; k < n; k++) {
      const [ax, ay] = toScreen(...sel.points[k]);
      const [bx, by] = toScreen(...sel.points[(k + 1) % n]);
      if (Math.hypot((ax + bx) / 2 - px, (ay + by) / 2 - py) <= HANDLE_PX * 0.8) return { type: "mid", idx: k, el: sel };
    }
    return null;
  }

  function localXY(ev) {
    const r = canvas.getBoundingClientRect();
    return [ev.clientX - r.left, ev.clientY - r.top];
  }

  canvas.addEventListener("pointerdown", (ev) => {
    canvas.setPointerCapture(ev.pointerId);
    const p = localXY(ev);
    pointers.set(ev.pointerId, p);
    if (pointers.size === 2) {
      const [a, b] = [...pointers.values()];
      const mid = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2];
      gesture = { type: "pinch", d0: Math.hypot(a[0] - b[0], a[1] - b[1]), s0: view.scale, world: toWorld(...mid) };
      rectDraft = null;
      draw();
      return;
    }
    if (pointers.size > 2) return;
    ev.preventDefault();
    if (editing && tool === "select") {
      const h = handleAt(...p);
      if (h) {
        const pts = h.el.points.map((q) => q.slice());
        let idx = h.idx;
        if (h.type === "mid") {
          idx = h.idx + 1;
          pts.splice(idx, 0, [snap(toWorld(...p)[0]), snap(toWorld(...p)[1])]);
        }
        activeVertex = idx;
        gesture = { type: "vertex", id: h.el.id, idx, pts, start: p, moved: h.type === "mid" };
        updateSelectionPanel();
        draw();
        return;
      }
    }
    if (editing && tool === "rect") {
      gesture = { type: "rect" };
      rectDraft = { a: p, b: p };
      return;
    }
    gesture = { type: "pan", start: p, pan0: [view.panX, view.panY], moved: false };
  });

  canvas.addEventListener("pointermove", (ev) => {
    if (!pointers.has(ev.pointerId)) return;
    const p = localXY(ev);
    pointers.set(ev.pointerId, p);
    if (!gesture) return;
    if (gesture.type === "pinch" && pointers.size >= 2) {
      const [a, b] = [...pointers.values()];
      const mid = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2];
      const d = Math.hypot(a[0] - b[0], a[1] - b[1]);
      view.scale = Math.max(4, Math.min(400, gesture.s0 * d / Math.max(gesture.d0, 1)));
      const [sx, sy] = toScreen(...gesture.world);
      view.panX += mid[0] - sx;
      view.panY += mid[1] - sy;
      view.userMoved = true;
      draw();
    } else if (gesture.type === "pan") {
      const dx = p[0] - gesture.start[0], dy = p[1] - gesture.start[1];
      if (!gesture.moved && Math.hypot(dx, dy) < 6) return;
      gesture.moved = true;
      view.panX = gesture.pan0[0] + dx;
      view.panY = gesture.pan0[1] + dy;
      view.userMoved = true;
      draw();
    } else if (gesture.type === "vertex") {
      if (!gesture.moved && Math.hypot(p[0] - gesture.start[0], p[1] - gesture.start[1]) < 4) return;
      gesture.moved = true;
      const [wx, wy] = toWorld(...p);
      gesture.pts[gesture.idx] = [snap(wx), snap(wy)];
      draw();
    } else if (gesture.type === "rect") {
      rectDraft.b = p;
      draw();
    }
  });

  function endPointer(ev) {
    if (!pointers.has(ev.pointerId)) return;
    const p = localXY(ev);
    pointers.delete(ev.pointerId);
    const g = gesture;
    if (pointers.size > 0) { if (g && g.type !== "pinch") gesture = null; return; }
    gesture = null;
    if (!g) return;
    if (g.type === "pan" && !g.moved && ev.type === "pointerup") tap(...p);
    else if (g.type === "vertex") {
      if (g.moved) post("/api/shapes/update", { id: g.id, points: g.pts });
      draw();
    } else if (g.type === "rect") {
      const { a, b } = rectDraft;
      rectDraft = null;
      if (Math.abs(b[0] - a[0]) > 12 && Math.abs(b[1] - a[1]) > 12 && ev.type === "pointerup") {
        const pts = [a, [b[0], a[1]], b, [a[0], b[1]]].map((q) => toWorld(...q).map(snap));
        post("/api/shapes", { kind, points: pts });
      }
      draw();
    }
  }
  canvas.addEventListener("pointerup", endPointer);
  canvas.addEventListener("pointercancel", endPointer);

  function tap(px, py) {
    if (!editing) return;
    const [wx, wy] = toWorld(px, py);
    if (tool === "poly") {
      draft.push([snap(wx), snap(wy)]);
      updateDrawBar();
      draw();
    } else if (tool === "select") {
      const hit = hitShape(wx, wy);
      select(hit ? hit.id : null);
    }
  }

  canvas.addEventListener("wheel", (ev) => {
    ev.preventDefault();
    const p = localXY(ev);
    const w = toWorld(...p);
    view.scale = Math.max(4, Math.min(400, view.scale * Math.exp(-ev.deltaY * 0.0015)));
    const [sx, sy] = toScreen(...w);
    view.panX += p[0] - sx;
    view.panY += p[1] - sy;
    view.userMoved = true;
    draw();
  }, { passive: false });

  // ---- buttons -------------------------------------------------------------

  $("btn-clean").addEventListener("click", () => {
    if (S && S.robot.activity === "paused") post("/api/command", { command: "resume" });
    else post("/api/command", { command: "clean", mode: $("clean-mode").value || null });
  });
  $("btn-pause").addEventListener("click", () => post("/api/command", { command: "pause" }));
  $("btn-dock").addEventListener("click", () => post("/api/command", { command: "dock" }));

  $("btn-fit").addEventListener("click", () => { view.userMoved = false; fit(); draw(); });
  $("btn-rotate").addEventListener("click", () => {
    const r = (((S && S.settings.view_rotation) || 0) + 90) % 360;
    view.userMoved = false;
    post("/api/settings", { view_rotation: r });
  });
  $("btn-edit").addEventListener("click", () => setEditing(!editing));

  document.querySelectorAll("[data-kind]").forEach((b) => b.addEventListener("click", () => {
    kind = b.dataset.kind;
    document.querySelectorAll("[data-kind]").forEach((o) => o.classList.toggle("active", o === b));
    if (tool === "select") setTool("rect");
    updateDrawBar();
  }));
  document.querySelectorAll("[data-tool]").forEach((b) => b.addEventListener("click", () => setTool(b.dataset.tool)));

  $("btn-undo-pt").addEventListener("click", () => { draft.pop(); updateDrawBar(); draw(); });
  $("btn-cancel").addEventListener("click", () => { draft = []; setTool("select"); });
  $("btn-finish").addEventListener("click", async () => {
    if (draft.length < 3) return;
    const pts = draft;
    draft = [];
    updateDrawBar();
    await post("/api/shapes", { kind, points: pts });
    setTool("select");
  });

  $("sel-kind").addEventListener("change", () => {
    if (selectedId !== null) post("/api/shapes/update", { id: selectedId, kind: $("sel-kind").value });
  });
  $("btn-del-shape").addEventListener("click", () => {
    const sel = elements.find((e) => e.id === selectedId);
    if (sel && confirm(`Delete this ${KIND_LABEL[sel.kind].toLowerCase()}?`)) {
      post("/api/shapes/delete", { id: sel.id });
      select(null);
    }
  });
  $("btn-del-point").addEventListener("click", () => {
    const sel = elements.find((e) => e.id === selectedId);
    if (!sel || activeVertex === null || sel.points.length <= 3) return;
    const pts = sel.points.filter((_, k) => k !== activeVertex);
    activeVertex = null;
    post("/api/shapes/update", { id: sel.id, points: pts });
  });

  $("btn-lock").addEventListener("click", () => post("/api/map", { action: S && S.locked ? "unlock" : "lock" }));
  $("btn-rebuild").addEventListener("click", () => {
    if (confirm("Regenerate the automatic shapes from everywhere the robot has driven? Shapes you drew or edited are kept.")) {
      post("/api/map", { action: "rebuild" });
    }
  });
  $("btn-reset-cov").addEventListener("click", () => {
    if (confirm("Forget when each spot was cleaned? The map itself is kept.")) post("/api/map", { action: "reset_coverage" });
  });
  $("btn-erase").addEventListener("click", () => {
    if (confirm("Erase the whole map, including shapes you drew? It will be relearned on the next clean.")) {
      post("/api/map", { action: "erase" });
      view.userMoved = false;
    }
  });
  $("btn-export").addEventListener("click", async () => {
    try {
      const res = await fetch("/api/export", { headers: { "X-Pin": pin } });
      if (!res.ok) throw new Error(res.statusText);
      const url = URL.createObjectURL(await res.blob());
      const a = document.createElement("a");
      a.href = url;
      a.download = "roomba-map.geojson";
      a.click();
      setTimeout(() => URL.revokeObjectURL(url), 5000);
    } catch (err) {
      setNotice("⚠ Export failed: " + err.message);
    }
  });

  $("settings-form").addEventListener("submit", async (ev) => {
    ev.preventDefault();
    await post("/api/settings", {
      stale_hours: Number($("set-stale").value),
      auto_start_percent: Number($("set-auto-pct").value),
      auto_start_min_hours: Number($("set-auto-hours").value),
      mop_wetness: Number($("set-wetness").value),
      auto_lock: $("set-auto-lock").checked,
      keep_learning: $("set-keep-learning").checked,
      align: $("set-align").checked,
      view_mirror: $("set-mirror").checked,
    });
    fillSettings();
  });


  // ---- walk-to-map ---------------------------------------------------------
  // Trace rooms by walking along the walls with the phone. Steps give
  // distance, the gyroscope gives direction, and a corner is recorded when
  // you keep walking in a new direction (walk.js). The server turns the
  // walked path into the outline (closing it, squaring near-right angles,
  // and moving it out to the walls).

  const walk = {
    active: false, drawing: false, kind: "floor", tracker: null,
    heading: new Walk.HeadingTracker(), counter: new Walk.StepCounter(),
    trail: [], stepLen: 0.7, gap: 0.3, square: true, shapesMade: 0,
    sensorSeen: false, gyro: false, wakeLock: null, lastDraw: 0,
  };
  try {
    const saved = parseFloat(localStorage.getItem("roomba-step-len"));
    if (saved > 0.3 && saved < 1.2) { walk.stepLen = saved; $("walk-step").value = Math.round(saved * 100); }
  } catch (_) { /* storage blocked */ }

  const walkPosition = () => walk.tracker.position();
  const walkFacing = () => (walk.gyro ? walk.heading.heading : walk.tracker.legHeading());
  const walkNote = (text) => { $("walk-note").textContent = text; };

  function updateWalkPanel() {
    $("walk-intro").hidden = walk.active;
    $("walk-live").hidden = !walk.active;
    $("btn-walk").setAttribute("aria-pressed", String(!$("walk-card").hidden));
    if (!walk.active) return;
    const t = walk.tracker;
    const what = walk.kind === "floor" ? "a room" : "furniture";
    const walls = t.legs.length;
    $("walk-what").textContent = walk.drawing
      ? `Tracing ${what} – ${walls} corner${walls === 1 ? "" : "s"}` : "Walking (not drawing)";
    $("walk-steps").textContent = walk.counter.steps;
    if (document.activeElement !== $("walk-dist")) $("walk-dist").value = t.legDist().toFixed(2);
    $("walk-manual").hidden = walk.gyro;
    $("walk-drawing-btns").hidden = !walk.drawing;
    $("walk-moving-btns").hidden = walk.drawing;
    $("walk-finish").textContent = walk.drawing && walk.kind === "obstacle" ? "✓ Finish furniture" : "✓ Finish room";
    $("walk-finish").disabled = walls < 2;
    $("walk-undo").disabled = !walls;
  }

  function learnStepLength(perStep) {
    if (!perStep) return;
    walk.stepLen = Math.min(1.2, Math.max(0.3, 0.5 * walk.stepLen + 0.5 * perStep));
    walk.tracker.stepLen = walk.stepLen;
    $("walk-step").value = Math.round(walk.stepLen * 100);
    try { localStorage.setItem("roomba-step-len", String(walk.stepLen)); } catch (_) { /* ignore */ }
  }

  function announceTurn(result) {
    if (!result) return;
    learnStepLength(result.learned);
    const deg = Math.round(Math.abs(result.angle));
    $("walk-turn").textContent = deg < 5 ? "" :
      `${result.angle > 0 ? "↰ Turned left" : "↱ Turned right"} ${deg}° – corner added`;
    if (navigator.vibrate) navigator.vibrate(60);
  }

  function onMotion(ev) {
    const a = ev.accelerationIncludingGravity;
    if (!a || a.x === null) return;
    walk.sensorSeen = true;
    walk.heading.addGravity(a.x, a.y, a.z);
    const r = ev.rotationRate;
    if (r && r.alpha !== null && r.alpha !== undefined) {
      walk.heading.addRotation(r.alpha, r.beta, r.gamma, ev.timeStamp);
      if (!walk.gyro && walk.heading.samples > 5) { walk.gyro = true; updateWalkPanel(); }
    }
    if (walk.counter.add(a.x, a.y, a.z, ev.timeStamp)) {
      announceTurn(walk.tracker.step(walkFacing()));
      updateWalkPanel();
      draw();
    } else if (ev.timeStamp - walk.lastDraw > 250) {
      walk.lastDraw = ev.timeStamp;  // keep the facing arrow live while turning
      draw();
    }
  }

  async function startSensors() {
    walk.sensorSeen = false;
    walk.gyro = false;
    if (!window.isSecureContext) {
      walkNote("Motion sensors need the app opened over https (start the server with --https). " +
               "Until then, type each wall's length and tap the turns yourself.");
      return;
    }
    try {
      if (typeof DeviceMotionEvent !== "undefined" && typeof DeviceMotionEvent.requestPermission === "function") {
        const answer = await DeviceMotionEvent.requestPermission(); // the iPhone asks the user
        if (answer !== "granted") {
          walkNote("Motion access was declined, so steps and turns can't be sensed. Type each wall's length " +
                   "and tap the turns (reload and tap Start again to be asked again).");
          return;
        }
      }
      window.addEventListener("devicemotion", onMotion);
      walkNote("Walk at a steady pace. Corners are added when you keep going in a new direction.");
      setTimeout(() => {
        if (!walk.active) return;
        if (!walk.sensorSeen) walkNote("No motion data from this phone/browser – type each wall's length and tap the turns.");
        else if (!walk.gyro) walkNote("Steps are counted, but this phone gives no gyroscope readings – tap each turn.");
      }, 3000);
    } catch (err) {
      walkNote("Couldn't use the motion sensors (" + err.message + "). Type each wall's length and tap the turns.");
    }
    try {
      if (navigator.wakeLock) walk.wakeLock = await navigator.wakeLock.request("screen");
    } catch (_) { /* the screen may dim; harmless */ }
  }

  function stopSensors() {
    window.removeEventListener("devicemotion", onMotion);
    if (walk.wakeLock) { walk.wakeLock.release().catch(() => {}); walk.wakeLock = null; }
  }

  function newTracker(start, heading) {
    walk.tracker = new Walk.WalkTracker({ start, heading, stepLen: walk.stepLen });
  }

  function walkStart() {
    walk.gap = Math.max(0, (parseFloat($("walk-gap").value) || 30) / 100);
    walk.stepLen = Math.min(1.2, Math.max(0.3, (parseFloat($("walk-step").value) || 70) / 100));
    walk.square = $("walk-square").checked;
    walk.heading = new Walk.HeadingTracker();
    walk.heading.reverse = $("walk-reverse").checked;
    walk.counter = new Walk.StepCounter();
    // The dock's centre is ~17 cm from its wall; you stand `gap` from that wall.
    newTracker([Math.max(0, walk.gap - 0.17), 0], 0);
    walk.trail = [];
    walk.drawing = true;
    walk.kind = "floor";
    walk.active = true;
    $("walk-turn").textContent = "";
    document.body.classList.add("walking");
    startSensors();  // straight from the tap, as iPhones require
    view.userMoved = false;
    updateWalkPanel();
    resizeCanvas();
    fit();
    draw();
    const mapTop = document.querySelector(".map-card").getBoundingClientRect().top + window.scrollY;
    window.scrollTo(0, Math.max(0, mapTop - document.querySelector("header").offsetHeight - 8));
  }

  async function walkFinishShape() {
    const path = walk.tracker.points();
    walk.shapesMade += 1;
    const name = walk.kind === "floor" ? `Walked room ${walk.shapesMade}` : `Walked furniture ${walk.shapesMade}`;
    const res = await post("/api/walk", { kind: walk.kind, path, gap: walk.gap, square: walk.square, name });
    if (!res) { walk.shapesMade -= 1; return; }
    // you are back where the shape started
    walk.trail.push(...path);
    newTracker(path[0], walkFacing());
    walk.drawing = false;
    $("walk-turn").textContent = "";
    walkNote(`Saved "${name}". Walk to the next room or piece of furniture and start tracing it – ` +
             "or tap Stop walking. Fix anything later in ✎ Edit map.");
    updateWalkPanel();
    draw();
  }

  function walkBeginShape(kind) {
    walk.trail.push(...walk.tracker.points());
    newTracker(walkPosition(), walkFacing());
    walk.kind = kind;
    walk.drawing = true;
    $("walk-turn").textContent = "";
    walkNote(kind === "floor"
      ? "Walk around this room along its walls. Finish back where you started."
      : "Walk all the way around the furniture, about 30 cm from it. Finish back where you started.");
    updateWalkPanel();
    draw();
  }

  function walkStop() {
    if (walk.drawing && walk.tracker.legs.length && !confirm("Stop and throw away the shape you are tracing?")) return;
    walk.active = false;
    document.body.classList.remove("walking");
    stopSensors();
    $("walk-card").hidden = true;
    walkNote("");
    updateWalkPanel();
    view.userMoved = false;
    resizeCanvas();
    fit();
    draw();
  }

  function walkPathPoints() {
    return walk.trail.concat(walk.tracker ? walk.tracker.points() : []);
  }

  function drawWalk() {
    const pts = walkPathPoints();
    ctx.strokeStyle = colors.select;
    ctx.lineWidth = 3;
    ctx.setLineDash([8, 5]);
    ctx.beginPath();
    pts.forEach(([x, y], k) => {
      const [sx, sy] = toScreen(x, y);
      if (k) ctx.lineTo(sx, sy); else ctx.moveTo(sx, sy);
    });
    ctx.stroke();
    ctx.setLineDash([]);
    if (walk.drawing) {
      const [sx, sy] = toScreen(...walk.tracker.start);
      ctx.lineWidth = 2;
      ctx.beginPath(); ctx.arc(sx, sy, 9, 0, Math.PI * 2); ctx.stroke();
    }
    const [x, y] = walkPosition();
    const h = walkFacing();
    const [px, py] = toScreen(x, y);
    const [tx, ty] = toScreen(x + Math.cos(h) * 0.45, y + Math.sin(h) * 0.45);
    ctx.fillStyle = colors.select;
    ctx.beginPath(); ctx.arc(px, py, 8, 0, Math.PI * 2); ctx.fill();
    ctx.lineWidth = 3;
    ctx.beginPath(); ctx.moveTo(px, py); ctx.lineTo(tx, ty); ctx.stroke();
    ctx.fillStyle = colors.text;
    ctx.font = "13px system-ui, sans-serif";
    ctx.fillText("🚶", px, py - 18);
  }

  $("btn-walk").addEventListener("click", () => {
    const card = $("walk-card");
    if (!card.hidden && walk.active) { walkStop(); return; }
    card.hidden = !card.hidden;
    if (!card.hidden && editing) setEditing(false);
    updateWalkPanel();
    if (!card.hidden) card.scrollIntoView({ behavior: "smooth", block: "start" });
  });
  $("walk-start").addEventListener("click", walkStart);
  document.querySelectorAll("[data-turn]").forEach((b) => b.addEventListener("click", () => {
    const deg = Math.min(180, Math.max(1, parseFloat($("walk-angle").value) || 90));
    announceTurn(walk.tracker.manualTurn(Number(b.dataset.turn) * deg));
    updateWalkPanel();
    draw();
  }));
  $("walk-dist").addEventListener("input", () => {
    const v = parseFloat($("walk-dist").value);
    walk.tracker.setLength(Number.isFinite(v) && v >= 0 ? v : null);
    draw();
  });
  $("walk-finish").addEventListener("click", walkFinishShape);
  $("walk-cancel-shape").addEventListener("click", () => {
    if (walk.tracker.legs.length && !confirm("Throw away the shape you are tracing?")) return;
    walk.trail.push(...walk.tracker.points());
    newTracker(walkPosition(), walkFacing());
    walk.drawing = false;
    updateWalkPanel();
    draw();
  });
  $("walk-new-room").addEventListener("click", () => walkBeginShape("floor"));
  $("walk-new-obstacle").addEventListener("click", () => walkBeginShape("obstacle"));
  $("walk-undo").addEventListener("click", () => {
    if (walk.tracker.undoCorner()) $("walk-turn").textContent = "Corner removed";
    updateWalkPanel();
    draw();
  });
  $("walk-stop").addEventListener("click", walkStop);
  $("walk-reverse").addEventListener("change", () => { walk.heading.reverse = $("walk-reverse").checked; });

  // ---- layout & theme ------------------------------------------------------

  const ro = new ResizeObserver(() => { resizeCanvas(); if (!view.userMoved) fit(); draw(); });
  ro.observe($("map-wrap"));
  const mq = window.matchMedia("(prefers-color-scheme: dark)");
  if (mq.addEventListener) mq.addEventListener("change", () => { hatch = null; readColors(); draw(); });

  resizeCanvas();
  poll();
})();
