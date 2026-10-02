"use strict";

(() => {
  const POLL_MS = 700;
  const PAINT_TOOLS = new Set(["clean", "dirty", "obstacle", "nogo", "unknown"]);
  const STATE_CODE = { unknown: "0", dirty: "1", clean: "2", obstacle: "3", nogo: "4" };
  const MODE_LABEL = { idle: "Idle", auto: "Cleaning", manual: "Manual", returning: "Docking" };

  const $ = (id) => document.getElementById(id);
  const canvas = $("map");
  const ctx = canvas.getContext("2d");

  let state = null;
  let tool = "goto";
  let cellPx = 20;
  let painting = null; // {cells: Map, state}
  let pin = "";
  try { pin = localStorage.getItem("roomba-pin") || ""; } catch (_) { /* storage blocked */ }

  // ---- API -----------------------------------------------------------------

  async function api(path, body) {
    const opts = { headers: { "X-Pin": pin } };
    if (body !== undefined) {
      opts.method = "POST";
      opts.headers["Content-Type"] = "application/json";
      opts.body = JSON.stringify(body);
    }
    const res = await fetch(path, opts);
    const data = await res.json().catch(() => ({}));
    if (res.status === 401) { askPin(); throw new Error("PIN required"); }
    if (!res.ok) throw new Error(data.error || res.statusText);
    return data;
  }

  async function post(path, body) {
    try {
      render(await api(path, body));
    } catch (err) {
      setStatus("⚠ " + err.message);
    }
  }

  async function poll() {
    try {
      if (!painting) render(await api("/api/state"));
    } catch (err) {
      if (err.message !== "PIN required") setStatus("⚠ Lost connection to robot server – retrying…");
    } finally {
      setTimeout(poll, document.hidden ? POLL_MS * 4 : POLL_MS);
    }
  }

  function askPin() {
    const dlg = $("pin-dialog");
    if (!dlg.open) dlg.showModal();
  }
  $("pin-form").addEventListener("submit", () => {
    pin = $("pin-input").value.trim();
    try { localStorage.setItem("roomba-pin", pin); } catch (_) { /* ignore */ }
  });

  // ---- rendering -------------------------------------------------------------

  function cssVar(name) {
    return getComputedStyle(document.documentElement).getPropertyValue(name).trim();
  }

  function setStatus(text) { $("status").textContent = text; }

  function layout() {
    if (!state) return;
    const wrap = $("map-wrap");
    const maxW = wrap.clientWidth;
    const maxH = Math.max(240, window.innerHeight * 0.62);
    cellPx = Math.max(4, Math.floor(Math.min(maxW / state.width, maxH / state.height)));
    const w = cellPx * state.width, h = cellPx * state.height;
    const dpr = window.devicePixelRatio || 1;
    canvas.style.width = w + "px";
    canvas.style.height = h + "px";
    canvas.width = Math.round(w * dpr);
    canvas.height = Math.round(h * dpr);
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  }

  function render(next) {
    const resized = !state || next.width !== state.width || next.height !== state.height;
    state = next;
    if (resized) { layout(); fillForms(); }
    draw();
    updatePanel();
  }

  function draw() {
    if (!state) return;
    const { width, height } = state;
    const colors = {
      "0": cssVar("--c-unknown"), "1": cssVar("--c-dirty"), "2": cssVar("--c-clean"),
      "3": cssVar("--c-obstacle"), "4": cssVar("--c-nogo"),
    };
    let cells = state.cells;
    if (painting) {
      const arr = cells.split("");
      for (const key of painting.cells.keys()) {
        const [x, y] = key.split(",").map(Number);
        arr[y * width + x] = STATE_CODE[painting.state];
      }
      cells = arr.join("");
    }
    ctx.clearRect(0, 0, width * cellPx, height * cellPx);
    for (let y = 0; y < height; y++) {
      for (let x = 0; x < width; x++) {
        const c = cells[y * width + x];
        ctx.fillStyle = colors[c];
        ctx.fillRect(x * cellPx, y * cellPx, cellPx, cellPx);
        if (c === "4") {
          ctx.strokeStyle = "rgba(255,255,255,.55)";
          ctx.lineWidth = 1;
          ctx.beginPath();
          ctx.moveTo(x * cellPx, (y + 1) * cellPx);
          ctx.lineTo((x + 1) * cellPx, y * cellPx);
          ctx.stroke();
        }
      }
    }
    if (cellPx >= 8) {
      ctx.strokeStyle = cssVar("--c-grid");
      ctx.lineWidth = 1;
      ctx.beginPath();
      for (let x = 0; x <= width; x++) { ctx.moveTo(x * cellPx + .5, 0); ctx.lineTo(x * cellPx + .5, height * cellPx); }
      for (let y = 0; y <= height; y++) { ctx.moveTo(0, y * cellPx + .5); ctx.lineTo(width * cellPx, y * cellPx + .5); }
      ctx.stroke();
    }

    // planned path
    const center = (v) => v * cellPx + cellPx / 2;
    if (state.path.length) {
      ctx.strokeStyle = cssVar("--c-path");
      ctx.globalAlpha = 0.6;
      ctx.lineWidth = Math.max(2, cellPx / 8);
      ctx.setLineDash([cellPx / 4, cellPx / 4]);
      ctx.beginPath();
      ctx.moveTo(center(state.robot.x), center(state.robot.y));
      for (const [x, y] of state.path) ctx.lineTo(center(x), center(y));
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.globalAlpha = 1;
    }

    // dock
    const [dx, dy] = state.dock;
    ctx.fillStyle = cssVar("--text");
    ctx.font = `${Math.floor(cellPx * 0.8)}px system-ui, sans-serif`;
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText("⌂", center(dx), center(dy) + 1);

    // robot
    const rx = center(state.robot.x), ry = center(state.robot.y), r = cellPx * 0.4;
    ctx.fillStyle = cssVar("--c-robot");
    ctx.beginPath(); ctx.arc(rx, ry, r, 0, Math.PI * 2); ctx.fill();
    const ang = { N: -Math.PI / 2, E: 0, S: Math.PI / 2, W: Math.PI }[state.robot.heading] || 0;
    ctx.fillStyle = "#fff";
    ctx.beginPath();
    ctx.moveTo(rx + Math.cos(ang) * r * 0.85, ry + Math.sin(ang) * r * 0.85);
    ctx.lineTo(rx + Math.cos(ang + 2.4) * r * 0.5, ry + Math.sin(ang + 2.4) * r * 0.5);
    ctx.lineTo(rx + Math.cos(ang - 2.4) * r * 0.5, ry + Math.sin(ang - 2.4) * r * 0.5);
    ctx.closePath(); ctx.fill();
  }

  function updatePanel() {
    const s = state.stats;
    $("stat-percent").textContent = s.percent + "%";
    $("stat-remaining").textContent = s.remaining;
    $("stat-area").textContent = s.area_m2;
    $("stat-obstacles").textContent = s.obstacles;
    $("progress-bar").style.width = s.percent + "%";
    $("mode-pill").textContent = MODE_LABEL[state.mode] || state.mode;
    setStatus(state.status + (state.robot.driver === "simulator" ? "  (simulated robot)" : ""));
    document.querySelectorAll(".controls button").forEach((b) => {
      b.classList.toggle("active", b.dataset.mode === state.mode);
    });
  }

  function fillForms() {
    if (!state) return;
    $("set-stale").value = state.settings.stale_hours;
    $("set-manual-clean").checked = state.settings.clean_in_manual;
    $("set-width").value = state.width;
    $("set-height").value = state.height;
    $("cell-size-hint").textContent =
      `Each cell is ${state.cell_cm} cm – the map covers ` +
      `${(state.width * state.cell_cm / 100).toFixed(1)} × ${(state.height * state.cell_cm / 100).toFixed(1)} m.`;
  }

  // ---- map interaction -----------------------------------------------------

  function cellAt(ev) {
    const rect = canvas.getBoundingClientRect();
    const x = Math.floor((ev.clientX - rect.left) / cellPx);
    const y = Math.floor((ev.clientY - rect.top) / cellPx);
    if (!state || x < 0 || y < 0 || x >= state.width || y >= state.height) return null;
    return [x, y];
  }

  canvas.addEventListener("pointerdown", (ev) => {
    const cell = cellAt(ev);
    if (!cell) return;
    ev.preventDefault();
    if (PAINT_TOOLS.has(tool)) {
      canvas.setPointerCapture(ev.pointerId);
      painting = { cells: new Map([[cell.join(","), cell]]), state: tool };
      draw();
    } else if (tool === "goto") {
      post("/api/goto", { x: cell[0], y: cell[1] });
    } else if (tool === "dock") {
      post("/api/dock", { x: cell[0], y: cell[1] });
    } else if (tool === "robot") {
      post("/api/robot", { x: cell[0], y: cell[1] });
    }
  });

  canvas.addEventListener("pointermove", (ev) => {
    if (!painting) return;
    const cell = cellAt(ev);
    if (cell && !painting.cells.has(cell.join(","))) {
      painting.cells.set(cell.join(","), cell);
      draw();
    }
  });

  function finishPaint() {
    if (!painting) return;
    const job = painting;
    painting = null;
    post("/api/cells", { cells: [...job.cells.values()], state: job.state });
  }
  canvas.addEventListener("pointerup", finishPaint);
  canvas.addEventListener("pointercancel", finishPaint);

  // ---- buttons -------------------------------------------------------------

  document.querySelectorAll(".controls button").forEach((b) =>
    b.addEventListener("click", () => post("/api/mode", { mode: b.dataset.mode })));

  document.querySelectorAll("#tools button").forEach((b) =>
    b.addEventListener("click", () => {
      tool = b.dataset.tool;
      document.querySelectorAll("#tools button").forEach((o) => o.classList.toggle("active", o === b));
    }));

  document.querySelectorAll(".dpad button").forEach((b) =>
    b.addEventListener("click", () => post("/api/drive", { direction: b.dataset.dir })));

  document.addEventListener("keydown", (ev) => {
    if (ev.target.tagName === "INPUT") return;
    const dir = { ArrowUp: "N", ArrowDown: "S", ArrowLeft: "W", ArrowRight: "E" }[ev.key];
    if (dir) { ev.preventDefault(); post("/api/drive", { direction: dir }); }
  });

  $("settings-form").addEventListener("submit", (ev) => {
    ev.preventDefault();
    post("/api/settings", {
      stale_hours: Number($("set-stale").value),
      clean_in_manual: $("set-manual-clean").checked,
    });
  });

  $("resize-form").addEventListener("submit", (ev) => {
    ev.preventDefault();
    post("/api/resize", { width: Number($("set-width").value), height: Number($("set-height").value) });
  });

  $("btn-new-pass").addEventListener("click", () => {
    if (confirm("Mark every cleaned cell as needing cleaning again? The layout is kept.")) {
      post("/api/reset", { scope: "pass" });
    }
  });
  $("btn-erase").addEventListener("click", () => {
    if (confirm("Erase the whole map, including obstacles and no-go zones?")) {
      post("/api/reset", { scope: "all" });
    }
  });

  let resizeTimer;
  window.addEventListener("resize", () => {
    clearTimeout(resizeTimer);
    resizeTimer = setTimeout(() => { layout(); draw(); }, 100);
  });
  window.matchMedia("(prefers-color-scheme: dark)").addEventListener?.("change", draw);

  poll();
})();
