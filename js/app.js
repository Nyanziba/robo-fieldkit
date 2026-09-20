/* RoboFieldKit — client-side robotics field toolkit
 * No framework, no dependencies, fully offline-capable.
 * Math lives in kinematics.js (window.RFK); this file wires the DOM.
 */
(function () {
  "use strict";

  const RFK = window.RFK;

  // ---------- helpers ----------

  const $ = (id) => document.getElementById(id);

  function parseNum(el) {
    const raw = (el.value || "").trim();
    if (raw === "") return NaN;
    return Number(raw);
  }

  function isFiniteNum(v) { return typeof v === "number" && Number.isFinite(v); }

  function fmt(x, digits) {
    digits = digits === undefined ? 4 : digits;
    if (!isFiniteNum(x)) return "—";
    const a = Math.abs(x);
    if (a !== 0 && (a < 1e-4 || a >= 1e7)) return x.toExponential(3);
    return x.toFixed(digits);
  }

  function row(key, value, unit) {
    const u = unit ? '<span class="v unit">' + unit + "</span>" : "";
    return '<div class="result-row"><span class="k">' + key + '</span><span class="v">' + value + u + "</span></div>";
  }

  function setError(id, msg) {
    const el = $(id);
    if (msg) { el.textContent = msg; el.classList.remove("hidden"); }
    else el.classList.add("hidden");
  }

  function setResults(id, html) { $(id).innerHTML = html; }

  function on(id, evt, fn) { $(id).addEventListener(evt, fn); }

  // ---------- 1. differential-drive odometry ----------

  function updateOdom() {
    const r = parseNum($("odom-r"));
    const L = parseNum($("odom-l"));
    const nl = parseNum($("odom-nl"));
    const nr = parseNum($("odom-nr"));
    const tpr = parseNum($("odom-tpr"));

    const bad = [];
    if (!isFiniteNum(r) || r <= 0) bad.push("車輪半径 r は 0 より大きい数値");
    if (!isFiniteNum(L) || L <= 0) bad.push("車軸幅 L は 0 より大きい数値");
    if (!isFiniteNum(tpr) || tpr <= 0) bad.push("1回転パルス数は 0 より大きい数値");
    if (!isFiniteNum(nl)) bad.push("左エンコーダ値が不正");
    if (!isFiniteNum(nr)) bad.push("右エンコーダ値が不正");

    if (bad.length) { setError("odom-error", bad.join(" · ")); return; }
    setError("odom-error", null);

    const o = RFK.odometry(r, L, nl, nr, tpr);
    let html = "";
    html += row("左輪 移動距離", fmt(o.dl) + " m");
    html += row("右輪 移動距離", fmt(o.dr) + " m");
    html += row("平均移動距離 d", fmt(o.d) + " m");
    html += row("方位角変化 Δθ", fmt(o.dtheta) + " rad", "= " + fmt(RFK.deg(o.dtheta)) + " °");
    html += row("新位置 x", fmt(o.x) + " m");
    html += row("新位置 y", fmt(o.y) + " m");
    html += row("新方位 θ", fmt(o.theta) + " rad", "= " + fmt(RFK.deg(o.theta)) + " °");
    setResults("odom-results", html);
  }

  // ---------- 2. quaternion ⇄ euler ⇄ matrix ----------

  function renderQuat(m) {
    const e = RFK.matrixToEuler(m);
    let html = "";
    html += '<div class="result-row"><span class="k">オイラー角 (roll / pitch / yaw)</span><span class="v">' +
      fmt(RFK.deg(e.roll), 3) + "° / " + fmt(RFK.deg(e.pitch), 3) + "° / " + fmt(RFK.deg(e.yaw), 3) + "°</span></div>";
    html += '<div class="result-row"><span class="k">回転行列</span></div>';
    html += matrixTable(m);
    setResults("quat-results", html);
  }

  function matrixTable(m) {
    let html = '<table class="matrix-table"><tr><td class="heading"></td><td class="heading">X</td><td class="heading">Y</td><td class="heading">Z</td></tr>';
    const rows = ["X", "Y", "Z"];
    for (let i = 0; i < 3; i++) {
      html += "<tr><td class='heading'>" + rows[i] + "</td>";
      for (let j = 0; j < 3; j++) html += "<td>" + fmt(m[i][j], 5) + "</td>";
      html += "</tr>";
    }
    html += "</table>";
    return html;
  }

  function quatFromInputs() {
    return { x: parseNum($("q-x")), y: parseNum($("q-y")), z: parseNum($("q-z")), w: parseNum($("q-w")) };
  }

  function eulerFromInputs() {
    return { roll: RFK.rad(parseNum($("e-roll"))), pitch: RFK.rad(parseNum($("e-pitch"))), yaw: RFK.rad(parseNum($("e-yaw"))) };
  }

  function quatToEulerMatrix() {
    const q = quatFromInputs();
    if (![q.x, q.y, q.z, q.w].every(isFiniteNum)) {
      setError("quat-error", "クォータニオンの各成分は数値で入力してください。");
      return;
    }
    const qn = RFK.normalizeQuat(q);
    if (!qn) { setError("quat-error", "クォータニオンのノルムが 0 です。"); return; }
    setError("quat-error", null);
    renderQuat(RFK.quatToMatrix(qn));
  }

  function eulerToQuatMatrix() {
    const e = eulerFromInputs();
    if (![e.roll, e.pitch, e.yaw].every(isFiniteNum)) {
      setError("quat-error", "オイラー角は数値（度）で入力してください。");
      return;
    }
    setError("quat-error", null);
    const q = RFK.eulerToQuat(e.roll, e.pitch, e.yaw);
    const m = RFK.eulerToMatrix(e.roll, e.pitch, e.yaw);
    $("q-x").value = fmt(q.x, 6);
    $("q-y").value = fmt(q.y, 6);
    $("q-z").value = fmt(q.z, 6);
    $("q-w").value = fmt(q.w, 6);
    renderQuat(m);
  }

  function matrixToQuatEuler() {
    const m = matrixFromInputs();
    if (!m) { setError("quat-error", "回転行列の各成分は数値で入力してください。"); return; }
    setError("quat-error", null);
    const q = RFK.matrixToQuat(m);
    const e = RFK.matrixToEuler(m);
    $("q-x").value = fmt(q.x, 6);
    $("q-y").value = fmt(q.y, 6);
    $("q-z").value = fmt(q.z, 6);
    $("q-w").value = fmt(q.w, 6);
    $("e-roll").value = fmt(RFK.deg(e.roll), 4);
    $("e-pitch").value = fmt(RFK.deg(e.pitch), 4);
    $("e-yaw").value = fmt(RFK.deg(e.yaw), 4);
    renderQuat(m);
  }

  function normalizeQuatInputs() {
    const q = quatFromInputs();
    if (![q.x, q.y, q.z, q.w].every(isFiniteNum)) {
      setError("quat-error", "クォータニオンの各成分は数値で入力してください。");
      return;
    }
    const qn = RFK.normalizeQuat(q);
    if (!qn) { setError("quat-error", "クォータニオンのノルムが 0 です。"); return; }
    $("q-x").value = fmt(qn.x, 6);
    $("q-y").value = fmt(qn.y, 6);
    $("q-z").value = fmt(qn.z, 6);
    $("q-w").value = fmt(qn.w, 6);
    quatToEulerMatrix();
  }

  const MATRIX_IDS = ["m00", "m01", "m02", "m10", "m11", "m12", "m20", "m21", "m22"];

  function buildMatrixInputs() {
    const identity = [1, 0, 0, 0, 1, 0, 0, 0, 1];
    let html = "";
    MATRIX_IDS.forEach((id, i) => {
      html += '<input id="' + id + '" type="number" inputmode="decimal" step="any" value="' + identity[i] + '" />';
    });
    $("matrix-inputs").innerHTML = html;
  }

  function matrixFromInputs() {
    const m = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
    for (let i = 0; i < 3; i++) {
      for (let j = 0; j < 3; j++) {
        const v = parseNum($("m" + i + j));
        if (!isFiniteNum(v)) return null;
        m[i][j] = v;
      }
    }
    return m;
  }

  // ---------- 3. wheel speed / gear ratio ----------

  function updateWheel() {
    const motor = parseNum($("wh-motor"));
    const D = parseNum($("wh-d"));
    const ratio = parseNum($("wh-ratio"));
    const bad = [];
    if (!isFiniteNum(D) || D <= 0) bad.push("車輪直径 D は 0 より大きい数値");
    if (!isFiniteNum(ratio) || ratio <= 0) bad.push("ギア比は 0 より大きい数値");
    if (!isFiniteNum(motor)) bad.push("モーター回転数が不正");
    if (bad.length) { setError("wheel-error", bad.join(" · ")); return; }
    setError("wheel-error", null);

    const w = RFK.wheelFromMotor(motor, D, ratio);
    let html = "";
    html += row("車輪回転数", fmt(w.wheelRpm, 2) + " RPM");
    html += row("車輪円周", fmt(w.circ, 5) + " m");
    html += row("直進速度", fmt(w.speed, 4) + " m/s", "= " + fmt(w.speed * 3.6, 3) + " km/h");
    setResults("wheel-results", html);
    updateWheelReverse();
  }

  function updateWheelReverse() {
    const speed = parseNum($("wh-speed"));
    const D = parseNum($("wh-d"));
    const ratio = parseNum($("wh-ratio"));
    if (!isFiniteNum(speed)) { setResults("wheel-rev-results", '<div class="result-row"><span class="k">入力待ち</span><span class="v">—</span></div>'); return; }
    if (!isFiniteNum(D) || D <= 0 || !isFiniteNum(ratio) || ratio <= 0) {
      setResults("wheel-rev-results", '<div class="result-row"><span class="k">車輪径・ギア比を正しく入力してください</span><span class="v">—</span></div>');
      return;
    }
    const w = RFK.motorFromSpeed(speed, D, ratio);
    let html = "";
    html += row("必要な車輪回転数", fmt(w.wheelRpm, 2) + " RPM");
    html += row("必要なモーター回転数", fmt(w.motorRpm, 2) + " RPM");
    setResults("wheel-rev-results", html);
  }

  // ---------- 4. CAN ----------

  function updateCanId() {
    const hexEl = $("can-id-hex");
    const decEl = $("can-id-dec");
    let value = null;
    if (document.activeElement === hexEl) value = RFK.parseHexId(hexEl.value);
    else { const dv = parseInt((decEl.value || "").trim(), 10); value = Number.isFinite(dv) ? dv : null; }

    if (value === null || value < 0) {
      setError("can-error", "CAN ID は 16進数 または 10進数の 0 以上の整数で入力してください。");
      return;
    }
    setError("can-error", null);

    if (document.activeElement !== hexEl) hexEl.value = "0x" + value.toString(16).toUpperCase();
    if (document.activeElement !== decEl) decEl.value = String(value);

    const bits = value <= 0x7FF ? "11-bit" : (value <= 0x1FFFFFFF ? "29-bit" : "範囲外");
    let html = "";
    html += row("10進数", String(value));
    html += row("16進数", "0x" + value.toString(16).toUpperCase());
    html += row("11-bit (標準)", "0x" + (value & 0x7FF).toString(16).toUpperCase().padStart(3, "0"), bits === "11-bit" ? "適合" : "切り捨て");
    html += row("29-bit (拡張)", "0x" + (value & 0x1FFFFFFF).toString(16).toUpperCase().padStart(8, "0"));
    html += row("種別", bits);
    setResults("can-id-results", html);
  }

  function updateCanData() {
    const bytes = RFK.parseHexBytes($("can-data").value);
    const order = $("can-order").value;
    const bits = parseInt($("can-bits").value, 10);
    const signed = $("can-signed").value === "signed";
    const scale = parseNum($("can-scale"));
    const offset = parseNum($("can-offset"));

    if (!bytes) { setError("can-error", "データは 16進数バイト列（偶数桁）で入力してください。例: 01 02"); return; }
    if (!isFiniteNum(scale) || !isFiniteNum(offset)) { setError("can-error", "スケール・オフセットは数値で入力してください。"); return; }

    const dec = RFK.decodeCan(bytes, order, bits, signed);
    if (!dec) {
      setError("can-error", "データが不足しています。" + bits + " ビットには " + (bits / 8) + " バイト必要です。");
      return;
    }
    setError("can-error", null);
    const phys = RFK.physical(dec.rawNum, scale, offset);

    let html = "";
    html += row("RAW 整数 (符号なし)", dec.raw);
    if (signed) html += row("RAW 整数 (符号あり)", dec.rawSigned);
    html += row("RAW 16進数", "0x" + dec.hex);
    html += row("物理値", fmt(phys, 5));
    setResults("can-data-results", html);
  }

  // ---------- 5. battery ----------

  function updateBattery() {
    const ah = parseNum($("batt-ah"));
    const a = parseNum($("batt-a"));
    const bad = [];
    if (!isFiniteNum(ah) || ah < 0) bad.push("容量 [Ah] は 0 以上の数値");
    if (!isFiniteNum(a) || a <= 0) bad.push("平均消費電流 [A] は 0 より大きい数値");
    if (bad.length) { setError("batt-error", bad.join(" · ")); return; }
    setError("batt-error", null);
    const t = RFK.runtimeHours(ah, a);
    let html = "";
    html += row("推定稼働時間", fmt(t.hours, 2) + " 時間", "= " + fmt(t.minutes, 1) + " 分");
    if (t.hours >= 24) html += row("(約)", fmt(t.hours / 24, 2) + " 日");
    setResults("batt-results", html);
  }

  function updateBatteryMah() {
    const mah = parseNum($("batt-mah"));
    const ma = parseNum($("batt-ma"));
    if (!isFiniteNum(mah) || mah < 0) { setResults("batt-mah-results", '<div class="result-row"><span class="k">容量 [mAh] を 0 以上で入力</span><span class="v">—</span></div>'); return; }
    if (!isFiniteNum(ma) || ma <= 0) { setResults("batt-mah-results", '<div class="result-row"><span class="k">消費電流 [mA] を 0 より大きく入力</span><span class="v">—</span></div>'); return; }
    const t = RFK.runtimeHours(mah, ma);
    setResults("batt-mah-results", '<div class="result-row"><span class="k">推定稼働時間</span><span class="v">' + fmt(t.hours, 2) + " 時間</span><span class='v unit'>= " + fmt(t.minutes, 1) + " 分</span></div>");
  }

  // ---------- 6. mecanum / omni / diff inverse kinematics ----------

  const KIN_POS = {
    mecanum: ["前左", "前右", "後左", "後右"],
    omni: ["前左", "前右", "後左", "後右"],
    diff: ["左", "右"]
  };

  let kinType = "mecanum";
  let kinUnit = "ms";

  function kinFormula(type, pos, vx, vy, w, Lx, Ly) {
    const s = (n) => fmt(n, 3);
    if (type === "mecanum") {
      const sum = Lx + Ly;
      if (pos === "前左") return s(vx) + " − " + s(vy) + " − " + s(w) + "×(" + s(Lx) + "+" + s(Ly) + ")";
      if (pos === "前右") return s(vx) + " + " + s(vy) + " + " + s(w) + "×(" + s(Lx) + "+" + s(Ly) + ")";
      if (pos === "後左") return s(vx) + " + " + s(vy) + " − " + s(w) + "×(" + s(Lx) + "+" + s(Ly) + ")";
      return s(vx) + " − " + s(vy) + " + " + s(w) + "×(" + s(Lx) + "+" + s(Ly) + ")";
    }
    if (type === "omni") {
      const L = Math.hypot(Lx, Ly), c = Lx / L, s_ = Ly / L;
      if (pos === "前左") return "(" + s(vy) + "×" + s(c) + " − " + s(vx) + "×" + s(s_) + ") + " + s(w) + "×" + s(L);
      if (pos === "前右") return "(" + s(vx) + "×" + s(s_) + " + " + s(vy) + "×" + s(c) + ") + " + s(w) + "×" + s(L);
      if (pos === "後左") return "−(" + s(vx) + "×" + s(s_) + " + " + s(vy) + "×" + s(c) + ") + " + s(w) + "×" + s(L);
      return "(" + s(vx) + "×" + s(s_) + " − " + s(vy) + "×" + s(c) + ") + " + s(w) + "×" + s(L);
    }
    // diff
    if (pos === "左") return s(vx) + " − " + s(w) + "×" + s(Ly);
    return s(vx) + " + " + s(w) + "×" + s(Ly);
  }

  function wheelDisplay(v, r, unit) {
    if (unit === "rads") return fmt(v / r, 3) + " rad/s";
    if (unit === "rpm") return fmt(RFK.angularToRpm(v / r), 2) + " RPM";
    return fmt(v, 3) + " m/s";
  }

  function updateKin() {
    const vx = parseNum($("kin-vx"));
    const vy = parseNum($("kin-vy"));
    const w = parseNum($("kin-w"));
    const r = parseNum($("kin-r"));
    const Lx = parseNum($("kin-lx"));
    const Ly = parseNum($("kin-ly"));

    const bad = [];
    if (!isFiniteNum(vx)) bad.push("vx が不正");
    if (!isFiniteNum(vy)) bad.push("vy が不正");
    if (!isFiniteNum(w)) bad.push("ω が不正");
    if (!isFiniteNum(r) || r <= 0) bad.push("車輪半径 r は 0 より大きい数値");
    if (!isFiniteNum(Lx) || Lx <= 0) bad.push("Lx は 0 より大きい数値");
    if (!isFiniteNum(Ly) || Ly <= 0) bad.push("Ly は 0 より大きい数値");
    if (bad.length) { setError("kin-error", bad.join(" · ")); return; }
    setError("kin-error", null);

    const speeds = RFK.wheelSpeeds(kinType, vx, vy, w, Lx, Ly);
    let html = "";
    const posOrder = KIN_POS[kinType];
    posOrder.forEach((pos) => {
      const v = speeds[pos];
      html += row(pos, wheelDisplay(v, r, kinUnit),
        "角 " + fmt(v / r, 3) + " rad/s · 回転 " + fmt(RFK.angularToRpm(v / r), 1) + " RPM");
      html += '<div class="formula">' + pos + " = " + kinFormula(kinType, pos, vx, vy, w, Lx, Ly) + "</div>";
    });
    setResults("kin-results", html);
  }

  // ---------- 7. motor layout → signal / code generation ----------

  const MOTOR_POS = {
    mecanum: ["前左", "前右", "後左", "後右"],
    omni: ["前左", "前右", "後左", "後右"],
    diff: ["左", "右"]
  };

  let motorType = "mecanum";
  let lastCode = "";

  function buildMotorConfig() {
    const pos = MOTOR_POS[motorType];
    const count = pos.length;
    let html = "";
    for (let i = 0; i < count; i++) {
      let opts = "";
      pos.forEach((p) => { opts += '<option value="' + p + '"' + (p === pos[i] ? " selected" : "") + ">" + p + "</option>"; });
      html += '<div class="motor-row" data-motor="' + i + '">' +
        '<span class="motor-no">M' + (i + 1) + "</span>" +
        '<select class="motor-pos">' + opts + "</select>" +
        '<label class="polarity"><input type="checkbox" class="motor-inv" /> 反転</label>' +
        "</div>";
    }
    $("motor-config").innerHTML = html;
  }

  function readMotorConfig() {
    const rows = Array.from($("motor-config").querySelectorAll(".motor-row"));
    return rows.map((r) => ({
      pos: r.querySelector(".motor-pos").value,
      inv: r.querySelector(".motor-inv").checked
    }));
  }

  function motorPosToWheelIndex(type, pos) {
    return MOTOR_POS[type].indexOf(pos);
  }

  function updateMotor() {
    const vx = parseNum($("mot-vx"));
    const vy = parseNum($("mot-vy"));
    const w = parseNum($("mot-w"));
    const r = parseNum($("mot-r"));
    const Lx = parseNum($("mot-lx"));
    const Ly = parseNum($("mot-ly"));
    const vmax = parseNum($("mot-vmax"));

    const bad = [];
    if (!isFiniteNum(vx) || !isFiniteNum(vy) || !isFiniteNum(w)) bad.push("vx/vy/ω は数値で入力してください");
    if (!isFiniteNum(r) || r <= 0) bad.push("車輪半径 r は 0 より大きい数値");
    if (!isFiniteNum(Lx) || Lx <= 0 || !isFiniteNum(Ly) || Ly <= 0) bad.push("Lx/Ly は 0 より大きい数値");
    if (!isFiniteNum(vmax) || vmax <= 0) bad.push("最大車輪速度は 0 より大きい数値");
    if (bad.length) { setError("motor-error", bad.join(" · ")); return; }
    setError("motor-error", null);

    const speeds = RFK.wheelSpeeds(motorType, vx, vy, w, Lx, Ly);
    const config = readMotorConfig();

    let html = '<table class="motor-table"><tr><th>モーター</th><th>位置</th><th>車輪速度 [m/s]</th><th>指令</th><th>PWM [%]</th><th>モーターRPM</th></tr>';
    config.forEach((c, i) => {
      const v = speeds[c.pos];           // logical wheel speed [m/s]
      const cmd = c.inv ? -v : v;        // electrical command
      const eps = 1e-9;
      let dir, dirCls;
      if (Math.abs(cmd) < eps) { dir = "停止"; dirCls = "dir-stop"; }
      else if (cmd > 0) { dir = "正転"; dirCls = "dir-fwd"; }
      else { dir = "逆転"; dirCls = "dir-rev"; }
      const pwm = Math.round(Math.min(1, Math.abs(v) / vmax) * 100);
      const rpm = RFK.angularToRpm(cmd / r);
      html += "<tr><td>M" + (i + 1) + "</td><td>" + c.pos + "</td><td>" + fmt(v, 3) + "</td>" +
        "<td class='" + dirCls + "'>" + dir + "</td><td>" + pwm + "</td><td>" + fmt(rpm, 0) + "</td></tr>";
    });
    html += "</table>";
    setResults("motor-results", html);
  }

  function numLit(x) { return String(Number(x.toFixed(6))); }

  function wheelFormulaLinesCpp(type, Lx, Ly) {
    const lx = numLit(Lx), ly = numLit(Ly);
    if (type === "mecanum") {
      const s = numLit(Lx + Ly);
      return [
        "  vw[0] = vx - vy - w*(" + lx + " + " + ly + ");   // 前左",
        "  vw[1] = vx + vy + w*(" + lx + " + " + ly + ");   // 前右",
        "  vw[2] = vx + vy - w*(" + lx + " + " + ly + ");   // 後左",
        "  vw[3] = vx - vy + w*(" + lx + " + " + ly + ");   // 後右"
      ];
    }
    if (type === "omni") {
      const L = Math.hypot(Lx, Ly), c = Lx / L, s = Ly / L;
      return [
        "  float L = " + numLit(L) + "; float c = " + numLit(c) + "; float s = " + numLit(s) + ";",
        "  vw[0] = (vy*c - vx*s) + w*L;   // 前左",
        "  vw[1] = (vx*s + vy*c) + w*L;   // 前右",
        "  vw[2] = -(vx*s + vy*c) + w*L;  // 後左",
        "  vw[3] = (vx*s - vy*c) + w*L;   // 後右"
      ];
    }
    return [
      "  vw[0] = vx - w*" + ly + ";   // 左",
      "  vw[1] = vx + w*" + ly + ";   // 右"
    ];
  }

  function motorWheelArray(type, config) {
    return config.map((c) => motorPosToWheelIndex(type, c.pos));
  }

  function polarityArray(config) {
    return config.map((c) => (c.inv ? -1 : 1));
  }

  function arrLiteral(arr) {
    return "{ " + arr.map((n) => String(n)).join(", ") + " }";
  }

  function genArduino() {
    const r = parseNum($("mot-r"));
    const Lx = parseNum($("mot-lx"));
    const Ly = parseNum($("mot-ly"));
    const vmax = parseNum($("mot-vmax"));
    const config = readMotorConfig();
    const count = config.length;

    const pinList = count === 4 ? "{5, 6, 9, 10}" : "{5, 6}";
    const dirList = count === 4 ? "{2, 3, 4, 7}" : "{2, 3}";
    const label = { mecanum: "メカナム4輪", omni: "オムニ4輪", diff: "差動2輪" }[motorType];

    let lines = [];
    lines.push("// RoboFieldKit 生成コード（" + label + " / PWM駆動）");
    lines.push("// ピン割り当ては配線に合わせて変更してください。");
    lines.push("const int PWM_PIN[" + count + "] = " + pinList + ";  // M1..M" + count);
    lines.push("const int DIR_PIN[" + count + "] = " + dirList + ";   // M1..M" + count);
    lines.push("// モーター→車輪位置 " + (count === 4 ? "(0=前左,1=前右,2=後左,3=後右)" : "(0=左,1=右)"));
    lines.push("const int MOTOR_WHEEL[" + count + "] = " + arrLiteral(motorWheelArray(motorType, config)) + ";");
    lines.push("// 極性: 1=正転, -1=反転（配線が逆の場合）");
    lines.push("const int POLARITY[" + count + "] = " + arrLiteral(polarityArray(config)) + ";");
    lines.push("");
    lines.push("void setMotor(int m, float v, float vmax) {");
    lines.push("  int pwm = (int)constrain(fabs(v) / vmax * 255.0, 0, 255);");
    lines.push("  digitalWrite(DIR_PIN[m], v >= 0 ? HIGH : LOW);");
    lines.push("  analogWrite(PWM_PIN[m], pwm);");
    lines.push("}");
    lines.push("");
    lines.push("void drive(float vx, float vy, float w) {");
    lines.push("  const float r = " + numLit(r) + ", V_MAX = " + numLit(vmax) + ";");
    lines.push("  float vw[" + count + "];");
    wheelFormulaLinesCpp(motorType, Lx, Ly).forEach((l) => lines.push(l));
    lines.push("  for (int m = 0; m < " + count + "; m++) setMotor(m, vw[MOTOR_WHEEL[m]] * POLARITY[m], V_MAX);");
    lines.push("}");
    lines.push("");
    lines.push("void setup() {");
    lines.push("  for (int i = 0; i < " + count + "; i++) { pinMode(PWM_PIN[i], OUTPUT); pinMode(DIR_PIN[i], OUTPUT); }");
    lines.push("}");
    lines.push("");
    lines.push("void loop() {");
    lines.push("  drive(1.0, 0.0, 0.0);  // 例: 前進 1 m/s");
    lines.push("  delay(1000);");
    lines.push("  drive(0.0, 0.0, 0.0);  // 停止");
    lines.push("  delay(1000);");
    lines.push("}");
    return lines.join("\n");
  }

  function genRos() {
    const r = parseNum($("mot-r"));
    const Lx = parseNum($("mot-lx"));
    const Ly = parseNum($("mot-ly"));
    const config = readMotorConfig();
    const count = config.length;
    const label = { mecanum: "メカナム4輪", omni: "オムニ4輪", diff: "差動2輪" }[motorType];

    let lines = [];
    lines.push("// RoboFieldKit 生成コード（" + label + " / ROS2 C++）");
    lines.push("// 購読: /cmd_vel (geometry_msgs/Twist) → 各モーター速度を /wheel_cmds に配信");
    lines.push("#include <memory>");
    lines.push("#include \"rclcpp/rclcpp.hpp\"");
    lines.push("#include \"geometry_msgs/msg/twist.hpp\"");
    lines.push("#include \"std_msgs/msg/float32_multi_array.hpp\"");
    lines.push("");
    lines.push("class WheelDriver : public rclcpp::Node {");
    lines.push("public:");
    lines.push("  WheelDriver() : Node(\"wheel_driver\") {");
    lines.push("    sub_ = create_subscription<geometry_msgs::msg::Twist>(");
    lines.push("      \"cmd_vel\", 10, [this](const geometry_msgs::msg::Twist::SharedPtr m) { on_cmd(m); });");
    lines.push("    pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(\"wheel_cmds\", 10);");
    lines.push("  }");
    lines.push("private:");
    lines.push("  const double r_ = " + numLit(r) + ", Lx_ = " + numLit(Lx) + ", Ly_ = " + numLit(Ly) + ";");
    lines.push("  const int MOTOR_WHEEL_[" + count + "] = " + arrLiteral(motorWheelArray(motorType, config)) + ";");
    lines.push("  const int POLARITY_[" + count + "] = " + arrLiteral(polarityArray(config)) + ";");
    lines.push("");
    lines.push("  void on_cmd(const geometry_msgs::msg::Twist::SharedPtr m) {");
    lines.push("    double vx = m->linear.x, vy = m->linear.y, w = m->angular.z;");
    lines.push("    double vw[" + count + "];");
    wheelFormulaLinesCpp(motorType, Lx, Ly).forEach((l) => lines.push(l));
    lines.push("    std_msgs::msg::Float32MultiArray out;");
    lines.push("    for (int i = 0; i < " + count + "; i++) out.data.push_back(vw[MOTOR_WHEEL_[i]] * POLARITY_[i]);");
    lines.push("    pub_->publish(out);");
    lines.push("  }");
    lines.push("");
    lines.push("  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;");
    lines.push("  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;");
    lines.push("};");
    lines.push("");
    lines.push("int main(int argc, char **argv) {");
    lines.push("  rclcpp::init(argc, argv);");
    lines.push("  rclcpp::spin(std::make_shared<WheelDriver>());");
    lines.push("  rclcpp::shutdown();");
    lines.push("  return 0;");
    lines.push("}");
    return lines.join("\n");
  }

  function showCode(text) {
    lastCode = text;
    $("code-output").textContent = text;
  }

  async function copyCode() {
    if (!lastCode) return;
    try {
      await navigator.clipboard.writeText(lastCode);
      const b = $("btn-copy");
      b.textContent = "✅ コピーしました";
      setTimeout(() => { b.textContent = "📋 コピー"; }, 1500);
    } catch (e) {
      // fallback
      const ta = document.createElement("textarea");
      ta.value = lastCode;
      document.body.appendChild(ta);
      ta.select();
      document.execCommand("copy");
      document.body.removeChild(ta);
    }
  }

  // ---------- presets ----------

  function applyKinPreset(name) {
    const sets = {
      forward: { vx: 1, vy: 0, w: 0 },
      strafe: { vx: 0, vy: 1, w: 0 },
      rotate: { vx: 0, vy: 0, w: 1 }
    };
    const s = sets[name];
    $("kin-vx").value = s.vx;
    $("kin-vy").value = s.vy;
    $("kin-w").value = s.w;
    updateKin();
  }

  // ---------- wiring ----------

  function bindCard(ids, fn) {
    ids.forEach((id) => { on(id, "input", fn); on(id, "change", fn); });
  }

  function init() {
    buildMatrixInputs();

    bindCard(["odom-r", "odom-l", "odom-nl", "odom-nr", "odom-tpr"], updateOdom);

    bindCard(["q-x", "q-y", "q-z", "q-w"], quatToEulerMatrix);
    bindCard(["e-roll", "e-pitch", "e-yaw"], eulerToQuatMatrix);
    bindCard(MATRIX_IDS, matrixToQuatEuler);
    on("btn-q2e", "click", quatToEulerMatrix);
    on("btn-q-norm", "click", normalizeQuatInputs);
    on("btn-e2q", "click", eulerToQuatMatrix);
    on("btn-m2qe", "click", matrixToQuatEuler);

    bindCard(["wh-motor", "wh-d", "wh-ratio"], updateWheel);
    bindCard(["wh-speed"], updateWheelReverse);

    bindCard(["can-id-hex", "can-id-dec"], updateCanId);
    bindCard(["can-data", "can-scale", "can-offset"], updateCanData);
    on("can-order", "change", updateCanData);
    on("can-bits", "change", updateCanData);
    on("can-signed", "change", updateCanData);

    bindCard(["batt-ah", "batt-a"], updateBattery);
    bindCard(["batt-mah", "batt-ma"], updateBatteryMah);

    // kinematics card
    bindCard(["kin-vx", "kin-vy", "kin-w", "kin-r", "kin-lx", "kin-ly"], updateKin);
    document.querySelectorAll("#kin-tabs .tab").forEach((t) => {
      t.addEventListener("click", () => {
        document.querySelectorAll("#kin-tabs .tab").forEach((x) => x.classList.remove("active"));
        t.classList.add("active");
        kinType = t.dataset.kin;
        updateKin();
      });
    });
    document.querySelectorAll("#kin-unit .chip").forEach((c) => {
      c.addEventListener("click", () => {
        document.querySelectorAll("#kin-unit .chip").forEach((x) => x.classList.remove("active"));
        c.classList.add("active");
        kinUnit = c.dataset.unit;
        updateKin();
      });
    });
    document.querySelectorAll("#card-kinematics .preset-row .chip").forEach((c) => {
      c.addEventListener("click", () => applyKinPreset(c.dataset.preset));
    });

    // motor card
    bindCard(["mot-vx", "mot-vy", "mot-w", "mot-r", "mot-lx", "mot-ly", "mot-vmax"], updateMotor);
    document.querySelectorAll("#motor-tabs .tab").forEach((t) => {
      t.addEventListener("click", () => {
        document.querySelectorAll("#motor-tabs .tab").forEach((x) => x.classList.remove("active"));
        t.classList.add("active");
        motorType = t.dataset.type;
        buildMotorConfig();
        updateMotor();
      });
    });
    $("motor-config").addEventListener("change", updateMotor);
    $("motor-config").addEventListener("input", updateMotor);
    on("btn-gen-arduino", "click", () => showCode(genArduino()));
    on("btn-gen-ros", "click", () => showCode(genRos()));
    on("btn-copy", "click", copyCode);

    buildMotorConfig();

    // initial render
    updateOdom();
    quatToEulerMatrix();
    updateWheel();
    updateCanId();
    updateCanData();
    updateBattery();
    updateBatteryMah();
    updateKin();
    updateMotor();

    registerSW();
    installPrompt();
  }

  // ---------- PWA ----------

  function registerSW() {
    const status = $("sw-status");
    if (!("serviceWorker" in navigator)) { status.textContent = "サービスワーカー: 非対応ブラウザ"; return; }
    navigator.serviceWorker.register("sw.js")
      .then(() => { status.textContent = "サービスワーカー: 登録済み（オフライン可）"; })
      .catch((err) => { status.textContent = "サービスワーカー: 登録失敗（" + err.message + "）"; });
  }

  let deferredPrompt = null;
  function installPrompt() {
    window.addEventListener("beforeinstallprompt", (e) => {
      e.preventDefault();
      deferredPrompt = e;
      const hint = $("installHint");
      hint.classList.remove("hidden");
      hint.addEventListener("click", async () => {
        if (!deferredPrompt) return;
        deferredPrompt.prompt();
        await deferredPrompt.userChoice;
        deferredPrompt = null;
        hint.classList.add("hidden");
      });
    });
    window.addEventListener("appinstalled", () => { $("installHint").classList.add("hidden"); });
  }

  if (document.readyState === "loading") document.addEventListener("DOMContentLoaded", init);
  else init();
})();
