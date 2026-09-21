/* RoboFieldKit verification harness (Node, no external deps).
 * 1) Unit-test the pure math in js/kinematics.js against known values.
 * 2) Smoke-test js/app.js against a minimal fake DOM (proves no JS errors + renders).
 */
"use strict";
const assert = require("assert");
const path = require("path");

const RFK = require(path.join(__dirname, "..", "js", "kinematics.js"));

let passed = 0;
function ok(name, cond) {
  assert.ok(cond, name);
  passed++;
  console.log("  ✓ " + name);
}
function approx(a, b, eps) {
  eps = eps || 1e-6;
  return Math.abs(a - b) <= eps * Math.max(1, Math.abs(b));
}

console.log("\n=== 1) 数学テスト (kinematics.js) ===");

// ---- odometry ----
{
  const o = RFK.odometry(0.05, 0.3, 100, 100, 100);
  ok("odometry 直進距離 d = 2πr·(counts/tpr)", approx(o.d, 2 * Math.PI * 0.05 * 1, 1e-9));
  ok("odometry 等カウント → Δθ=0", approx(o.dtheta, 0, 1e-12));
  ok("odometry 等カウント → x=d, y=0", approx(o.x, o.d, 1e-12) && approx(o.y, 0, 1e-12));
  const o2 = RFK.odometry(0.05, 0.3, 0, 100, 100); // right only → rotate
  ok("odometry 右輪のみ → 左進む(旋回)", o2.dl === 0 && o2.dr > 0 && o2.dtheta > 0);
}

// ---- quaternion / euler / matrix ----
{
  const q0 = { x: 0, y: 0, z: 0, w: 1 };
  const m = RFK.quatToMatrix(q0);
  ok("単位クォータニオン → 単位行列", approx(m[0][0], 1) && approx(m[1][1], 1) && approx(m[2][2], 1) && approx(m[0][1], 0));

  const e = RFK.matrixToEuler(m);
  ok("(0,0,0,1) → オイラー (0,0,0)", approx(e.roll, 0, 1e-12) && approx(e.pitch, 0, 1e-12) && approx(e.yaw, 0, 1e-12));

  // yaw 90° → quat (0,0,sin45,cos45)
  const q90 = RFK.eulerToQuat(0, 0, RFK.rad(90));
  ok("オイラー(0,0,90°) → クォータニオン", approx(q90.x, 0, 1e-9) && approx(q90.y, 0, 1e-9) && approx(q90.z, Math.SQRT1_2, 1e-9) && approx(q90.w, Math.SQRT1_2, 1e-9));

  // round-trip euler -> matrix -> euler
  const r = RFK.rad(10), p = RFK.rad(20), y = RFK.rad(30);
  const eRt = RFK.matrixToEuler(RFK.eulerToMatrix(r, p, y));
  ok("euler→matrix→euler 往復 (10,20,30°)",
    approx(eRt.roll, r, 1e-9) && approx(eRt.pitch, p, 1e-9) && approx(eRt.yaw, y, 1e-9));

  // round-trip euler -> quat -> euler
  const q = RFK.eulerToQuat(r, p, y);
  const eQ = RFK.matrixToEuler(RFK.quatToMatrix(q));
  ok("euler→quat→matrix→euler 往復",
    approx(eQ.roll, r, 1e-9) && approx(eQ.pitch, p, 1e-9) && approx(eQ.yaw, y, 1e-9));

  // quat -> matrix -> quat round-trip (non-trivial, normalized)
  const qn = RFK.normalizeQuat({ x: 1, y: 2, z: 3, w: 4 });
  const qrt = RFK.matrixToQuat(RFK.quatToMatrix(qn));
  ok("quat→matrix→quat 往復 (正規化後)",
    approx(qrt.x, qn.x, 1e-9) && approx(qrt.y, qn.y, 1e-9) && approx(qrt.z, qn.z, 1e-9) && approx(qrt.w, qn.w, 1e-9));
}

// ---- wheel speed / gear ratio ----
{
  const w = RFK.wheelFromMotor(1000, 0.10, 10);
  ok("車輪回転数 = motor/ratio", approx(w.wheelRpm, 100, 1e-9));
  ok("車輪円周 = πD", approx(w.circ, Math.PI * 0.10, 1e-12));
  ok("直進速度 = circ·rpm/60", approx(w.speed, Math.PI * 0.10 * 100 / 60, 1e-9));
  const rev = RFK.motorFromSpeed(1.0, 0.10, 10);
  ok("逆算: 1 m/s → モーターRPM", approx(rev.wheelRpm, 60 / (Math.PI * 0.10), 1e-9));
}

// ---- CAN ----
{
  ok("parseHexId '0x123' = 291", RFK.parseHexId("0x123") === 291);
  ok("parseHexId '123h' = 291", RFK.parseHexId("123h") === 291);
  const d = RFK.decodeCan([0x01, 0x02], "le", 16, false);
  ok("LE 16 unsigned '01 02' = 513", Number(d.raw) === 513);
  const ds = RFK.decodeCan([0xFF, 0xFF], "le", 16, true);
  ok("LE 16 signed 'FF FF' = -1", ds.rawNum === -1);
  const db = RFK.decodeCan([0x00, 0x01], "be", 16, false);
  ok("BE 16 unsigned '00 01' = 1", Number(db.raw) === 1);
  ok("物理値 = raw·scale + offset", approx(RFK.physical(513, 0.1, 5), 56.3, 1e-9));
  ok("parseHexBytes 奇数桁 → null", RFK.parseHexBytes("123") === null);
}

// ---- battery ----
{
  const t = RFK.runtimeHours(5.0, 1.0);
  ok("5 Ah / 1 A = 5 時間", approx(t.hours, 5, 1e-12) && approx(t.minutes, 300, 1e-9));
}

// ---- mecanum inverse kinematics ----
{
  const Lx = 0.10, Ly = 0.15;
  let s = RFK.mecanumWheels(1, 0, 0, Lx, Ly);
  ok("メカナム前進: 全輪 +1", ["前左", "前右", "後左", "後右"].every((p) => approx(s[p], 1, 1e-9)));

  s = RFK.mecanumWheels(0, 1, 0, Lx, Ly);
  ok("メカナム横移動: 対角チェッカー", approx(s["前左"], -1, 1e-9) && approx(s["前右"], 1, 1e-9) && approx(s["後左"], 1, 1e-9) && approx(s["後右"], -1, 1e-9));

  s = RFK.mecanumWheels(0, 0, 1, Lx, Ly);
  const ssum = Lx + Ly;
  ok("メカナム旋回: 左ペア− / 右ペア＋",
    approx(s["前左"], -ssum, 1e-9) && approx(s["後左"], -ssum, 1e-9) && approx(s["前右"], ssum, 1e-9) && approx(s["後右"], ssum, 1e-9));
}

// ---- omni inverse kinematics ----
{
  const Lx = 0.10, Ly = 0.15, L = Math.hypot(Lx, Ly);
  let s = RFK.omniWheels(0, 0, 1, Lx, Ly);
  ok("オムニ旋回: 全輪 +ωL", ["前左", "前右", "後左", "後右"].every((p) => approx(s[p], L, 1e-9)));

  s = RFK.omniWheels(1, 0, 0, Lx, Ly);
  const c = Lx / L, ss = Ly / L;
  ok("オムニ前進: 左ペア− / 右ペア＋",
    approx(s["前左"], -ss, 1e-9) && approx(s["後左"], -ss, 1e-9) && approx(s["前右"], ss, 1e-9) && approx(s["後右"], ss, 1e-9));
}

// ---- diff inverse kinematics ----
{
  const Ly = 0.15;
  let s = RFK.diffWheels(1, 0, Ly);
  ok("差動前進: 左右 +1", approx(s["左"], 1, 1e-9) && approx(s["右"], 1, 1e-9));
  s = RFK.diffWheels(0, 1, Ly);
  ok("差動旋回: 左− / 右＋", approx(s["左"], -Ly, 1e-9) && approx(s["右"], Ly, 1e-9));
}

// ---- unit helpers ----
ok("linearToAngular", approx(RFK.linearToAngular(1, 0.05), 20, 1e-9));
ok("angularToRpm", approx(RFK.angularToRpm(20), 20 * 60 / (2 * Math.PI), 1e-9));

console.log("\n  数学テスト: " + passed + " 件 パス");

// ============================================================
console.log("\n=== 2) DOM スモークテスト (app.js, JSエラーなし・描画確認) ===");

class FakeElement {
  constructor(id) {
    this.id = id;
    this._value = "";
    this.innerHTML = "";
    this.textContent = "";
    this.dataset = {};
    this.checked = false;
    this.classList = {
      _s: new Set(),
      add(c) { this._s.add(c); },
      remove(c) { this._s.delete(c); },
      contains(c) { return this._s.has(c); }
    };
    this._l = {};
  }
  get value() { return this._value; }
  set value(v) { this._value = v; }
  addEventListener(type, fn) { (this._l[type] = this._l[type] || []).push(fn); }
  click() { (this._l.click || []).forEach((f) => f({ preventDefault() {} })); }
  querySelector() { return null; }
  querySelectorAll() { return []; }
  select() {}
  appendChild() {}
  remove() {}
}

class FakeDocument {
  constructor() {
    this.readyState = "complete";
    this.activeElement = null;
    this.body = new FakeElement("body");
    this.els = {};
  }
  getElementById(id) {
    if (!this.els[id]) this.els[id] = new FakeElement(id);
    return this.els[id];
  }
  querySelectorAll() { return []; }
  createElement(tag) { return new FakeElement(tag); }
  addEventListener() {}
}

const doc = new FakeDocument();
const defaults = {
  "odom-r": "0.05", "odom-l": "0.30", "odom-nl": "100", "odom-nr": "100", "odom-tpr": "100",
  "q-x": "0", "q-y": "0", "q-z": "0", "q-w": "1",
  "e-roll": "0", "e-pitch": "0", "e-yaw": "0",
  "m00": "1", "m01": "0", "m02": "0", "m10": "0", "m11": "1", "m12": "0", "m20": "0", "m21": "0", "m22": "1",
  "wh-motor": "1000", "wh-d": "0.10", "wh-ratio": "10", "wh-speed": "1.0",
  "can-id-hex": "0x123", "can-id-dec": "291", "can-data": "01 02", "can-order": "le", "can-bits": "16", "can-signed": "unsigned", "can-scale": "1", "can-offset": "0",
  "batt-ah": "5.0", "batt-a": "1.0", "batt-mah": "5000", "batt-ma": "500",
  "kin-vx": "1", "kin-vy": "0", "kin-w": "0", "kin-r": "0.05", "kin-lx": "0.10", "kin-ly": "0.15",
  "mot-vx": "1", "mot-vy": "0", "mot-w": "0", "mot-r": "0.05", "mot-lx": "0.10", "mot-ly": "0.15", "mot-vmax": "1.5"
};
for (const [id, val] of Object.entries(defaults)) doc.getElementById(id).value = val;

// Provide 4 fake motor rows; M2 (前右) を「反転」にして極性切替を検証
function motorRow(pos, inv) {
  const r = new FakeElement("motor-row");
  const posSel = new FakeElement("motor-pos"); posSel.value = pos;
  const invChk = new FakeElement("motor-inv"); invChk.checked = inv;
  r.querySelector = (sel) => (sel === ".motor-pos" ? posSel : sel === ".motor-inv" ? invChk : null);
  return r;
}
const motorConfigEl = doc.getElementById("motor-config");
motorConfigEl.querySelectorAll = () => [
  motorRow("前左", false), motorRow("前右", true), motorRow("後左", false), motorRow("後右", false)
];

global.window = { RFK, addEventListener() {} };
global.document = doc;

let threw = null;
try {
  require(path.join(__dirname, "..", "js", "app.js"));
} catch (e) {
  threw = e;
}

if (threw) {
  console.error("  ✗ app.js 実行中に例外:", threw);
  process.exit(1);
}
console.log("  ✓ app.js が例外なく初期化（init 完了）");

function html(id) { return doc.getElementById(id).innerHTML; }
function count(str, sub) { return str.split(sub).length - 1; }

const odomHtml = html("odom-results");
console.log("  ✓ オドメトリ描画: " + (odomHtml.includes("0.3142") ? "d=0.3142 正しい" : "FAIL: " + odomHtml));
assert.ok(odomHtml.includes("0.3142"), "odometry straight-line distance");

const quatHtml = html("quat-results");
console.log("  ✓ クォータニオン描画: " + (quatHtml.includes("0.000° / 0.000° / 0.000°") ? "単位→オイラー(0,0,0) 正しい" : "FAIL"));
assert.ok(quatHtml.includes("0.000° / 0.000° / 0.000°"), "identity quaternion -> zero euler");

const kinHtml = html("kin-results");
console.log("  ✓ メカナム前進描画: " + (kinHtml.includes("1.000 m/s") ? "全輪 1.000 m/s 正しい" : "FAIL"));
assert.ok(kinHtml.includes("1.000 m/s"), "mecanum forward");

const motorHtml = html("motor-results");
assert.ok(motorHtml.includes("前左") && motorHtml.includes("前右"), "motor table renders");
const fwdCount = count(motorHtml, "正転");
const revCount = count(motorHtml, "逆転");
console.log("  ✓ モーター指令（M2 反転）: 正転×" + fwdCount + " / 逆転×" + revCount);
assert.ok(fwdCount === 3 && revCount === 1, "polarity flip changes direction (expect 3 正転 + 1 逆転)");

// CAN ID bidirectional smoke
const canIdHtml = html("can-id-results");
assert.ok(canIdHtml.includes("291"), "CAN ID dec 291 rendered");
console.log("  ✓ CAN ID 変換描画: 0x123 = 291 正しい");

const battHtml = html("batt-results");
assert.ok(battHtml.includes("5.00"), "battery 5h rendered");
console.log("  ✓ バッテリー描画: 5 Ah / 1 A = 5 時間 正しい");

// ---- code generation (core new feature) ----
doc.getElementById("btn-gen-arduino").click();
const ard = doc.getElementById("code-output").textContent;
assert.ok(ard.includes("MOTOR_WHEEL") && ard.includes("POLARITY") && ard.includes("void drive("), "arduino code generated");
assert.ok(ard.includes("{ 1, -1, 1, 1 }"), "arduino POLARITY reflects inverted M2");
assert.ok(ard.includes("メカナム"), "arduino code labels robot type");
assert.ok(ard.includes("vw[0] = vx - vy - w*"), "arduino mecanum formula present");
console.log("  ✓ Arduino/ESP32 コード生成: PWM・MOTOR_WHEEL・POLARITY{1,-1,1,1}・逆運動学式を含む");

doc.getElementById("btn-gen-ros").click();
const ros = doc.getElementById("code-output").textContent;
assert.ok(ros.includes("rclcpp") && ros.includes("cmd_vel") && ros.includes("geometry_msgs"), "ros code generated");
assert.ok(ros.includes("POLARITY_") && ros.includes("on_cmd"), "ros polarity + callback present");
console.log("  ✓ ROS2 C++ コード生成: rclcpp / cmd_vel / geometry_msgs / 極性配列を含む");

console.log("\n  DOM スモークテスト: パス（JSエラーなし・各計算器が正しい値を描画）");

console.log("\n=== 検証完了: 全てパス ===");
