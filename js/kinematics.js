/* RoboFieldKit — pure math / kinematics module (no DOM).
 * Loaded in the browser as window.RFK, and consumable from Node via module.exports.
 */
(function (root, factory) {
  const mod = factory();
  if (typeof module === "object" && module.exports) module.exports = mod;
  else root.RFK = mod;
})(typeof self !== "undefined" ? self : this, function () {
  "use strict";

  const DEG = Math.PI / 180;

  function deg(r) { return r / DEG; }
  function rad(d) { return d * DEG; }

  // ---------- differential-drive odometry ----------
  // r: wheel radius [m], L: track width [m], nl/nr: encoder counts, tpr: ticks/rev
  function odometry(r, L, nl, nr, tpr) {
    const dl = 2 * Math.PI * r * nl / tpr;
    const dr = 2 * Math.PI * r * nr / tpr;
    const d = (dl + dr) / 2;
    const dtheta = (dr - dl) / L;
    const x = d * Math.cos(dtheta / 2);
    const y = d * Math.sin(dtheta / 2);
    return { dl, dr, d, dtheta, x, y, theta: dtheta };
  }

  // ---------- quaternion / euler / matrix ----------
  // Quaternion uses Hamilton form q = w + xi + yj + zk, order {x,y,z,w}.
  // Euler uses ZYX intrinsic: R = Rz(yaw)*Ry(pitch)*Rx(roll).

  function normalizeQuat(q) {
    const n = Math.hypot(q.x, q.y, q.z, q.w);
    if (n === 0) return null;
    return { x: q.x / n, y: q.y / n, z: q.z / n, w: q.w / n };
  }

  function quatToMatrix(q) {
    const { x, y, z, w } = q;
    const xx = x * x, yy = y * y, zz = z * z;
    const xy = x * y, xz = x * z, yz = y * z;
    const wx = w * x, wy = w * y, wz = w * z;
    return [
      [1 - 2 * (yy + zz), 2 * (xy - wz),     2 * (xz + wy)],
      [2 * (xy + wz),     1 - 2 * (xx + zz), 2 * (yz - wx)],
      [2 * (xz - wy),     2 * (yz + wx),     1 - 2 * (xx + yy)]
    ];
  }

  function matrixToEuler(m) {
    const pitch = Math.asin(clamp(-m[2][0], -1, 1));
    const roll = Math.atan2(m[2][1], m[2][2]);
    const yaw = Math.atan2(m[1][0], m[0][0]);
    return { roll, pitch, yaw };
  }

  function mulMat(a, b) {
    const out = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
    for (let i = 0; i < 3; i++) {
      for (let j = 0; j < 3; j++) {
        let s = 0;
        for (let k = 0; k < 3; k++) s += a[i][k] * b[k][j];
        out[i][j] = s;
      }
    }
    return out;
  }

  function eulerToMatrix(roll, pitch, yaw) {
    const cr = Math.cos(roll), sr = Math.sin(roll);
    const cp = Math.cos(pitch), sp = Math.sin(pitch);
    const cy = Math.cos(yaw), sy = Math.sin(yaw);
    const rx = [[1, 0, 0], [0, cr, -sr], [0, sr, cr]];
    const ry = [[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]];
    const rz = [[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]];
    return mulMat(mulMat(rz, ry), rx);
  }

  function eulerToQuat(roll, pitch, yaw) {
    const cy = Math.cos(yaw / 2), sy = Math.sin(yaw / 2);
    const cp = Math.cos(pitch / 2), sp = Math.sin(pitch / 2);
    const cr = Math.cos(roll / 2), sr = Math.sin(roll / 2);
    return {
      w: cr * cp * cy + sr * sp * sy,
      x: sr * cp * cy - cr * sp * sy,
      y: cr * sp * cy + sr * cp * sy,
      z: cr * cp * sy - sr * sp * cy
    };
  }

  function matrixToQuat(m) {
    const m00 = m[0][0], m01 = m[0][1], m02 = m[0][2];
    const m10 = m[1][0], m11 = m[1][1], m12 = m[1][2];
    const m20 = m[2][0], m21 = m[2][1], m22 = m[2][2];
    const tr = m00 + m11 + m22;
    let qw, qx, qy, qz;
    if (tr > 0) {
      const S = Math.sqrt(tr + 1.0) * 2;
      qw = 0.25 * S; qx = (m21 - m12) / S; qy = (m02 - m20) / S; qz = (m10 - m01) / S;
    } else if (m00 > m11 && m00 > m22) {
      const S = Math.sqrt(1.0 + m00 - m11 - m22) * 2;
      qw = (m21 - m12) / S; qx = 0.25 * S; qy = (m01 + m10) / S; qz = (m02 + m20) / S;
    } else if (m11 > m22) {
      const S = Math.sqrt(1.0 + m11 - m00 - m22) * 2;
      qw = (m02 - m20) / S; qx = (m01 + m10) / S; qy = 0.25 * S; qz = (m12 + m21) / S;
    } else {
      const S = Math.sqrt(1.0 + m22 - m00 - m11) * 2;
      qw = (m10 - m01) / S; qx = (m02 + m20) / S; qy = (m12 + m21) / S; qz = 0.25 * S;
    }
    const n = Math.hypot(qx, qy, qz, qw);
    return { x: qx / n, y: qy / n, z: qz / n, w: qw / n };
  }

  // ---------- wheel speed / gear ratio ----------
  function wheelFromMotor(motorRpm, diameter, gearRatio) {
    const wheelRpm = motorRpm / gearRatio;
    const circ = Math.PI * diameter;
    const speed = circ * wheelRpm / 60; // m/s
    return { wheelRpm, circ, speed };
  }

  function motorFromSpeed(speed, diameter, gearRatio) {
    const circ = Math.PI * diameter;
    const wheelRpm = speed / circ * 60;
    const motorRpm = wheelRpm * gearRatio;
    return { wheelRpm, motorRpm };
  }

  // ---------- CAN ----------
  function parseHexId(str) {
    let s = (str || "").trim().toLowerCase();
    if (s === "") return null;
    if (s.startsWith("0x")) s = s.slice(2);
    if (s.endsWith("h")) s = s.slice(0, -1);
    if (!/^[0-9a-f]+$/.test(s)) return null;
    return parseInt(s, 16);
  }

  function parseHexBytes(str) {
    const cleaned = (str || "").replace(/0x/gi, "").replace(/[^0-9a-fA-F]/g, "");
    if (cleaned === "" || cleaned.length % 2 !== 0) return null;
    const bytes = [];
    for (let i = 0; i < cleaned.length; i += 2) bytes.push(parseInt(cleaned.slice(i, i + 2), 16));
    return bytes;
  }

  function decodeCan(bytes, order, bits, signed) {
    const nBytes = bits / 8;
    if (!bytes || bytes.length < nBytes) return null;
    const used = bytes.slice(0, nBytes);
    let raw = 0n;
    if (order === "le") {
      for (let i = 0; i < used.length; i++) raw |= BigInt(used[i]) << BigInt(8 * i);
    } else {
      for (let i = 0; i < used.length; i++) raw = (raw << 8n) | BigInt(used[i]);
    }
    const maxUnsigned = 1n << BigInt(bits);
    let rawSigned = raw;
    if (signed && raw >= (maxUnsigned >> 1n)) rawSigned = raw - maxUnsigned;
    const rawNum = signed ? Number(rawSigned) : Number(raw);
    return { raw: raw.toString(), rawSigned: rawSigned.toString(), rawNum, hex: raw.toString(16).toUpperCase().padStart(nBytes * 2, "0") };
  }

  function physical(rawNum, scale, offset) { return rawNum * scale + offset; }

  // ---------- battery ----------
  function runtimeHours(capacity, current) {
    const hours = capacity / current;
    return { hours, minutes: hours * 60 };
  }

  // ---------- wheel inverse kinematics ----------
  // Convention: vx = forward [m/s] (+x), vy = lateral/strafe [m/s] (+y = left),
  // w = yaw rate [rad/s] (+ = counter-clockwise). Returns LINEAR wheel speeds [m/s]
  // keyed by position, plus angular velocities via /r.
  // r: wheel radius [m], Lx: half wheelbase [m], Ly: half track width [m].

  function mecanumWheels(vx, vy, w, Lx, Ly) {
    const s = Lx + Ly;
    return {
      "前左": vx - vy - w * s,
      "前右": vx + vy + w * s,
      "後左": vx + vy - w * s,
      "後右": vx - vy + w * s
    };
  }

  function omniWheels(vx, vy, w, Lx, Ly) {
    const L = Math.hypot(Lx, Ly);
    const c = Lx / L, s = Ly / L;
    return {
      "前左": (vy * c - vx * s) + w * L,
      "前右": (vx * s + vy * c) + w * L,
      "後左": (-(vx * s + vy * c)) + w * L,
      "後右": (vx * s - vy * c) + w * L
    };
  }

  function diffWheels(vx, w, Ly) {
    // Ly = half track width (track / 2)
    return { "左": vx - w * Ly, "右": vx + w * Ly };
  }

  // Generic: returns linear wheel speeds [m/s] keyed by position label for a robot type.
  function wheelSpeeds(type, vx, vy, w, Lx, Ly) {
    if (type === "mecanum") return mecanumWheels(vx, vy, w, Lx, Ly);
    if (type === "omni") return omniWheels(vx, vy, w, Lx, Ly);
    if (type === "diff") return diffWheels(vx, w, Ly);
    return null;
  }

  function linearToAngular(v, r) { return v / r; }
  function angularToRpm(radps) { return radps * 60 / (2 * Math.PI); }

  function clamp(x, lo, hi) { return Math.max(lo, Math.min(hi, x)); }

  return {
    deg, rad, clamp,
    odometry,
    normalizeQuat, quatToMatrix, matrixToEuler, eulerToMatrix, eulerToQuat, matrixToQuat, mulMat,
    wheelFromMotor, motorFromSpeed,
    parseHexId, parseHexBytes, decodeCan, physical,
    runtimeHours,
    mecanumWheels, omniWheels, diffWheels, wheelSpeeds,
    linearToAngular, angularToRpm
  };
});
