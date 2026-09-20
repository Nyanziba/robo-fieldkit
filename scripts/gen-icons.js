// Generate RoboFieldKit PNG icons (no external deps, uses zlib).
"use strict";
const zlib = require("zlib");
const fs = require("fs");
const path = require("path");

// ---- PNG encoding ----
const CRC_TABLE = (() => {
  const t = new Uint32Array(256);
  for (let n = 0; n < 256; n++) {
    let c = n;
    for (let k = 0; k < 8; k++) c = c & 1 ? 0xedb88320 ^ (c >>> 1) : c >>> 1;
    t[n] = c >>> 0;
  }
  return t;
})();

function crc32(buf) {
  let c = 0xffffffff;
  for (let i = 0; i < buf.length; i++) c = CRC_TABLE[(c ^ buf[i]) & 0xff] ^ (c >>> 8);
  return (c ^ 0xffffffff) >>> 0;
}

function chunk(type, data) {
  const len = Buffer.alloc(4);
  len.writeUInt32BE(data.length, 0);
  const typeBuf = Buffer.from(type, "ascii");
  const crc = Buffer.alloc(4);
  crc.writeUInt32BE(crc32(Buffer.concat([typeBuf, data])), 0);
  return Buffer.concat([len, typeBuf, data, crc]);
}

function encodePNG(width, height, rgba) {
  const sig = Buffer.from([137, 80, 78, 71, 13, 10, 26, 10]);
  const ihdr = Buffer.alloc(13);
  ihdr.writeUInt32BE(width, 0);
  ihdr.writeUInt32BE(height, 4);
  ihdr[8] = 8;  // bit depth
  ihdr[9] = 6;  // color type RGBA
  ihdr[10] = 0; // compression
  ihdr[11] = 0; // filter
  ihdr[12] = 0; // interlace

  const stride = width * 4;
  const raw = Buffer.alloc((stride + 1) * height);
  for (let y = 0; y < height; y++) {
    raw[y * (stride + 1)] = 0; // filter: none
    rgba.copy(raw, y * (stride + 1) + 1, y * stride, y * stride + stride);
  }
  const idat = zlib.deflateSync(raw, { level: 9 });
  return Buffer.concat([sig, chunk("IHDR", ihdr), chunk("IDAT", idat), chunk("IEND", Buffer.alloc(0))]);
}

// ---- drawing ----
const BG = [0x0b, 0x11, 0x20];
const CYAN = [0x38, 0xbd, 0xf8];
const ORANGE = [0xf5, 0x9e, 0x0b];
const WHITE = [0xe6, 0xed, 0xf7];

function drawIcon(size) {
  const rgba = Buffer.alloc(size * size * 4);
  const c = size / 2;
  const ringOuter = size * 0.29;
  const ringInner = size * 0.195;
  const toothOuter = size * 0.345;
  const teeth = 8;

  const set = (x, y, col) => {
    const i = (y * size + x) * 4;
    rgba[i] = col[0]; rgba[i + 1] = col[1]; rgba[i + 2] = col[2]; rgba[i + 3] = 255;
  };

  for (let y = 0; y < size; y++) {
    for (let x = 0; x < size; x++) {
      const dx = x - c, dy = y - c;
      const r = Math.sqrt(dx * dx + dy * dy);
      let color = BG;

      if (r <= ringOuter && r >= ringInner) {
        color = CYAN;
      }
      // gear teeth
      if (r > ringOuter && r <= toothOuter) {
        const ang = Math.atan2(dy, dx);
        const step = (Math.PI * 2) / teeth;
        let da = Math.abs(ang % step);
        da = Math.min(da, step - da);
        if (da < step * 0.28) color = CYAN;
      }
      // eyes (inside the ring hole)
      const eyeR = size * 0.042;
      const eyeOff = size * 0.07;
      const e1x = c - eyeOff, e2x = c + eyeOff, ey = c;
      const d1 = Math.hypot(x - e1x, y - ey);
      const d2 = Math.hypot(x - e2x, y - ey);
      if (d1 <= eyeR || d2 <= eyeR) color = WHITE;

      set(x, y, color);
    }
  }
  return encodePNG(size, size, rgba);
}

const outDir = path.join(__dirname, "..", "icons");
fs.mkdirSync(outDir, { recursive: true });
for (const size of [192, 512]) {
  const png = drawIcon(size);
  fs.writeFileSync(path.join(outDir, `icon-${size}.png`), png);
  console.log(`wrote icon-${size}.png (${png.length} bytes)`);
}
