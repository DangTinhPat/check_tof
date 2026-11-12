/*
 * VL53L8CX raw-data filter pipeline (runs in the browser; also loadable in Node for tests).
 *
 * The ESP32 sends every zone exactly as the sensor reports it (distance, target_status,
 * sigma). All decisions about which zones to trust happen here, step by step:
 *
 *   1. status   keep zones whose target_status is in the accepted set
 *   2. sigma    drop zones whose estimated uncertainty (sigma) is too large
 *   3. median   temporal median over the last 3 or 5 accepted values (kills single spikes)
 *   4. hold     when a zone drops out briefly, keep its last good value for a few frames
 *   5. smooth   exponential smoothing; a change larger than `jumpMm` passes through at once
 *               so moving objects are not delayed
 *   6. edge     (optional) drop "flying pixels": zones whose value sits between a near and a
 *               far neighbour with no neighbour close to it (typical at object edges)
 *
 * Every step can be switched off. The pipeline keeps a little per-zone state (history,
 * last good value, smoothed value); each frame costs a few microseconds.
 */
(function (root) {
  'use strict';

  /* target_status meanings, after ST UM3109 (VL53L8CX ULD user manual). */
  const STATUS_INFO = {
    0: 'Dữ liệu không được cập nhật',
    1: 'Tín hiệu trên mảng SPAD quá yếu',
    2: 'Lỗi pha mục tiêu',
    3: 'Sigma ước lượng quá cao',
    4: 'Mục tiêu không nhất quán',
    5: 'Hợp lệ',
    6: 'Chưa xử lý wrap-around (thường ở frame đầu)',
    7: 'Tốc độ tín hiệu không nhất quán',
    8: 'Tín hiệu quá yếu cho mục tiêu này',
    9: 'Hợp lệ, xung lớn (có thể là 2 vật gộp)',
    10: 'Hợp lệ, nhưng frame trước không có vật',
    11: 'Đo không nhất quán',
    12: 'Bị vật khác làm nhoè (do sharpener)',
    13: 'Có vật nhưng dữ liệu không nhất quán',
    255: 'Không phát hiện vật',
  };

  const DEFAULT_OPTIONS = {
    accept: [5, 9],      // 1. accepted target_status codes
    sigmaOn: true,       // 2.
    sigmaMax: 15,        //    mm
    median: 3,           // 3. 1 = off, 3 or 5 frames
    holdFrames: 3,       // 4. 0 = off
    smooth: 0.4,         // 5. 0 = off; weight of the previous value
    jumpMm: 100,         //    changes larger than this are not smoothed
    edgeOn: false,       // 6.
    edgeMm: 80,          //    gap to near/far neighbours that marks a flying pixel
  };

  function median(a) {
    const s = a.slice().sort((x, y) => x - y), m = s.length >> 1;
    return s.length % 2 ? s[m] : (s[m - 1] + s[m]) / 2;
  }

  function createPipeline(N) {
    N = N || 8;
    const NZ = N * N;
    let hist, last, age, ema;

    function reset() {
      hist = Array.from({ length: NZ }, () => []);
      last = new Float64Array(NZ).fill(NaN);
      age = new Uint32Array(NZ).fill(1e9);
      ema = new Float64Array(NZ).fill(NaN);
    }
    reset();

    /*
     * frame: { d: [NZ] distance mm, s: [NZ] status, g: [NZ] sigma mm }
     * returns { v: Float64Array (NaN = no value), why: [NZ] '' | 'status' | 'sigma' | 'nodata' | 'edge',
     *           held: Uint8Array (1 = showing an older value) }
     */
    function process(frame, options) {
      const o = Object.assign({}, DEFAULT_OPTIONS, options || {});
      const accept = new Set(o.accept);
      const medN = Math.max(1, o.median | 0);
      const v = new Float64Array(NZ).fill(NaN), why = new Array(NZ).fill(''), held = new Uint8Array(NZ);

      for (let i = 0; i < NZ; i++) {
        const x = frame.d[i], s = frame.s[i], g = frame.g ? frame.g[i] : 0;

        // 1-2: is this sample trustworthy at all?
        let ok = true;
        if (!accept.has(s)) { ok = false; why[i] = 'status'; }
        else if (o.sigmaOn && g > o.sigmaMax) { ok = false; why[i] = 'sigma'; }
        else if (!(x > 0)) { ok = false; why[i] = 'nodata'; }

        // 3: temporal median over the recent accepted samples
        const h = hist[i];
        h.push(ok ? x : NaN);
        while (h.length > medN) h.shift();
        let y = NaN;
        if (ok) {
          if (medN > 1) { const good = h.filter(Number.isFinite); y = median(good); }
          else y = x;
        }

        // 4: hold the last good value through short dropouts
        if (ok) { last[i] = y; age[i] = 0; }
        else {
          age[i]++;
          if (o.holdFrames > 0 && age[i] <= o.holdFrames && Number.isFinite(last[i])) { y = last[i]; held[i] = 1; }
        }

        // 5: smoothing that never lags behind a real movement
        if (Number.isFinite(y)) {
          if (o.smooth > 0 && Number.isFinite(ema[i]) && Math.abs(y - ema[i]) < o.jumpMm) {
            ema[i] = (1 - o.smooth) * y + o.smooth * ema[i];
          } else {
            ema[i] = y;
          }
          v[i] = ema[i];
        } else {
          ema[i] = NaN;
        }
      }

      // 6: flying pixels at object edges
      if (o.edgeOn) {
        const src = Float64Array.from(v);
        for (let r = 0; r < N; r++) for (let c = 0; c < N; c++) {
          const i = r * N + c, x = src[i];
          if (!Number.isFinite(x)) continue;
          let near = 0, far = 0, close = 0;
          for (const [dr, dc] of [[-1, 0], [1, 0], [0, -1], [0, 1]]) {
            const rr = r + dr, cc = c + dc;
            if (rr < 0 || cc < 0 || rr >= N || cc >= N) continue;
            const y = src[rr * N + cc];
            if (!Number.isFinite(y)) continue;
            if (y < x - o.edgeMm) near++;
            else if (y > x + o.edgeMm) far++;
            else close++;
          }
          if (near && far && !close) { v[i] = NaN; why[i] = 'edge'; held[i] = 0; }
        }
      }

      return { v, why, held };
    }

    return { process, reset };
  }

  const api = { STATUS_INFO, DEFAULT_OPTIONS, createPipeline, median };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else root.TofFilters = api;
})(typeof window !== 'undefined' ? window : this);
