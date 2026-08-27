#!/usr/bin/env python3
"""Receive PICO WebXR controller poses and print them; no ROS or robot access."""

import logging

from flask import Flask, jsonify, request, Response


app = Flask(__name__)


PAGE = r"""<!doctype html>
<html lang="zh-CN">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>PICO Controller Reader</title>
<style>
body { font-family: sans-serif; background:#111; color:#eee; padding:24px }
button { font-size:24px; padding:16px 28px }
pre { font-size:18px; white-space:pre-wrap }
</style>
<h2>PICO 4 Pro 手柄读取</h2>
<button id="enter">进入 VR 并发送数据</button>
<pre id="status">等待进入 VR</pre>
<canvas id="xr-canvas"></canvas>
<script>
const statusEl = document.getElementById('status');
const canvas = document.getElementById('xr-canvas');
let refSpace = null;
let lastSend = 0;
let gl = null;
let frameCount = 0;
let lastDiagnostic = 0;

function report(stage, detail = '') {
  fetch('/event', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({stage, detail})
  }).catch(() => {});
}

report('page_loaded', navigator.userAgent);

function controllerData(frame, source) {
  if (!source.gripSpace) return null;
  const pose = frame.getPose(source.gripSpace, refSpace);
  if (!pose) return null;
  const p = pose.transform.position;
  const q = pose.transform.orientation;
  const gp = source.gamepad;
  return {
    hand: source.handedness,
    position: [p.x, p.y, p.z],
    orientation: [q.x, q.y, q.z, q.w],
    trigger: gp?.buttons?.[0]?.value ?? 0,
    squeeze: gp?.buttons?.[1]?.value ?? 0
  };
}

async function onFrame(time, frame) {
  const session = frame.session;
  frameCount += 1;
  const layer = session.renderState.baseLayer;
  if (gl && layer) {
    gl.bindFramebuffer(gl.FRAMEBUFFER, layer.framebuffer);
    gl.clearColor(0.03, 0.03, 0.03, 1.0);
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
  }
  const controllers = [];
  for (const source of session.inputSources) {
    const data = controllerData(frame, source);
    if (data) controllers.push(data);
  }
  if (time - lastDiagnostic >= 1000) {
    lastDiagnostic = time;
    report('xr_frames', `frames=${frameCount}, inputSources=${session.inputSources.length}, poses=${controllers.length}`);
  }
  if (time - lastSend >= 50 && controllers.length) {
    lastSend = time;
    statusEl.textContent = JSON.stringify(controllers, null, 2);
    fetch('/data', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({timestamp_ms: Date.now(), controllers}),
      keepalive: true
    }).catch(() => {});
  }
  session.requestAnimationFrame(onFrame);
}

document.getElementById('enter').onclick = async () => {
  report('button_clicked');
  if (!navigator.xr) {
    statusEl.textContent = '当前浏览器没有 WebXR';
    report('error', 'navigator.xr unavailable');
    return;
  }
  try {
    gl = canvas.getContext('webgl', {xrCompatible: true});
    if (!gl) throw new Error('无法创建 WebGL 上下文');
    await gl.makeXRCompatible();
    const session = await navigator.xr.requestSession('immersive-vr', {
      requiredFeatures: ['local-floor']
    });
    report('session_started');
    session.updateRenderState({baseLayer: new XRWebGLLayer(session, gl)});
    refSpace = await session.requestReferenceSpace('local-floor');
    report('reference_space_ready');
    statusEl.textContent = 'VR 会话已启动，等待手柄数据';
    session.requestAnimationFrame(onFrame);
  } catch (e) {
    statusEl.textContent = '启动失败: ' + e;
    report('error', String(e));
  }
};
</script>
</html>
"""


@app.get("/")
def index() -> Response:
    response = Response(PAGE, mimetype="text/html")
    response.headers["Cache-Control"] = "no-store, no-cache, must-revalidate"
    return response


@app.post("/event")
def event():
    payload = request.get_json(silent=True) or {}
    print(
        f"[VR诊断] {payload.get('stage', '?')}: {payload.get('detail', '')}",
        flush=True,
    )
    return jsonify(ok=True)


@app.post("/data")
def data():
    payload = request.get_json(silent=True) or {}
    for controller in payload.get("controllers", []):
        p = controller.get("position", [0.0] * 3)
        q = controller.get("orientation", [0.0] * 4)
        print(
            f"{controller.get('hand', '?'):>5}  "
            f"xyz=({p[0]:+.4f}, {p[1]:+.4f}, {p[2]:+.4f})  "
            f"quat=({q[0]:+.4f}, {q[1]:+.4f}, {q[2]:+.4f}, {q[3]:+.4f})  "
            f"squeeze={controller.get('squeeze', 0.0):.3f}  "
            f"trigger={controller.get('trigger', 0.0):.3f}",
            flush=True,
        )
    return jsonify(ok=True)


if __name__ == "__main__":
    logging.getLogger("werkzeug").setLevel(logging.ERROR)
    print("VR reader: http://127.0.0.1:8000", flush=True)
    app.run(host="0.0.0.0", port=8000, threaded=True)
