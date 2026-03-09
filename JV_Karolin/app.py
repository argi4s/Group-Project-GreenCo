from flask import Flask, jsonify, render_template, request
from mavlink_monitor import MavlinkMonitor
from lock_controller import LockController

app = Flask(__name__)

mav = MavlinkMonitor("/dev/ttyAMA0", 57600)
mav.start()

lockbox = LockController(mav)

ACCESS_TOKEN = "change-this-token"

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/api/status")
def status():
    return jsonify(lockbox.snapshot())

@app.route("/api/unlock", methods=["POST"])
def unlock():
    token = request.headers.get("X-Access-Token", "")
    if token != ACCESS_TOKEN:
        return jsonify({"ok": False, "message": "Unauthorized"}), 403

    ok, msg = lockbox.request_unlock()
    return jsonify({"ok": ok, "message": msg})

@app.route("/api/relock", methods=["POST"])
def relock():
    token = request.headers.get("X-Access-Token", "")
    if token != ACCESS_TOKEN:
        return jsonify({"ok": False, "message": "Unauthorized"}), 403

    ok, msg = lockbox.relock()
    return jsonify({"ok": ok, "message": msg})

@app.route("/healthz")
def healthz():
    return jsonify({"ok": True})

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8080, debug=False)