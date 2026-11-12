# Secure Wi-Fi Sensing via CSI Fuzzing

A multi-node ESP32-S3 Wi-Fi CSI sensing platform (calibration-light device-free
localization) with a CSI-fuzzing defense (the Guardian role) against passive
CSI eavesdropping, plus a live Flask-SocketIO dashboard.

## Setup

1. **Python 3.9+** is required for the controller.
2. Install the controller's dependencies:

   ```bash
   pip install flask flask-socketio
   ```

   (`pyserial` is also useful if you're working with the hardware bring-up
   scripts, but isn't required to run the controller itself.)

3. Flash the firmware to your ESP32-S3 boards with [PlatformIO](https://platformio.org/),
   using the environments defined in `platformio.ini` (adjust `upload_port` /
   `monitor_port` per board to match your machine's COM ports).

## Starting the Controller

The controller listens for CSI records over UDP from the Receiver and serves
the live dashboard over HTTP/WebSocket.

```bash
cd controller
python app.py
```

By default this starts:

- the UDP listener on `0.0.0.0:1201` (matches `CONTROLLER_PORT` in firmware), and
- the dashboard at `http://localhost:5000`

Open `http://localhost:5000` in a browser once it's running to view live
motion/presence status, room localization, and the CSI-fuzzing evaluation
panel.

### Optional flags

```bash
python app.py --host 0.0.0.0 --port 1201 --web-host 0.0.0.0 --web-port 5000
```

| Flag | Default | Description |
|---|---|---|
| `--host` | `0.0.0.0` | Interface to bind the UDP listener to |
| `--port` | `1201` | UDP port for incoming CSI records |
| `--web-host` | `0.0.0.0` | Interface to bind the web dashboard to |
| `--web-port` | `5000` | Port for the web dashboard |
