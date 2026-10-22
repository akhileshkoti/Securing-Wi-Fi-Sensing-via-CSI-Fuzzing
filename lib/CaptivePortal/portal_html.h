#pragma once

// The setup form, kept out of the logic file for readability.
// {{MAC}} and {{DEVICE_ID}} are filled in by CaptivePortal::handleRoot().
const char PORTAL_FORM_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>ESP Setup</title>

    <style>
        body {
            font-family: sans-serif;
            max-width: 420px;
            margin: 24px auto;
            padding: 0 16px;
        }

        h2 {
            text-align: center;
        }

        label {
            display: block;
            margin: 12px 0 4px;
            font-weight: 600;
        }

        input,
        select {
            width: 100%;
            padding: 10px;
            box-sizing: border-box;
            border: 1px solid #ccc;
            border-radius: 6px;
        }

        button {
            width: 100%;
            margin-top: 18px;
            padding: 12px;
            border: 0;
            border-radius: 6px;
            background: #2563eb;
            color: #fff;
            font-size: 16px;
            cursor: pointer;
        }

        button:hover {
            opacity: 0.9;
        }

        .device {
            margin: 0 0 8px;
            padding: 12px;
            border: 1px solid #c7d2fe;
            border-radius: 6px;
            background: #eef2ff;
            text-align: center;
        }

        .device .mac {
            display: block;
            margin: 4px 0;
            font-family: monospace;
            font-size: 20px;
            font-weight: 700;
            letter-spacing: 1px;
            user-select: all;   /* one tap selects the whole MAC for copying */
        }

        .device small {
            color: #555;
        }
    </style>
</head>

<body>
    <h2>ESP Configuration</h2>

    <div class="device">
        <small>This board's MAC (device {{DEVICE_ID}})</small>
        <span class="mac">{{MAC}}</span>
        <small>If this board is an Illuminator, enter this MAC as its TX
        in the controller dashboard's Node positions.</small>
    </div>

    <form action="/save" method="POST">
        <label for="ssid">Wi-Fi SSID</label>
        <input id="ssid" name="ssid" required>

        <label for="pass">Wi-Fi Password</label>
        <input id="pass" name="pass" type="password">

        <label for="role">Role</label>
        <select name="role" id="role" onchange="toggleServer()">
            <option value="Illuminator">Illuminator</option>
            <option value="Receiver">Receiver</option>
            <option value="Guardian">Guardian</option>
        </select>

        <div id="serverRow">
            <label for="server">Server IP (controller)</label>
            <input
                name="server"
                id="server"
                placeholder="192.168.1.100"
            >
        </div>

        <button type="submit">
            Save &amp; Reboot
        </button>
    </form>

    <script>
        function toggleServer() {
            const isReceiver =
                document.getElementById('role').value === 'Receiver';

            document.getElementById('serverRow').style.display =
                isReceiver ? 'block' : 'none';

            document.getElementById('server').required =
                isReceiver;
        }

        toggleServer();
    </script>
</body>
</html>
)rawliteral";