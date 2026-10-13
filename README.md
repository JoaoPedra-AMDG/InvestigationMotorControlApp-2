# ODrive motor test workbench

This local site runs the bundled `AllTests.py` workflow. **The load motor controls velocity and the test motor applies opposing torque**, matching the working script. The script owns USB connection, board configuration, monitoring, captures and stopping. The browser collects test choices, shows script output and operator prompts, and opens the saved results.

The GitHub repository is the download link, not the running site. Open the styled workbench through the local address below after starting `start.ps1`. Opening `script_site.html` directly cannot connect to the motor controller.

## Run on the computer connected to the ODrives

1. Download or clone this repository to a local drive. Install 64-bit Python 3.10 or newer. Close the ODrive GUI and other programs using the boards.
2. Check the two serial numbers and rig settings near the top of `AllTests.py`. The site displays the serials from that file. Its other settings can also be edited in the script, just as in the standalone workflow.
3. Open PowerShell in this folder and run `./start.ps1`. The launcher creates `.venv`, installs `numpy` and the pinned ODrive package, and starts the local server.
4. Open `http://127.0.0.1:8765`. Starting the server does not connect or start either motor. Choose a point or matrix to start the script. Read the script's output and answer its prompts in **Connect and status**.

Manual setup, if needed:

```powershell
py -3 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe script_site.py
```

## Sections

- **Connect and status:** board serials, script output, live run state, operator prompts, and side-by-side test/load position, speed and phase-current trends from the script's `monitor.jsonl`. The same live trends appear on Manual control and Test matrix. Each script workflow opens the boards itself and releases them on completion.
- **Manual control:** one sensored or sensorless measured point using the script's speed ramp, current sign probe, capture and stop. This is a point workflow, rather than arbitrary direct setpoint control.
- **Motor tuning:** explains the script-managed velocity gains, ramps, limits and per-point feedback bandwidth. Independent automated PID tuning is not part of the working script and is not exposed as an active control.
- **Test matrix:** runs the script's matrix workflow or full six-stage sequence. An error stops the matrix, as in the script.
- **Results:** browses original CSV, JSON and SVG files in `results/`. CSV previews plot position, speed and phase currents; files are available to download.

The red **Stop both motors** button writes the script's existing `site-estop.request` file. Its active monitor requests IDLE on both boards and the normal shutdown path verifies the stop. During a pre-motion prompt the page also answers “no”. The script's configured current, speed, voltage, temperature, fault, watchdog and coupling checks remain in force. A physical stop must still be accessible, as required by the script.

This software has been checked without the physical ODrives. USB operation, firmware behaviour, capture timing and stopping must be verified on the rig before relying on the site for experiments.
