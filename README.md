# ODrive two-motor workbench

A local Python website for two commissioned ODrive Pro boards: one test motor in speed control and one sensored load motor in opposing torque control. The application uses real board data only. When disconnected, values are unavailable and motor commands are blocked. Starting the application, opening a page, saving a plan or viewing a script does not connect or start a motor.

## Share and run this release

This repository is the distributable application, not a GitHub Pages motor controller. Each operator downloads the repository on the computer connected to their ODrives, starts the local Python server, then opens `http://127.0.0.1:8765`. A remote website cannot use that computer's USB devices through this Python backend. The operator must enter their own board serial numbers and validated rig limits; the example labels are not motor settings.

On Windows, use **Code → Download ZIP** on GitHub, extract the ZIP to a local drive, open PowerShell in the extracted folder and run `./start.ps1` (or `.\start.ps1` in Windows PowerShell). The launcher installs pinned Python dependencies in a local environment on first use. On Linux, use the setup instructions below. Do not copy a `.venv` from another computer.

**Results → Current-frequency spectrum** calculates and saves a one-sided Hann PSD for test and load three-phase onboard captures. It shows the share of resolved current variance (A²) in a band around the requested electrical frequency versus the rest, with a 0–500 Hz and full-band plot. The result is not electrical power in watts, and a short single capture has limited frequency resolution. Recalculation reads the original captured CSV; the run ZIP includes the saved spectrum JSON.

The **Motor tuning** page runs bounded, temporary velocity-PI gain comparisons on an already commissioned **sensored test motor**. It checks readiness and saves the starting gains, gives explicit physical-rig instructions, runs up to three short trials within the configured limits, records the response, and restores the starting gains before review. The operator can apply a recommended gain set temporarily, restore the original values, or explicitly save the applied values to the test board. The load-motor option records a current-tracking diagnostic without changing its current-loop gains. Automated sensorless commissioning/startup is not enabled in this release; the existing sensorless recording/import path remains available.

Software tests use offline fixtures. No claim of physical-rig validation, sensorless handover, USB timing, emergency-stop effectiveness or measurement bandwidth is made. Verify those on each installation before motor operation.

The hardware code is implemented but has not been validated on the physical rig. Keep the application disconnected while waiting for the USB isolators. You can prepare the connection profile, plan tests and inspect the Python source now.

## First-time setup on a lab computer

Use these instructions once on each Windows or Linux computer that may operate the rig. Extract or copy the complete repository folder to the computer's local drive first. Do not run it from OneDrive, a network share or a USB drive, and do not copy a `.venv` folder from another computer.

Before connecting either board:

1. Install 64-bit Python 3.10 or later.
2. Confirm that the motors are mechanically safe, the independent emergency stop is available and the correct USB isolator is fitted to each ODrive.
3. Close the ODrive GUI, `odrivetool` and any other program that could be using either board. Only one program can own an ODrive USB connection at a time.
4. Complete the operating-system setup below.

### Windows 10 or 11

1. Open PowerShell in the extracted repository folder.
2. Run:

   ```powershell
   .\start.ps1
   ```

   The launcher creates a new `.venv` for this computer, installs the required packages and starts the dashboard. If Python is installed but is not available from PowerShell, provide its full path:

   ```powershell
   .\start.ps1 -PythonPath 'C:\path\to\python.exe'
   ```

3. If Windows blocks local PowerShell scripts, use the manual commands under **Manual setup or troubleshooting** below.
4. When the launcher reports that it is ready, open [http://127.0.0.1:8765](http://127.0.0.1:8765) in a browser.

### Linux

The ODrive Python package requires a supported 64-bit Linux distribution, `libusb` and ODrive USB permission rules. These commands are written for Ubuntu or Debian-based lab computers.

1. Open a terminal in the extracted repository folder and install the required system packages:

   ```bash
   sudo apt update
   sudo apt install python3 python3-venv python3-pip libusb-1.0-0 curl
   ```

2. Install the official ODrive USB permission rules once on that computer:

   ```bash
   sudo bash -c "curl https://cdn.odriverobotics.com/files/odrive-udev-rules.rules > /etc/udev/rules.d/91-odrive.rules && udevadm control --reload-rules && udevadm trigger"
   ```

3. Unplug and reconnect both ODrives after installing the rules.
4. Create this computer's Python environment and install the application dependencies:

   ```bash
   python3 -m venv .venv
   .venv/bin/python -m pip install --upgrade pip
   .venv/bin/python -m pip install -r requirements.txt
   ```

5. Start the dashboard without `sudo`:

   ```bash
   .venv/bin/python app.py
   ```

6. Open [http://127.0.0.1:8765](http://127.0.0.1:8765) in a browser.

### First connection on either operating system

Starting the application does not connect to or start a motor. In the dashboard:

1. Open **Connections**.
2. Enter the two ODrive serial numbers and assign the physical test and load roles.
3. Enter the verified local speed, current and direction limits.
4. Complete the commissioning confirmations only after checking the physical rig.
5. Save the profile, click **Connect**, and resolve every readiness blocker before attempting motion.

Repeat these checks when moving the rig to a different computer. Never treat the dashboard's software Stop button as an emergency stop.

## Start the application after first-time setup

The ODrive Python dependency is pinned to `odrive==0.6.11.post1`; this is a Python package version, not a firmware recommendation.

On Windows, open PowerShell in the folder and run:

```powershell
.\start.ps1
```

On Linux, open a terminal in the folder and run:

```bash
.venv/bin/python app.py
```

Then open [http://127.0.0.1:8765](http://127.0.0.1:8765). The server binds only to that computer. Stop the application with Ctrl+C after ending the test. A second process cannot use the same port.

The default recording target is 20 Hz. On Windows, `start.ps1 -Rate 50` requests a different rate. On either operating system, `app.py --rate 50` requests a rate between 1 and 100 Hz. The board polling rate is a separate connection-profile setting. Neither setting guarantees that rate; inspect actual timestamps and achieved rate. Increasing the recorder rate cannot create new board samples.

## Manual setup or troubleshooting

If the Windows launcher cannot create the environment automatically, run:

```powershell
py -3 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe app.py
```

If the `py` launcher is unavailable, replace `py -3` with the full path to `python.exe`. Delete only the local `.venv` directory and recreate it if it was copied from another computer or refers to a Python installation that no longer exists. Do not delete the `recordings` directory when doing this.

## Pages

Use the horizontal menu bar to move between these views:

| Page | Purpose |
| --- | --- |
| Connect and status | Assign both serial numbers, save the validated profile, connect, inspect board states and readiness, and resolve blocked settings. |
| Manual control | Set test speed and opposing load, watch side-by-side position, speed and three-phase current trends, and record a selected point. |
| Motor tuning | Follow guided tasks and bounded velocity-PI trials on the sensored test motor, or measure load current tracking. |
| Test matrix | Select a tile and run one point or an included sequence; watch progress on the matrix. |
| Results | Review raw runs, plot and process data, compare frequency spectra and independent repeats, and export CSV or run ZIPs. |

Repeat comparisons and earlier analysis history remain available. Historical records retain their original acquisition-source tags; they are not relabelled as hardware experiments.

## Commissioning before the first connection

Commission both motors individually using the ODrive GUI, save their configurations, and release the GUI's USB connections before connecting this application. The application does not flash firmware, calibrate motors, change encoder routing. **Connect and status → Resolve blocked board settings** can explicitly update and optionally save the allowlisted watchdog, controller-mode, ramp and zero-setpoint settings.

The current adapter checks these requirements before allowing motion:

- Distinct hexadecimal serial numbers assigned to the physical test and load roles.
- Verified motor-shaft-turn units and direction conventions, including the opposing load direction.
- Positive, experimentally validated local maximum speed and load-current limits. The profile starts with zero limits so an unconfigured rig cannot start.
- Completed motor calibration and valid sensored commutation calibration where required.
- Firmware motor-current and velocity limits that are readable, positive and compatible with the selected point.
- Test drive already configured for `VELOCITY_CONTROL`, `VEL_RAMP` and a positive velocity ramp rate.
- Load drive already configured for `TORQUE_CONTROL`, `TORQUE_RAMP`, a positive torque ramp rate and a readable positive torque constant.
- Sensored feedback on the load drive. This version starts sensored sessions only; sensorless sessions are read-only monitoring/recording.
- An enabled onboard watchdog on each board with a timeout of at least max(0.5 seconds, six polling periods) and no more than 2 seconds. The application feeds it while it owns active motion.
- A verified stop procedure. This implementation supports only requesting `IDLE` to disable torque on both drives, allowing the rig to coast.
- Both boards initially idle, with readable zero active errors and disarm reasons.
- The same released 0.6.x firmware on both boards (0.6.10 or newer with inspected APIs).
- Initial velocity and torque setpoints (`axis0.config.init_vel` / `init_torque`) both zero; firmware restores these values during arming.

The application may reject additional unsupported configurations rather than guess signal mappings or motion behavior. Read the readiness details and hardware source for the exact checks. Role verification means inspecting the physical rig; checking a box does not validate wiring, units or operating limits.

Match the actual motor voltage/current limits, regeneration handling, mechanical coupling, guards and independent emergency-stop provision to your commissioned rig. The website's Stop control requests a software coast-to-IDLE stop. It is not an emergency stop and does not ensure that the shaft has stopped rotating.

## Connect and run one matrix point

1. On Connections, enter the two serial numbers, verified local limits, directions and required commissioning confirmations. Save the profile while disconnected.
2. Click Connect. Connection discovers only those serial numbers and reads their state/configuration; it does not enable motion. Confirm that the physical role assignment and displayed firmware/state are correct.
3. Resolve the readiness blockers. Fault clearing is a separate explicit action, and no automatic restart occurs.
4. On Test matrix, create or choose the desired speed/load/method/repeat point. Set recording duration and settling tolerances. Generating a matrix proposes points; it does not approve them as feasible for the rig.
5. Click Run selected test. This explicit action starts the configured sensored motor control, holds load torque at zero until the test motor reaches the load-application speed, waits for the point to settle, records for the specified duration, then requests the supported stop. If it does not settle within 60 seconds or a board/control/acquisition fault occurs, the workflow fails and requests a stop.
6. Inspect both Motors columns during the run. The browser display updates independently of the recorder.
7. Open Review and export, inspect timing and channel availability, record the outcome and download the complete run ZIP.

Manual condition application and recording are separate controls where provided. Ending a manual recording does not itself promise a mechanical stop: use Stop and inspect the rig. Closing the browser does not stop the Python application, active workflow or recording. Ending the Python application requests the supported stop; if communications fail, the application cannot guarantee delivery and reports the fault.

Sensorless Start is currently blocked because the adapter has no verified firmware marker separating the open-loop ramp from observer handover. Commissioning flags do not override that restriction. A separately started sensorless session may be connected for read-only monitoring and manual recording; the app does not feed the watchdog for motion it does not own. Sensorless tests require separately validated startup, handover, minimum speed and stopping behavior for the exact firmware and rig. Selecting a sensorless matrix point does not reconfigure a commissioned sensored drive. Follow the current readiness restriction; unsupported sensorless startup remains blocked rather than being approximated by a speed ramp.

## What the plots mean

Unavailable or unsupported channels remain blank/null and are not replaced with zeros. A disconnected board does not produce invented readings. Sign conventions are configured per motor; compare directions only after verifying the physical mapping.

- **DC voltage:** reported board bus voltage.
- **DC current:** the controller's reported/estimated DC-bus current, distinct from motor phase current and torque-producing Iq. Negative values can represent regeneration under the verified sign convention.
- **Speed and position:** speed in rpm; displayed position in revolutions relative to the first reading after connection, Run selected or Zero positions. Raw axis position is preserved in recordings. In sensorless operation these are not automatically independent encoder measurements.
- **Torque:** the configured torque constant multiplied by reported Iq, explicitly labelled as an estimate. It is not a torque-transducer measurement.

The profile can describe a separately verified encoder path and scale for comparison measurements. Record its provenance and calibration. Do not label a generic sensorless velocity estimate as an independent encoder reference.

## CSV, timing and high-rate acquisition

The live recorder saves new hardware samples to `recordings`, with actual host timestamps, per-board read timing, commands, channel provenance and run metadata. Missing values remain missing. A run bundle includes raw CSV files, events, metadata, source-file snapshots and retained review history. Earlier files are preserved when a test is repeated.

The two boards and their fields are read sequentially. Their host timestamps do not make the measurements simultaneous. Windows scheduling, USB latency and the number of available fields all affect the achieved acquisition rate. The dashboard refresh rate and chart history length do not set the CSV sampling bandwidth.

New steady-state plans require high-rate capture by default. Before motion, each board must expose the compatible oscilloscope API, a readable control-loop rate and all requested channels. Missing capability blocks the point before starting. Existing older plans retain their saved capture choice. The Connections page reports capability, rate and finite buffer size; enter the documented measurement bandwidth and filtering there before processing ripple. A motor current-control bandwidth is not automatically a measurement bandwidth.

At settling, `capture_runtime.py` uses the official synchronous ODrive capture helper on the existing connections. Two download threads keep the controller thread available for watchdog feeding and Stop. Each drive saves phase A/B/C current, DC voltage/current, speed and axis position. A sensorless test additionally saves the verified independent encoder angle and estimated electrical angle (up to nine channels). Raw angles remain available; relative mechanical revolutions are derived using the saved pole-pair count. Signals unavailable on the installed firmware block capture rather than becoming generated values.

Each board produces a **finite native-rate buffer**, not an unlimited high-rate stream. CSV files preserve the helper's trigger-relative control-cycle indices, device-reported sample rate, original axis positions and relative positions. Separate boards are **not synchronized**. The configured recording duration must cover capture and download. If duration expires while capture/download is pending, Stop/disconnect occurs, steady-state tolerances are lost, or the buffer is incomplete, the run is retained and marked invalid for ripple processing. Downloads time out after 25 seconds; an unfinished worker retains buffer ownership until it returns. Increase recording duration for a slow USB connection. Neither the reported control-loop rate nor a smooth graph establishes PWM switching-ripple bandwidth.

The feature must still be checked on the actual installed firmware and USB setup. Firmware lacking the required endpoints cannot capture these channels from this application. The app never installs prerelease firmware or weakens motion prerequisites to obtain capture support. External DAQ/oscilloscope imports remain available with explicit units, bandwidth, calibration and timing declarations. `capture_helpers.py` remains an optional standalone development helper; do not open another connection to a drive already in use.

## Running the matrix

Open **Configure tests** to generate paired speed/load points or edit the included subset. Select a tile, then **Run selected**, or press **Run all selected tests** once to queue all included pending points. The Python process runs the queue independently of the browser. Each point settles, records, coasts to IDLE, and waits until both reported speeds are below 5 rpm before the next start. A fault, incomplete acquisition, unsupported feedback mode or missing capability pauses the queue with a reason. Stop cancels pending work. An application restart never resumes motion automatically.

Feedback routing is never switched by the queue. Automatic sensorless startup remains blocked; a mixed queue pauses at such a point. Independently commissioned and already running sensorless sessions can be manually recorded using a selected sensorless plan, or imported from an external acquisition. A recorded tile means acquisition finished, not that an operator has accepted the result.

## Processing current ripple

**Results → Process all recorded results** reads original high-rate CSV samples. It uses a common duration (the shortest eligible buffer) so sensored and sensorless channel counts do not give unequal maximum-search windows. It fits a DC term plus sine/cosine at the electrical fundamental separately for each phase. The fundamental comes from captured speed and configured pole pairs, or a previously saved analysis of that same dataset with an explicit frequency for imported data. For imports without speed, first load the dataset, enter its frequency and Analyze selected window; then Process all recorded results. A frequency entered for one run is never applied to other speeds.

The metric selector offers:

- **Peak-to-peak:** `max over A/B/C of (max residual - min residual)` in amperes.
- **Absolute peak:** `max over A/B/C of max(abs(residual))` in amperes.

Each load graph has separate sensored and sensorless lines against rpm. A point is the maximum across valid independent repeats; the results table and CSV retain both metrics for each repeat. Retried attempts count once (latest valid attempt). Different acquisition sources, rates, bandwidths, filtering, device identities, calibration or analysis windows remain separate groups. Missing data remains gaps. Operator acceptance is displayed separately from numerical quality checks; process again after changes to runs/reviews.

Unknown bandwidth/filtering, missing phase currents, timing gaps, clipping, partial/interrupted captures, speed outside tolerance or insufficient electrical cycles prevent ripple results. These are in-band residual metrics and include harmonics/noise; they are not isolated PWM ripple. Raw data, metadata and derived results are retained in each run ZIP; **Export processed results CSV** saves the comparison table with both metrics and run IDs.

## Files and checks

- `app.py`: local HTTP server, test workflow and host CSV recording.
- `hardware.py`: real board connections, telemetry, readiness checks and explicit motor commands.
- `experiment.py`: plans, run storage, review history and ZIP export.
- `signals.py` and `analysis.py`: channel definitions, quality checks and numerical analysis.
- `import_data.py`: external CSV import and provenance.
- `capture_runtime.py`: onboard capture, original samples and per-board metadata.
- `batch_runner.py`: persistent test queue and restart recovery.
- `processing.py`: quality-gated ripple comparisons and processed CSV export.
- `capture_helpers.py`: optional standalone development helper.
- `recordings/connection-profile.json`: explicitly saved local profile.

Run `python -m unittest discover -p 'test_*.py'` using the local environment for automated checks. Test fixtures exercise disconnected behavior, command validation and data handling without connecting physical boards. Passing them does not validate actual motor control, sensorless handover, USB timing, emergency stopping or measurement accuracy.

Official references for commissioning and API details:

- [ODrive Python package](https://docs.odriverobotics.com/v/latest/guides/python-package.html)
- [ODrive hardware configuration](https://docs.odriverobotics.com/v/latest/manual/hardware-config.html)
- [ODrive sensorless operation](https://docs.odriverobotics.com/v/latest/manual/hardware-config.html#sensorless)

- [ODrive high-rate capture](https://docs.odriverobotics.com/v/latest/interfaces/odrivetool.html#high-rate-capture)

## Fixing blocked setup checks

Open **Connect and status → Resolve blocked board settings**. The five cards show the current local limits and each board's watchdog and control-mode checks.

1. Enter validated local maximum speed and load current, then **Save local rig limits**. These only limit commands from this computer. Connected boards must be idle; values above their existing velocity/current limits are rejected.
2. Connect the boards and verify their physical roles and axis units on Connect and status. Use **Read current board settings** to refresh the forms.
3. Enter compatible watchdog timeouts and validated ramp rates. Test speed ramp is entered in rpm/s and converted to turns/s²; load torque ramp is entered in N·m/s.
4. Click **Preview board changes**. Inspect both serial numbers and every current/proposed value. The tool sets test velocity control + velocity ramp, load torque control + torque ramp, enables both watchdogs, and zeros current and initial inputs. It does not run calibration, switch encoders or increase firmware current/voltage limits.
5. Click **Apply reviewed board changes**. Each setting is read back. Both boards must stay IDLE, disarmed and below 1 rpm; no tests, captures or batches may be active. A changed/expired preview is rejected. Partial failures remain visible and are logged under `recordings/settings-history.jsonl`.
6. To retain settings across power loss, tick the separate save option before Apply. This saves the entire current board configuration and may reboot the boards. Automatic startup must be disabled; connections are released and you must reconnect to verify persistence. A lost save response is reported as unconfirmed, never treated as success.

These actions do not start either motor. Other readiness checks still apply. The watchdog timeout must be at least max(0.5 seconds, six polling periods) and at most 2 seconds. Lower polling rates may need adjustment in Connections. Clear errors is a separate action after resolving their cause.
