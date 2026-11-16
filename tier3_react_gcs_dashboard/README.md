# Tier 3: React Web GCS Dashboard

Interactive, glass-cockpit Ground Control Station (GCS) telemetry dashboard designed to visualize, compare, and benchmark the **Baseline UAV** and the **Proposed Adaptive UAV** side-by-side in real time.

Powered by Vite, React 18, and Tailwind CSS with direct telemetry feeding from Tier 1 (`flight_telemetry.json`).

---

## 1. Key Features

### Dual Glass-Cockpit Primary Flight Displays (PFD)
* **Real-Time Artificial Horizon**: Dynamic pitch ladder and roll rotation responding to 6-DoF attitude rates.
* **Vertical Airspeed Tapes**: Shows true indicated airspeed $V_a$ with target commanded speed bug $V_{a,\text{cmd}}$.
* **Altitude & Heading Tapes**: Displays active barometric altitude and course tracking.

### Tactical 3D Flight Corridor Viewport
* Visualizes the **$6\,\text{km}$ North-East-Down flight corridor** through waypoints WP0 $\to$ WP2.
* Renders real-time Delta-wing avatars for both aircraft.
* Dynamically displays the **headwind lead gap** as the Adaptive UAV outperforms the Baseline in $6.0\,\text{m/s}$ headwinds.
* Highlights the payload drop event at $t = 150\,\text{s}$.

### Interactive Mass Observability Proof
* Renders the true mass step ($13.5\,\text{kg} \to 10.5\,\text{kg}$ at $t = 150\,\text{s}$).
* Visually exposes the **legacy MATLAB EKF defect** (flatline stuck at $13.5\,\text{kg}$ due to rank-deficient $H = [1, -1, 0]$).
* Displays the **Observable EKF convergence** to $10.5\,\text{kg}$ within seconds alongside its calculated $\pm 3\sigma$ confidence envelope.

### Actuator Energy Benchmark Gauge
* Live side-by-side cumulative energy consumption progress bars.
* Highlights the **$> 10\%$ total energy reduction** achieved by the analytical optimal airspeed guidance law.

### Interactive Mission Timeline Controls
* Play / Pause, scrubbing slider across the full $300\,\text{s}$ flight.
* $1\times$, $2\times$, $5\times$, and $10\times$ playback speed multipliers.
* Automatic event detection marking payload release.

---

## 2. Quick Start

### 1. Install Dependencies
From within `tier3_react_gcs_dashboard`:
```bash
npm install
```

### 2. Launch Local Dev Server
```bash
npm run dev
```
Open [http://localhost:3000](http://localhost:3000) in your browser to launch the mission control dashboard.

### 3. Build for Production Deployment
```bash
npm run build
```
Generates an optimized static bundle in `dist/`.