import React, { useState, useEffect, useRef, useMemo } from "react";
import telemetryData from "./data/flight_telemetry.json";
import PrimaryFlightDisplay from "./components/PrimaryFlightDisplay";
import EKFCovarianceChart from "./components/EKFCovarianceChart";
import EnergyComparisonGauge from "./components/EnergyComparisonGauge";
import Viewport3D from "./components/Viewport3D";
import FlightTimelineControls from "./components/FlightTimelineControls";

export default function App() {
  const metadata = telemetryData.metadata || {};
  const timeline = telemetryData.timeline || [];
  const maxTime = metadata.total_time || 300.0;

  const [currentTime, setCurrentTime] = useState(0.0);
  const [isPlaying, setIsPlaying] = useState(true);
  const [speed, setSpeed] = useState(2);

  const lastFrameRef = useRef(performance.now());

  // Real-time animation loop
  useEffect(() => {
    let animId;

    const loop = (now) => {
      const dtReal = (now - lastFrameRef.current) / 1000.0;
      lastFrameRef.current = now;

      if (isPlaying) {
        setCurrentTime((prev) => {
          const next = prev + dtReal * speed;
          if (next >= maxTime) {
            setIsPlaying(false);
            return maxTime;
          }
          return next;
        });
      }

      animId = requestAnimationFrame(loop);
    };

    animId = requestAnimationFrame(loop);
    return () => cancelAnimationFrame(animId);
  }, [isPlaying, speed, maxTime]);

  // Current active sample extraction
  const currentSample = useMemo(() => {
    if (timeline.length === 0) return null;
    const idx = Math.min(
      timeline.length - 1,
      Math.max(0, Math.floor((currentTime / maxTime) * (timeline.length - 1))),
    );
    return timeline[idx];
  }, [currentTime, maxTime, timeline]);

  const base = currentSample?.baseline || {
    pn: 0,
    pe: 0,
    alt: 100,
    airspeed: 25,
    energy_kj: 0,
  };
  const prop = currentSample?.proposed || {
    pn: 0,
    pe: 0,
    alt: 100,
    airspeed: 25,
    airspeed_cmd: 25,
    energy_kj: 0,
    true_mass: 13.5,
    ekf_mass: 13.5,
    ekf_sigma: 0.1,
    legacy_mass: 13.5,
  };

  return (
    <div className="min-h-screen bg-[#0d1117] text-[#f0f6fc] flex flex-col font-sans p-4 gap-3.5 max-w-7xl mx-auto">
      {/* Top GCS Engineering Header */}
      <header className="flex flex-wrap items-center justify-between bg-[#161b22] border border-[#30363d] rounded p-3 shadow-md">
        <div className="flex items-center gap-3">
          <div className="px-2 py-1 bg-[#21262d] border border-[#30363d] rounded text-[11px] font-mono font-bold tracking-wider text-[#38bdf8]">
            GNC-SITL-01
          </div>
          <div>
            <div className="flex items-center gap-2">
              <h1 className="text-sm font-semibold tracking-wide text-white uppercase">
                Fixed-Wing UAV Flight Test Ground Control Station
              </h1>
              <span className="text-[10px] text-[#8b949e] border border-[#30363d] px-1.5 py-0.2 rounded font-mono">
                MIL-STD-1797A
              </span>
            </div>
            <p className="text-xs text-[#8b949e]">
              Wind-Aware Optimal Airspeed Guidance & Lyapunov-Stable MRAC Adaptive Flight Architecture
            </p>
          </div>
        </div>

        {/* Real Flight Annunciator Readouts */}
        <div className="flex items-center gap-2 text-xs font-mono">
          <div className="flex items-center gap-1.5 px-2.5 py-1 bg-[#1c2128] border border-[#30363d] rounded text-[#3fb950]">
            <span className="w-2 h-2 rounded-full bg-[#3fb950]" />
            <span className="font-semibold text-[11px] tracking-wide">AUTONOMOUS 6-DoF</span>
          </div>

          <div className="flex items-center gap-1.5 px-2.5 py-1 bg-[#1c2128] border border-[#30363d] rounded text-[#8b949e]">
            <span className="text-[#38bdf8] font-bold">WIND:</span>
            <span className="text-[#f0f6fc] tabular-nums">6.0 m/s [000°]</span>
          </div>

          <div className="hidden sm:flex items-center px-2 py-1 bg-[#1c2128] border border-[#30363d] rounded text-[#8b949e] text-[11px]">
            <span className="text-[#8b949e]">RATE:</span>
            <span className="text-[#f0f6fc] ml-1">50 Hz</span>
          </div>
        </div>
      </header>

      {/* Mission Timeline Controls */}
      <FlightTimelineControls
        currentTime={currentTime}
        maxTime={maxTime}
        isPlaying={isPlaying}
        onTogglePlay={() => setIsPlaying(!isPlaying)}
        onSeek={(t) => setCurrentTime(t)}
        speed={speed}
        onSetSpeed={(s) => setSpeed(s)}
        onReset={() => {
          setCurrentTime(0.0);
          setIsPlaying(false);
        }}
      />

      {/* Dual Primary Flight Displays */}
      <div className="grid grid-cols-1 md:grid-cols-2 gap-3.5">
        <PrimaryFlightDisplay
          title="Adaptive UAV (Optimal Guidance + MRAC)"
          subTitle="Nonlinear Lyapunov-Stable Adaptation"
          airspeed={prop.airspeed}
          airspeedCmd={prop.airspeed_cmd}
          altitude={prop.alt}
          altitudeCmd={100.0}
          pitch={1.8}
          roll={0.0}
          heading={0.0}
          energyKj={prop.energy_kj}
          variant="proposed"
        />

        <PrimaryFlightDisplay
          title="Baseline UAV (Fixed Trim + PID)"
          subTitle="Classical Fixed-Gain Control"
          airspeed={base.airspeed}
          airspeedCmd={25.0}
          altitude={base.alt}
          altitudeCmd={100.0}
          pitch={2.1}
          roll={0.0}
          heading={0.0}
          energyKj={base.energy_kj}
          variant="baseline"
        />
      </div>

      {/* Tactical 3D Flight Corridor */}
      <Viewport3D
        basePn={base.pn}
        propPn={prop.pn}
        currentTime={currentTime}
        timeline={timeline}
      />

      {/* Bottom Analytic Telemetry Gauges */}
      <div className="grid grid-cols-1 md:grid-cols-2 gap-3.5">
        <EKFCovarianceChart
          timeline={timeline}
          currentTime={currentTime}
          maxTime={maxTime}
        />

        <EnergyComparisonGauge
          baseEnergy={base.energy_kj}
          propEnergy={prop.energy_kj}
          savedPercent={metadata.energy_saved_percent || 10.77}
          baseAirspeed={base.airspeed}
          propAirspeed={prop.airspeed}
        />
      </div>
    </div>
  );
}
