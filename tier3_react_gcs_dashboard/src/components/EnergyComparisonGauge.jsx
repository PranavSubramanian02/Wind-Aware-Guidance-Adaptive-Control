import React from "react";

export default function EnergyComparisonGauge({
  baseEnergy = 0.0,
  propEnergy = 0.0,
  savedPercent = 10.77,
  baseAirspeed = 25.0,
  propAirspeed = 25.0,
}) {
  // Max scale reference for progress bars
  const maxScaleKj = 130.0;
  const basePct = Math.min(100, (baseEnergy / maxScaleKj) * 100);
  const propPct = Math.min(100, (propEnergy / maxScaleKj) * 100);

  const deltaEnergyKj = Math.max(0.0, baseEnergy - propEnergy);

  return (
    <div className="bg-[#161b22] border border-[#30363d] rounded p-3 flex flex-col gap-2.5 shadow-sm font-sans">
      {/* Header */}
      <div className="flex items-center justify-between border-b border-[#30363d] pb-2 text-xs">
        <div className="flex items-center gap-2">
          <span className="font-semibold text-white tracking-wide uppercase text-xs">
            Actuator Energy Consumption Audit
          </span>
          <span className="text-[10px] text-[#8b949e] font-mono">[6-DoF MISSION 300s]</span>
        </div>
        <div className="flex items-center gap-1.5 px-2 py-0.5 bg-[#1c2128] border border-[#238636] rounded text-[#3fb950] font-mono font-bold text-xs">
          <span>{savedPercent.toFixed(1)}% PRESERVED</span>
        </div>
      </div>

      {/* Side-by-Side Comparison Bars */}
      <div className="flex flex-col gap-2">
        {/* Baseline Bar */}
        <div className="flex flex-col gap-1">
          <div className="flex justify-between text-xs font-mono">
            <span className="text-[#f87171] font-semibold text-[11px]">
              Baseline UAV (Fixed Trim + PID)
            </span>
            <span className="text-white font-bold tabular-nums">
              {baseEnergy.toFixed(1)}{" "}
              <span className="text-[#8b949e] text-[10px]">kJ</span>
            </span>
          </div>
          <div className="w-full h-2 bg-[#0d1117] rounded overflow-hidden border border-[#30363d]">
            <div
              className="h-full bg-[#f87171] transition-all duration-100 ease-out"
              style={{ width: `${basePct}%` }}
            />
          </div>
        </div>

        {/* Proposed Adaptive Bar */}
        <div className="flex flex-col gap-1">
          <div className="flex justify-between text-xs font-mono">
            <span className="text-[#38bdf8] font-semibold text-[11px]">
              Adaptive UAV (Optimal Guidance + MRAC)
            </span>
            <span className="text-white font-bold tabular-nums">
              {propEnergy.toFixed(1)}{" "}
              <span className="text-[#8b949e] text-[10px]">kJ</span>
            </span>
          </div>
          <div className="w-full h-2 bg-[#0d1117] rounded overflow-hidden border border-[#30363d]">
            <div
              className="h-full bg-[#38bdf8] transition-all duration-100 ease-out"
              style={{ width: `${propPct}%` }}
            />
          </div>
        </div>

        {/* Scale Markings */}
        <div className="flex justify-between text-[9px] text-[#6e7681] font-mono px-0.5">
          <span>0 kJ</span>
          <span>30 kJ</span>
          <span>60 kJ</span>
          <span>90 kJ</span>
          <span>130 kJ</span>
        </div>
      </div>

      {/* Telemetry Summary Metrics */}
      <div className="grid grid-cols-2 gap-2 pt-1 border-t border-[#30363d] font-mono text-xs">
        <div className="bg-[#0d1117] p-2 rounded border border-[#21262d] flex justify-between items-center">
          <span className="text-[#8b949e] text-[11px] font-sans">Total Preserved</span>
          <span className="text-[#3fb950] font-bold text-xs tabular-nums">
            +{deltaEnergyKj.toFixed(1)} kJ
          </span>
        </div>

        <div className="bg-[#0d1117] p-2 rounded border border-[#21262d] flex justify-between items-center">
          <span className="text-[#8b949e] text-[11px] font-sans">Optimal V_a vs Base</span>
          <span className="text-white font-bold text-xs tabular-nums">
            {propAirspeed.toFixed(1)}{" "}
            <span className="text-[#6e7681] text-[10px]">vs</span>{" "}
            {baseAirspeed.toFixed(1)}{" "}
            <span className="text-[#6e7681] text-[10px]">m/s</span>
          </span>
        </div>
      </div>
    </div>
  );
}
