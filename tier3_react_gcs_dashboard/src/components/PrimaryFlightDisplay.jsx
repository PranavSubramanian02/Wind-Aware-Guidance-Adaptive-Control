import React from "react";

export default function PrimaryFlightDisplay({
  title = "UAV PFD",
  subTitle = "GNC Architecture",
  airspeed = 25.0,
  airspeedCmd = 25.0,
  altitude = 100.0,
  altitudeCmd = 100.0,
  pitch = 0.0,
  roll = 0.0,
  heading = 0.0,
  energyKj = 0.0,
  variant = "proposed",
}) {
  const isProposed = variant === "proposed";
  const borderClass = isProposed ? "border-[#38bdf8]/40" : "border-[#f87171]/40";
  const badgeColor = isProposed ? "text-[#38bdf8]" : "text-[#f87171]";

  // Pitch translation (pixels per degree)
  const pitchOffset = Math.max(-60, Math.min(60, pitch * 4));

  return (
    <div
      className={`bg-[#161b22] border ${borderClass} rounded p-3 flex flex-col gap-2 shadow-sm relative overflow-hidden`}
    >
      {/* Flight Card Header */}
      <div className="flex items-center justify-between border-b border-[#30363d] pb-2">
        <div className="flex flex-col">
          <div className="flex items-center gap-2">
            <span className={`font-semibold text-xs tracking-wider uppercase ${badgeColor}`}>
              {title}
            </span>
          </div>
          <span className="text-[10px] text-[#8b949e]">
            {subTitle}
          </span>
        </div>
        <div className="flex items-center gap-2 font-mono text-[11px]">
          <span className="bg-[#0d1117] px-2 py-0.5 rounded border border-[#30363d] text-[#8b949e]">
            HDG: <span className="font-bold text-white tabular-nums">{heading.toFixed(0)}°</span>
          </span>
        </div>
      </div>

      {/* Main PFD Viewport */}
      <div className="relative w-full h-44 bg-[#0d1117] rounded border border-[#30363d] overflow-hidden flex items-center justify-center">
        {/* Artificial Horizon Background (FAA Blue / Sepia) */}
        <div
          className="absolute w-80 h-80 transition-transform duration-75 ease-out pointer-events-none"
          style={{
            transform: `rotate(${-roll}deg) translateY(${pitchOffset}px)`,
          }}
        >
          {/* Sky: Aviation Blue */}
          <div className="w-full h-1/2 bg-[#1b3b5f] border-b border-white" />
          {/* Ground: Aviation Brown */}
          <div className="w-full h-1/2 bg-[#3d2612]" />

          {/* Pitch Ladder Marks */}
          {[-20, -10, 10, 20].map((deg) => (
            <div
              key={deg}
              className="absolute left-1/2 -translate-x-1/2 flex items-center gap-1.5 font-mono text-[9px] text-white/90"
              style={{ top: `${50 - deg * 2}%` }}
            >
              <div className="w-5 h-[1.5px] bg-white/90" />
              <span className="tabular-nums font-semibold">{Math.abs(deg)}</span>
              <div className="w-5 h-[1.5px] bg-white/90" />
            </div>
          ))}
        </div>

        {/* Aircraft Fixed Center Symbol (FAA Yellow/Amber Inverted Gull-Wing) */}
        <div className="absolute z-10 pointer-events-none flex items-center justify-center">
          <div className="w-7 h-[2.5px] bg-[#e5a93c]" />
          <div className="w-2.5 h-2.5 border-[2px] border-[#e5a93c] rounded-full mx-1" />
          <div className="w-7 h-[2.5px] bg-[#e5a93c]" />
        </div>

        {/* Left Tape: Airspeed (IAS) */}
        <div className="absolute left-2 top-2.5 bottom-2.5 w-14 bg-[#0d1117]/90 border border-[#30363d] rounded flex flex-col justify-between items-center py-1 z-20 font-mono">
          <span className="text-[10px] text-[#8b949e] font-bold">IAS</span>
          <div className="px-1.5 py-0.5 rounded bg-[#1c2128] border border-[#30363d] text-xs font-bold text-white tabular-nums">
            {airspeed.toFixed(1)}
          </div>
          <span className="text-[9px] text-[#3fb950] font-semibold tabular-nums">
            CMD {airspeedCmd.toFixed(1)}
          </span>
        </div>

        {/* Right Tape: Altitude (ALT) */}
        <div className="absolute right-2 top-2.5 bottom-2.5 w-14 bg-[#0d1117]/90 border border-[#30363d] rounded flex flex-col justify-between items-center py-1 z-20 font-mono">
          <span className="text-[10px] text-[#8b949e] font-bold">ALT</span>
          <div className="px-1.5 py-0.5 rounded bg-[#1c2128] border border-[#30363d] text-xs font-bold text-white tabular-nums">
            {altitude.toFixed(0)}m
          </div>
          <span className="text-[9px] text-[#8b949e] font-semibold tabular-nums">
            CMD {altitudeCmd.toFixed(0)}m
          </span>
        </div>
      </div>

      {/* Footer Engineering Metrics */}
      <div className="grid grid-cols-3 gap-1.5 pt-0.5 font-mono text-xs border-t border-[#30363d]">
        <div className="bg-[#0d1117] p-1.5 rounded border border-[#21262d] text-center">
          <div className="text-[9px] text-[#8b949e] uppercase font-sans">Actuator Energy</div>
          <div className="font-semibold text-white tabular-nums text-[11px]">
            {energyKj.toFixed(1)}{" "}
            <span className="text-[9px] text-[#6e7681]">kJ</span>
          </div>
        </div>
        <div className="bg-[#0d1117] p-1.5 rounded border border-[#21262d] text-center">
          <div className="text-[9px] text-[#8b949e] uppercase font-sans">Pitch / Roll</div>
          <div className="font-semibold text-white tabular-nums text-[11px]">
            {pitch.toFixed(1)}° / {roll.toFixed(1)}°
          </div>
        </div>
        <div className="bg-[#0d1117] p-1.5 rounded border border-[#21262d] text-center">
          <div className="text-[9px] text-[#8b949e] uppercase font-sans">Tracking Error</div>
          <div
            className={`font-semibold tabular-nums text-[11px] ${
              Math.abs(airspeed - airspeedCmd) > 1.0 ? "text-[#d29922]" : "text-[#3fb950]"
            }`}
          >
            {(airspeed - airspeedCmd).toFixed(2)}{" "}
            <span className="text-[9px] text-[#6e7681]">m/s</span>
          </div>
        </div>
      </div>
    </div>
  );
}
