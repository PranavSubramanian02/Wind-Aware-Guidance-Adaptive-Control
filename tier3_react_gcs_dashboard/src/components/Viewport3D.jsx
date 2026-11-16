import React from "react";

export default function Viewport3D({
  basePn = 0.0,
  propPn = 0.0,
  currentTime = 0.0,
  timeline = [],
}) {
  const width = 600;
  const height = 210;
  const maxNorth = 6000.0;

  const padding = { top: 25, right: 35, bottom: 30, left: 45 };
  const innerW = width - padding.left - padding.right;
  const innerH = height - padding.top - padding.bottom;

  const getX = (pn) =>
    padding.left + (Math.min(maxNorth, Math.max(0, pn)) / maxNorth) * innerW;

  const baseLaneY = padding.top + innerH * 0.35;
  const propLaneY = padding.top + innerH * 0.7;

  const basePosX = getX(basePn);
  const propPosX = getX(propPn);

  // Waypoints at 0m, 3000m, 6000m
  const waypoints = [0.0, 3000.0, 6000.0];

  return (
    <div className="bg-[#161b22] border border-[#30363d] rounded p-3 flex flex-col gap-2 shadow-sm font-sans">
      {/* Viewport Header */}
      <div className="flex items-center justify-between border-b border-[#30363d] pb-2 text-xs">
        <div className="flex items-center gap-2">
          <span className="font-semibold text-white tracking-wide uppercase text-xs">
            Tactical Flight Corridor & Trajectory Tracking
          </span>
          <span className="text-[10px] text-[#8b949e] font-mono">[NORTHING 0–6000 m]</span>
        </div>

        <div className="flex items-center gap-4 text-xs font-mono">
          <div className="flex items-center gap-1.5">
            <span className="w-2 h-2 rounded bg-[#f87171]" />
            <span className="text-[#8b949e] text-[11px]">Baseline (Fixed)</span>
          </div>
          <div className="flex items-center gap-1.5">
            <span className="w-2 h-2 rounded bg-[#38bdf8]" />
            <span className="text-[#8b949e] text-[11px]">Adaptive (Proposed)</span>
          </div>
          <div className="flex items-center gap-1 text-[#8b949e] border-l border-[#30363d] pl-3 text-[11px]">
            <span className="text-[#d29922]">WIND:</span>
            <span>6.0 m/s Headwind [000°]</span>
          </div>
        </div>
      </div>

      {/* SVG Corridor Viewport */}
      <div className="relative w-full overflow-hidden bg-[#0d1117] rounded border border-[#30363d]">
        <svg viewBox={`0 0 ${width} ${height}`} className="w-full h-auto">
          {/* Grid lines */}
          {[0, 1500, 3000, 4500, 6000].map((m) => (
            <line
              key={m}
              x1={getX(m)}
              y1={padding.top}
              x2={getX(m)}
              y2={height - padding.bottom}
              stroke="#21262d"
              strokeDasharray="2 3"
              strokeWidth="1"
            />
          ))}

          {/* Waypoint Markers */}
          {waypoints.map((wp, i) => (
            <g key={wp} transform={`translate(${getX(wp)}, 0)`}>
              <line
                x1={0}
                y1={padding.top}
                x2={0}
                y2={height - padding.bottom}
                stroke="#30363d"
                strokeWidth="1"
              />
              <circle cx={0} cy={baseLaneY} r="3" fill="#6e7681" />
              <circle cx={0} cy={propLaneY} r="3" fill="#6e7681" />
              <text
                x={0}
                y={padding.top - 7}
                textAnchor="middle"
                fill="#8b949e"
                fontSize="9"
                fontFamily="Segoe UI, sans-serif"
                fontWeight="600"
              >
                {`WP${i} (${(wp / 1000).toFixed(0)} km)`}
              </text>
            </g>
          ))}

          {/* Lane 1: Baseline Trajectory Path */}
          <line
            x1={padding.left}
            y1={baseLaneY}
            x2={width - padding.right}
            y2={baseLaneY}
            stroke="#21262d"
            strokeDasharray="4 4"
            strokeWidth="1.5"
          />
          {/* Baseline Active Travel Trail */}
          <line
            x1={padding.left}
            y1={baseLaneY}
            x2={basePosX}
            y2={baseLaneY}
            stroke="#f87171"
            strokeWidth="2"
            opacity="0.8"
          />

          {/* Baseline Aircraft Tactical Symbol */}
          <g transform={`translate(${basePosX}, ${baseLaneY})`}>
            <polygon
              points="10,0 -8,-5 -4,0 -8,5"
              fill="#f87171"
              stroke="#ffffff"
              strokeWidth="0.8"
            />
            <text
              x={-12}
              y={-7}
              fill="#f87171"
              fontSize="9"
              fontFamily="Segoe UI Mono, Consolas, monospace"
              fontWeight="bold"
              textAnchor="end"
            >
              BASE
            </text>
            <text
              x={-12}
              y={14}
              fill="#8b949e"
              fontSize="8"
              fontFamily="Segoe UI Mono, Consolas, monospace"
              textAnchor="end"
            >
              {basePn.toFixed(0)}m
            </text>
          </g>

          {/* Lane 2: Proposed Trajectory Path */}
          <line
            x1={padding.left}
            y1={propLaneY}
            x2={width - padding.right}
            y2={propLaneY}
            stroke="#21262d"
            strokeDasharray="4 4"
            strokeWidth="1.5"
          />
          {/* Proposed Active Travel Trail */}
          <line
            x1={padding.left}
            y1={propLaneY}
            x2={propPosX}
            y2={propLaneY}
            stroke="#38bdf8"
            strokeWidth="2"
            opacity="0.8"
          />

          {/* Proposed Aircraft Tactical Symbol */}
          <g transform={`translate(${propPosX}, ${propLaneY})`}>
            <polygon
              points="10,0 -8,-5 -4,0 -8,5"
              fill="#38bdf8"
              stroke="#ffffff"
              strokeWidth="0.8"
            />
            <text
              x={-12}
              y={-7}
              fill="#38bdf8"
              fontSize="9"
              fontFamily="Segoe UI Mono, Consolas, monospace"
              fontWeight="bold"
              textAnchor="end"
            >
              PROP
            </text>
            <text
              x={-12}
              y={14}
              fill="#8b949e"
              fontSize="8"
              fontFamily="Segoe UI Mono, Consolas, monospace"
              textAnchor="end"
            >
              {propPn.toFixed(0)}m
            </text>
          </g>
        </svg>
      </div>
    </div>
  );
}
