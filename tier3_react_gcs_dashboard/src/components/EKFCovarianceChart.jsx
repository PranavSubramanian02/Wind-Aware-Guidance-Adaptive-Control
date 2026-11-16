import React from "react";

export default function EKFCovarianceChart({
  timeline = [],
  currentTime = 0.0,
  maxTime = 300.0,
}) {
  const width = 540;
  const height = 180;
  const padding = { top: 20, right: 30, bottom: 25, left: 45 };

  const innerW = width - padding.left - padding.right;
  const innerH = height - padding.top - padding.bottom;

  const minM = 8.0;
  const maxM = 16.0;

  const getX = (t) => padding.left + (t / maxTime) * innerW;
  const getY = (m) => padding.top + (1.0 - (m - minM) / (maxM - minM)) * innerH;

  const stride = Math.max(1, Math.floor(timeline.length / 150));
  const sampled = timeline.filter((_, idx) => idx % stride === 0);

  const truePath =
    sampled.length > 0
      ? sampled
          .map(
            (d, i) =>
              `${i === 0 ? "M" : "L"} ${getX(d.time)} ${getY(d.proposed.true_mass)}`,
          )
          .join(" ")
      : "";

  const legacyPath =
    sampled.length > 0
      ? sampled
          .map(
            (d, i) =>
              `${i === 0 ? "M" : "L"} ${getX(d.time)} ${getY(d.proposed.legacy_mass)}`,
          )
          .join(" ")
      : "";

  const ekfPath =
    sampled.length > 0
      ? sampled
          .map(
            (d, i) =>
              `${i === 0 ? "M" : "L"} ${getX(d.time)} ${getY(d.proposed.ekf_mass)}`,
          )
          .join(" ")
      : "";

  const upperPoints = sampled
    .map(
      (d) =>
        `${getX(d.time)},${getY(Math.min(maxM, d.proposed.ekf_mass + 3.0 * d.proposed.ekf_sigma))}`,
    )
    .join(" ");
  const lowerPoints = sampled
    .map(
      (d) =>
        `${getX(d.time)},${getY(Math.max(minM, d.proposed.ekf_mass - 3.0 * d.proposed.ekf_sigma))}`,
    )
    .reverse()
    .join(" ");

  const confidencePolygon = `${upperPoints} ${lowerPoints}`;

  const currentSample =
    timeline.length > 0
      ? timeline[
          Math.min(
            timeline.length - 1,
            Math.max(
              0,
              Math.floor((currentTime / maxTime) * (timeline.length - 1)),
            ),
          )
        ]
      : {
          proposed: { ekf_mass: 13.5, true_mass: 13.5, legacy_mass: 13.5 },
        };

  return (
    <div className="bg-[#161b22] border border-[#30363d] rounded p-3 flex flex-col gap-2 shadow-sm font-sans">
      {/* Header */}
      <div className="flex items-center justify-between border-b border-[#30363d] pb-2 text-xs">
        <div className="flex items-center gap-2">
          <span className="font-semibold text-white tracking-wide uppercase text-xs">
            In-Flight Mass Estimation (EKF Observability)
          </span>
          <span className="text-[10px] text-[#3fb950] font-mono border border-[#238636] px-1 rounded">
            rank(O) = 3/3
          </span>
        </div>

        <div className="flex items-center gap-3 text-xs font-mono">
          <div className="flex items-center gap-1.5">
            <span className="w-2.5 h-0.5 bg-[#38bdf8] rounded" />
            <span className="text-[#8b949e]">EKF:</span>
            <span className="text-[#38bdf8] font-bold tabular-nums">
              {currentSample.proposed.ekf_mass.toFixed(2)} kg
            </span>
          </div>

          <div className="flex items-center gap-1.5">
            <span className="w-2.5 h-0.5 bg-[#f87171] rounded" />
            <span className="text-[#8b949e]">Legacy:</span>
            <span className="text-[#f87171] font-bold tabular-nums">
              {currentSample.proposed.legacy_mass.toFixed(1)} kg
            </span>
          </div>

          <div className="flex items-center gap-1.5">
            <span className="text-[#8b949e]">True:</span>
            <span className="text-white font-bold tabular-nums">
              {currentSample.proposed.true_mass.toFixed(1)} kg
            </span>
          </div>
        </div>
      </div>

      {/* SVG Chart */}
      <div className="relative w-full overflow-hidden bg-[#0d1117] rounded border border-[#30363d]">
        <svg viewBox={`0 0 ${width} ${height}`} className="w-full h-auto">
          {/* Horizontal gridlines */}
          {[10.0, 12.0, 14.0].map((m) => (
            <g key={m}>
              <line
                x1={padding.left}
                y1={getY(m)}
                x2={width - padding.right}
                y2={getY(m)}
                stroke="#21262d"
                strokeDasharray="2 3"
              />
              <text
                x={padding.left - 6}
                y={getY(m) + 3}
                textAnchor="end"
                fill="#6e7681"
                fontSize="9"
                fontFamily="Segoe UI Mono, Consolas, monospace"
              >
                {m.toFixed(0)}kg
              </text>
            </g>
          ))}

          {/* Vertical Payload Drop Line at 150s */}
          <line
            x1={getX(150.0)}
            y1={padding.top}
            x2={getX(150.0)}
            y2={height - padding.bottom}
            stroke="#f85149"
            strokeDasharray="3 3"
            strokeWidth="1"
          />
          <text
            x={getX(150.0) + 4}
            y={padding.top + 10}
            fill="#f85149"
            fontSize="8"
            fontFamily="Segoe UI, sans-serif"
            fontWeight="600"
          >
            DROP EVENT (150s)
          </text>

          {/* Confidence Bounds (3-sigma) */}
          {sampled.length > 0 && (
            <polygon
              points={confidencePolygon}
              fill="#38bdf8"
              fillOpacity="0.12"
            />
          )}

          {/* Legacy Divergent Path */}
          {legacyPath && (
            <path
              d={legacyPath}
              fill="none"
              stroke="#f87171"
              strokeWidth="1.2"
              strokeDasharray="4 2"
              opacity="0.7"
            />
          )}

          {/* True Mass Path */}
          {truePath && (
            <path
              d={truePath}
              fill="none"
              stroke="#8b949e"
              strokeWidth="1.5"
            />
          )}

          {/* EKF Estimated Mass Path */}
          {ekfPath && (
            <path
              d={ekfPath}
              fill="none"
              stroke="#38bdf8"
              strokeWidth="1.8"
            />
          )}

          {/* Current Time Indicator Vertical Line */}
          <line
            x1={getX(currentTime)}
            y1={padding.top}
            x2={getX(currentTime)}
            y2={height - padding.bottom}
            stroke="#ffffff"
            strokeWidth="1"
            opacity="0.8"
          />

          {/* X Axis Baseline */}
          <line
            x1={padding.left}
            y1={height - padding.bottom}
            x2={width - padding.right}
            y2={height - padding.bottom}
            stroke="#30363d"
          />

          {/* Time ticks */}
          {[0, 60, 120, 180, 240, 300].map((t) => (
            <g key={t} transform={`translate(${getX(t)}, ${height - padding.bottom})`}>
              <line y2="3" stroke="#30363d" />
              <text
                y="14"
                textAnchor="middle"
                fill="#6e7681"
                fontSize="9"
                fontFamily="Segoe UI Mono, Consolas, monospace"
              >
                {t}s
              </text>
            </g>
          ))}
        </svg>
      </div>
    </div>
  );
}
