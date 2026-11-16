import React from "react";

export default function FlightTimelineControls({
  currentTime,
  maxTime = 300.0,
  isPlaying,
  onTogglePlay,
  onSeek,
  speed,
  onSetSpeed,
  onReset,
}) {
  const progressPercent = Math.min(
    100,
    Math.max(0, (currentTime / maxTime) * 100),
  );
  const payloadDropPercent = (150.0 / maxTime) * 100;

  return (
    <div className="bg-[#161b22] border border-[#30363d] rounded p-3 shadow-sm flex flex-col gap-2.5">
      <div className="flex items-center justify-between">
        {/* Controls */}
        <div className="flex items-center gap-2">
          <button
            onClick={onTogglePlay}
            className="flex items-center justify-center w-8 h-8 rounded bg-[#21262d] hover:bg-[#30363d] border border-[#30363d] text-white font-mono text-xs transition"
            title={isPlaying ? "Pause" : "Play"}
          >
            {isPlaying ? "❚❚" : "▶"}
          </button>

          <button
            onClick={onReset}
            className="flex items-center justify-center w-8 h-8 rounded bg-[#21262d] hover:bg-[#30363d] border border-[#30363d] text-[#8b949e] hover:text-white font-mono text-xs transition"
            title="Reset to 0s"
          >
            ⟲
          </button>

          <div className="flex items-center gap-1 bg-[#0d1117] p-0.5 rounded border border-[#30363d] text-[11px] font-mono">
            {[1, 2, 5, 10].map((s) => (
              <button
                key={s}
                onClick={() => onSetSpeed(s)}
                className={`px-2 py-0.5 rounded transition ${
                  speed === s
                    ? "bg-[#38bdf8] text-[#0d1117] font-bold"
                    : "text-[#8b949e] hover:text-white"
                }`}
              >
                {s}×
              </button>
            ))}
          </div>
        </div>

        {/* Telemetry Time & Annunciator */}
        <div className="flex items-center gap-3 text-xs font-mono">
          <div className="flex items-center gap-1.5 bg-[#0d1117] px-2.5 py-1 rounded border border-[#30363d]">
            <span className="text-[#8b949e]">T_MET:</span>
            <span className="text-white font-bold tabular-nums">
              {currentTime.toFixed(1)}s
            </span>
            <span className="text-[#6e7681]">/ {maxTime.toFixed(0)}s</span>
          </div>

          {currentTime >= 150.0 ? (
            <div className="px-2 py-1 bg-[#21262d] border border-[#f85149] text-[#f85149] text-[10px] rounded font-semibold tracking-wider uppercase">
              STATUS: PAYLOAD JETTISON (10.5 kg)
            </div>
          ) : (
            <div className="px-2 py-1 bg-[#21262d] border border-[#30363d] text-[#8b949e] text-[10px] rounded font-semibold tracking-wider uppercase">
              STATUS: NOMINAL ENVELOPE (13.5 kg)
            </div>
          )}
        </div>
      </div>

      {/* Scrub Bar */}
      <div className="relative w-full flex items-center">
        {/* Payload Release Event Marker */}
        <div
          className="absolute z-20 pointer-events-none -top-3 flex flex-col items-center"
          style={{ left: `${payloadDropPercent}%` }}
        >
          <span className="text-[9px] font-mono font-semibold text-[#f85149] tracking-wider uppercase">
            DROP (150s)
          </span>
          <div className="w-[1px] h-4 bg-[#f85149]" />
        </div>

        <input
          type="range"
          min="0"
          max={maxTime}
          step="0.1"
          value={currentTime}
          onChange={(e) => onSeek(parseFloat(e.target.value))}
          className="w-full h-1.5 bg-[#0d1117] rounded cursor-pointer accent-[#38bdf8] border border-[#30363d]"
        />
      </div>
    </div>
  );
}
