"use client";
import { startEval, stopEval, skipMission } from "@/lib/api";

interface Props {
  running: boolean;
  selected: Set<number>;
  totalMissions: number;
  timeout: number;
  onTimeoutChange: (t: number) => void;
}

export default function EvalControls({
  running,
  selected,
  totalMissions,
  timeout,
  onTimeoutChange,
}: Props) {
  const handleRunAll = () => {
    const ids = Array.from({ length: totalMissions }, (_, i) => i);
    startEval(ids, timeout);
  };

  const handleRunSelected = () => {
    if (selected.size === 0) return;
    startEval(Array.from(selected).sort((a, b) => a - b), timeout);
  };

  return (
    <div className="flex items-center gap-3 flex-wrap">
      <div className="flex items-center gap-1.5">
        <label className="text-xs text-gray-400">Timeout</label>
        <input
          type="number"
          value={timeout}
          onChange={(e) => onTimeoutChange(Number(e.target.value))}
          className="w-20 px-2 py-1 text-sm bg-gray-800 border border-gray-700 rounded"
        />
        <span className="text-xs text-gray-500">s</span>
      </div>

      <button
        onClick={handleRunAll}
        disabled={running}
        className="px-3 py-1.5 text-sm bg-green-700 hover:bg-green-600 rounded disabled:opacity-30"
      >
        Run All ({totalMissions})
      </button>

      <button
        onClick={handleRunSelected}
        disabled={running || selected.size === 0}
        className="px-3 py-1.5 text-sm bg-blue-700 hover:bg-blue-600 rounded disabled:opacity-30"
      >
        Run Selected ({selected.size})
      </button>

      <button
        onClick={() => skipMission()}
        disabled={!running}
        className="px-3 py-1.5 text-sm bg-orange-700 hover:bg-orange-600 rounded disabled:opacity-30"
      >
        Skip
      </button>

      <button
        onClick={() => stopEval()}
        disabled={!running}
        className="px-3 py-1.5 text-sm bg-red-700 hover:bg-red-600 rounded disabled:opacity-30"
      >
        Stop
      </button>
    </div>
  );
}
