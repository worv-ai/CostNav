"use client";
import { Play, Square, SkipForward, Clock, CheckCircle2 } from "lucide-react";
import { startEval, stopEval, skipMission } from "@/lib/api";
import Card from "@/components/Card";

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
    <Card className="flex items-center gap-3 flex-wrap">
      <div className="flex items-center gap-2 px-3 py-1.5 rounded-lg bg-[var(--surface-overlay)]">
        <Clock size={14} className="text-[var(--text-muted)]" />
        <input
          type="number"
          value={timeout}
          onChange={(e) => onTimeoutChange(Number(e.target.value))}
          className="w-16 bg-transparent text-sm font-mono outline-none text-[var(--text-primary)]"
        />
        <span className="text-xs text-[var(--text-muted)]">sec</span>
      </div>

      <button
        onClick={handleRunAll}
        disabled={running}
        className="inline-flex items-center gap-1.5 px-3.5 py-1.5 text-sm font-medium rounded-lg bg-emerald-600 text-white hover:bg-emerald-500 disabled:opacity-30 disabled:cursor-not-allowed transition-colors"
      >
        <Play size={14} />
        Run All ({totalMissions})
      </button>

      <button
        onClick={handleRunSelected}
        disabled={running || selected.size === 0}
        className="inline-flex items-center gap-1.5 px-3.5 py-1.5 text-sm font-medium rounded-lg bg-blue-600 text-white hover:bg-blue-500 disabled:opacity-30 disabled:cursor-not-allowed transition-colors"
      >
        <CheckCircle2 size={14} />
        Run Selected ({selected.size})
      </button>

      <button
        onClick={() => skipMission()}
        disabled={!running}
        className="inline-flex items-center gap-1.5 px-3.5 py-1.5 text-sm font-medium rounded-lg bg-amber-600 text-white hover:bg-amber-500 disabled:opacity-30 disabled:cursor-not-allowed transition-colors"
      >
        <SkipForward size={14} />
        Skip
      </button>

      <button
        onClick={() => stopEval()}
        disabled={!running}
        className="inline-flex items-center gap-1.5 px-3.5 py-1.5 text-sm font-medium rounded-lg bg-red-600 text-white hover:bg-red-500 disabled:opacity-30 disabled:cursor-not-allowed transition-colors"
      >
        <Square size={14} />
        Stop
      </button>
    </Card>
  );
}
