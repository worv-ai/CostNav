"use client";
import { useState } from "react";
import type { Mission, MissionResult } from "@/types/mission";
import { startMission } from "@/lib/api";

const diffColor: Record<string, string> = {
  none: "text-gray-400",
  easy: "text-green-400",
  hard: "text-red-400",
};

const statusBg: Record<string, string> = {
  success: "bg-green-900/40",
  failure_timeout: "bg-red-900/40",
  failure_physicalassistance: "bg-red-900/40",
  failure_foodspoiled: "bg-orange-900/40",
  skipped: "bg-gray-800/40",
};

interface Props {
  missions: Mission[];
  results: MissionResult[];
  currentId: number | null;
  timeout: number;
  selected: Set<number>;
  onToggleSelect: (id: number) => void;
  onSelectAll: () => void;
  onDeselectAll: () => void;
}

export default function MissionTable({
  missions,
  results,
  currentId,
  timeout,
  selected,
  onToggleSelect,
  onSelectAll,
  onDeselectAll,
}: Props) {
  const [starting, setStarting] = useState<number | null>(null);
  const resultMap = new Map(results.map((r) => [r.mission_id, r]));

  const handleRun = async (id: number) => {
    setStarting(id);
    try {
      await startMission(id, timeout);
    } finally {
      setStarting(null);
    }
  };

  return (
    <div className="overflow-auto max-h-[70vh] rounded border border-gray-800">
      <table className="w-full text-sm">
        <thead className="bg-gray-900 sticky top-0 z-10">
          <tr className="text-left text-gray-400">
            <th className="px-2 py-2 w-8">
              <input
                type="checkbox"
                checked={selected.size === missions.length}
                onChange={() =>
                  selected.size === missions.length
                    ? onDeselectAll()
                    : onSelectAll()
                }
              />
            </th>
            <th className="px-2 py-2 w-12">#</th>
            <th className="px-2 py-2 w-16">Diff</th>
            <th className="px-2 py-2">Start (x, y)</th>
            <th className="px-2 py-2">Goal (x, y)</th>
            <th className="px-2 py-2 w-24">Status</th>
            <th className="px-2 py-2 w-16">Time</th>
            <th className="px-2 py-2 w-16">Dist</th>
            <th className="px-2 py-2 w-20"></th>
          </tr>
        </thead>
        <tbody>
          {missions.map((m) => {
            const r = resultMap.get(m.id);
            const isCurrent = currentId === m.id;
            const bg = isCurrent
              ? "bg-blue-900/30"
              : r
                ? statusBg[r.result] || ""
                : "";

            return (
              <tr
                key={m.id}
                className={`border-t border-gray-800 hover:bg-gray-800/50 ${bg}`}
              >
                <td className="px-2 py-1.5">
                  <input
                    type="checkbox"
                    checked={selected.has(m.id)}
                    onChange={() => onToggleSelect(m.id)}
                  />
                </td>
                <td className="px-2 py-1.5 font-mono">{m.id + 1}</td>
                <td className={`px-2 py-1.5 ${diffColor[m.difficulty] || ""}`}>
                  {m.difficulty}
                </td>
                <td className="px-2 py-1.5 font-mono text-xs">
                  {m.start.x.toFixed(1)}, {m.start.y.toFixed(1)}
                </td>
                <td className="px-2 py-1.5 font-mono text-xs">
                  {m.goal.x.toFixed(1)}, {m.goal.y.toFixed(1)}
                </td>
                <td className="px-2 py-1.5 text-xs">
                  {isCurrent && !r ? (
                    <span className="text-blue-400">running...</span>
                  ) : r ? (
                    <span>{r.result}</span>
                  ) : (
                    <span className="text-gray-600">-</span>
                  )}
                </td>
                <td className="px-2 py-1.5 font-mono text-xs">
                  {r ? `${r.elapsed_time.toFixed(1)}s` : "-"}
                </td>
                <td className="px-2 py-1.5 font-mono text-xs">
                  {r ? `${r.traveled_distance.toFixed(1)}m` : "-"}
                </td>
                <td className="px-2 py-1.5">
                  <button
                    onClick={() => handleRun(m.id)}
                    disabled={starting !== null}
                    className="px-2 py-0.5 text-xs bg-blue-700 hover:bg-blue-600 rounded disabled:opacity-30"
                  >
                    {starting === m.id ? "..." : "Run"}
                  </button>
                </td>
              </tr>
            );
          })}
        </tbody>
      </table>
    </div>
  );
}
