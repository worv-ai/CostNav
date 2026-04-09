"use client";
import { useState } from "react";
import { Play, Loader2, MapPin } from "lucide-react";
import { clsx } from "clsx";
import type { Mission, MissionResult } from "@/types/mission";
import { startMission } from "@/lib/api";
import Card from "@/components/Card";
import Badge from "@/components/Badge";

const diffBadge: Record<string, { variant: "neutral" | "success" | "danger"; label: string }> = {
  none: { variant: "neutral", label: "None" },
  easy: { variant: "success", label: "Easy" },
  hard: { variant: "danger", label: "Hard" },
};

const statusBadge: Record<string, { variant: "success" | "danger" | "warning" | "neutral"; label: string }> = {
  success: { variant: "success", label: "Success" },
  failure_timeout: { variant: "danger", label: "Timeout" },
  failure_physicalassistance: { variant: "danger", label: "Phys. Assist" },
  failure_foodspoiled: { variant: "warning", label: "Spoiled" },
  skipped: { variant: "neutral", label: "Skipped" },
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
  mapMission: number | null;
  onSelectMapMission: (id: number | null) => void;
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
  mapMission,
  onSelectMapMission,
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
    <Card padding={false} className="overflow-hidden">
      <div className="overflow-auto max-h-[70vh]">
        <table className="w-full text-sm">
          <thead className="bg-[var(--surface-overlay)] sticky top-0 z-10">
            <tr className="text-left text-xs text-[var(--text-muted)] uppercase tracking-wider">
              <th className="px-3 py-2.5 w-8">
                <input
                  type="checkbox"
                  checked={selected.size === missions.length && missions.length > 0}
                  onChange={() =>
                    selected.size === missions.length ? onDeselectAll() : onSelectAll()
                  }
                  className="rounded"
                />
              </th>
              <th className="px-3 py-2.5 w-12">#</th>
              <th className="px-3 py-2.5 w-20">Diff</th>
              <th className="px-3 py-2.5">Start</th>
              <th className="px-3 py-2.5">Goal</th>
              <th className="px-3 py-2.5 w-28">Status</th>
              <th className="px-3 py-2.5 w-16">Time</th>
              <th className="px-3 py-2.5 w-16">Dist</th>
              <th className="px-3 py-2.5 w-24"></th>
            </tr>
          </thead>
          <tbody className="divide-y divide-[var(--border-subtle)]">
            {missions.map((m) => {
              const r = resultMap.get(m.id);
              const isCurrent = currentId === m.id;
              const isMapSelected = mapMission === m.id;
              const diff = diffBadge[m.difficulty] || diffBadge.none;
              const st = r ? statusBadge[r.result] : null;

              return (
                <tr
                  key={m.id}
                  className={clsx(
                    "transition-colors cursor-pointer",
                    isMapSelected && "bg-purple-500/5 dark:bg-purple-500/10",
                    isCurrent && !isMapSelected && "bg-blue-500/5 dark:bg-blue-500/10",
                    !isCurrent && !isMapSelected && "hover:bg-[var(--surface-overlay)]"
                  )}
                  onClick={() => onSelectMapMission(isMapSelected ? null : m.id)}
                >
                  <td className="px-3 py-2" onClick={(e) => e.stopPropagation()}>
                    <input
                      type="checkbox"
                      checked={selected.has(m.id)}
                      onChange={() => onToggleSelect(m.id)}
                      className="rounded"
                    />
                  </td>
                  <td className="px-3 py-2 font-mono text-[var(--text-secondary)]">
                    {m.id + 1}
                  </td>
                  <td className="px-3 py-2">
                    <Badge variant={diff.variant}>{diff.label}</Badge>
                  </td>
                  <td className="px-3 py-2 font-mono text-xs text-[var(--text-secondary)]">
                    {m.start.x.toFixed(1)}, {m.start.y.toFixed(1)}
                  </td>
                  <td className="px-3 py-2 font-mono text-xs text-[var(--text-secondary)]">
                    {m.goal.x.toFixed(1)}, {m.goal.y.toFixed(1)}
                  </td>
                  <td className="px-3 py-2">
                    {isCurrent && !r ? (
                      <Badge variant="info" dot>
                        Running
                      </Badge>
                    ) : st ? (
                      <Badge variant={st.variant}>{st.label}</Badge>
                    ) : (
                      <span className="text-[var(--text-muted)]">-</span>
                    )}
                  </td>
                  <td className="px-3 py-2 font-mono text-xs text-[var(--text-secondary)]">
                    {r ? `${r.elapsed_time.toFixed(1)}s` : "-"}
                  </td>
                  <td className="px-3 py-2 font-mono text-xs text-[var(--text-secondary)]">
                    {r ? `${r.traveled_distance.toFixed(1)}m` : "-"}
                  </td>
                  <td className="px-3 py-2" onClick={(e) => e.stopPropagation()}>
                    <div className="flex items-center gap-1">
                      <button
                        onClick={() => handleRun(m.id)}
                        disabled={starting !== null}
                        className="inline-flex items-center gap-1 px-2.5 py-1 text-xs font-medium rounded-md bg-blue-600 text-white hover:bg-blue-500 disabled:opacity-30 transition-colors"
                      >
                        {starting === m.id ? (
                          <Loader2 size={12} className="animate-spin" />
                        ) : (
                          <Play size={12} />
                        )}
                        Run
                      </button>
                      {isMapSelected && (
                        <MapPin size={14} className="text-purple-500" />
                      )}
                    </div>
                  </td>
                </tr>
              );
            })}
          </tbody>
        </table>
      </div>
    </Card>
  );
}
