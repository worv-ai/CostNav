"use client";
import { useCallback, useEffect, useState } from "react";
import useSWR from "swr";
import MissionTable from "@/components/MissionTable";
import EvalControls from "@/components/EvalControls";
import LiveStatus from "@/components/LiveStatus";
import { useMissionStatus } from "@/hooks/useMissionStatus";
import type { Mission } from "@/types/mission";

const API = process.env.NEXT_PUBLIC_API_URL || "http://localhost:8100";
const fetcher = (url: string) => fetch(url).then((r) => r.json());

export default function Home() {
  const { data: missionsData } = useSWR<{ missions: Mission[] }>(
    `${API}/api/missions`,
    fetcher
  );
  const { data: status } = useMissionStatus(1000);
  const [selected, setSelected] = useState<Set<number>>(new Set());
  const [timeout, setTimeout] = useState(241);

  const missions = missionsData?.missions || [];

  const toggleSelect = useCallback(
    (id: number) =>
      setSelected((prev) => {
        const next = new Set(prev);
        next.has(id) ? next.delete(id) : next.add(id);
        return next;
      }),
    []
  );

  const selectAll = useCallback(
    () => setSelected(new Set(missions.map((m) => m.id))),
    [missions]
  );

  const deselectAll = useCallback(() => setSelected(new Set()), []);

  return (
    <div className="max-w-7xl mx-auto p-4 space-y-4">
      <header className="flex items-center justify-between">
        <h1 className="text-xl font-bold">CostNav Mission Control</h1>
        <span className="text-xs text-gray-500">
          {missions.length} missions loaded
        </span>
      </header>

      <EvalControls
        running={status?.running || false}
        selected={selected}
        totalMissions={missions.length}
        timeout={timeout}
        onTimeoutChange={setTimeout}
      />

      <div className="grid grid-cols-1 lg:grid-cols-3 gap-4">
        <div className="lg:col-span-2">
          <MissionTable
            missions={missions}
            results={status?.results || []}
            currentId={status?.current_mission_id ?? null}
            timeout={timeout}
            selected={selected}
            onToggleSelect={toggleSelect}
            onSelectAll={selectAll}
            onDeselectAll={deselectAll}
          />
        </div>
        <div>
          <LiveStatus status={status} />
        </div>
      </div>
    </div>
  );
}
