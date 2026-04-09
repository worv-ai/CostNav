"use client";
import { useCallback, useState } from "react";
import useSWR from "swr";
import {
  Navigation,
  Layers,
} from "lucide-react";
import MissionTable from "@/components/MissionTable";
import MissionMap from "@/components/MissionMap";
import EvalControls from "@/components/EvalControls";
import LiveStatus from "@/components/LiveStatus";
import ThemeToggle from "@/components/ThemeToggle";
import Badge from "@/components/Badge";
import { useMissionStatus } from "@/hooks/useMissionStatus";
import type { MapMeta, Mission } from "@/types/mission";

const API = process.env.NEXT_PUBLIC_API_URL || "http://localhost:8100";
const fetcher = (url: string) => fetch(url).then((r) => r.json());

export default function Home() {
  const { data: missionsData } = useSWR<{ missions: Mission[] }>(
    `${API}/api/missions`,
    fetcher
  );
  const { data: mapMeta } = useSWR<MapMeta>(`${API}/api/map/meta`, fetcher);
  const { data: status } = useMissionStatus(1000);
  const [selected, setSelected] = useState<Set<number>>(new Set());
  const [timeout, setTimeout] = useState(241);
  const [mapMission, setMapMission] = useState<number | null>(null);

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
    <div className="min-h-screen">
      {/* Header */}
      <header className="sticky top-0 z-30 border-b border-[var(--border)] bg-[var(--surface)]/80 backdrop-blur-xl">
        <div className="max-w-[1600px] mx-auto px-6 h-14 flex items-center justify-between">
          <div className="flex items-center gap-3">
            <div className="flex items-center justify-center w-8 h-8 rounded-lg bg-blue-500/10">
              <Navigation size={16} className="text-blue-500" />
            </div>
            <h1 className="text-base font-semibold tracking-tight">
              CostNav
            </h1>
            <Badge variant="info" dot>
              {missions.length} missions
            </Badge>
          </div>
          <div className="flex items-center gap-2">
            {status?.running && (
              <Badge variant="success" dot>
                Running
              </Badge>
            )}
            <ThemeToggle />
          </div>
        </div>
      </header>

      {/* Main content */}
      <main className="max-w-[1600px] mx-auto px-6 py-5 space-y-5">
        {/* Controls bar */}
        <EvalControls
          running={status?.running || false}
          selected={selected}
          totalMissions={missions.length}
          timeout={timeout}
          onTimeoutChange={setTimeout}
        />

        {/* Map */}
        {mapMeta && mapMeta.width && (
          <MissionMap
            missions={missions}
            mapMeta={mapMeta}
            selectedMission={mapMission}
            onSelectMission={setMapMission}
          />
        )}

        {/* Table + Status */}
        <div className="grid grid-cols-1 lg:grid-cols-3 gap-5">
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
              mapMission={mapMission}
              onSelectMapMission={setMapMission}
            />
          </div>
          <div>
            <LiveStatus status={status} />
          </div>
        </div>
      </main>
    </div>
  );
}
