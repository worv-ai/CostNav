"use client";
import { useCallback, useMemo, useRef, useState } from "react";
import { Map, SlidersHorizontal } from "lucide-react";
import type { MapMeta, Mission, Obstacle } from "@/types/mission";
import Card from "@/components/Card";
import Badge from "@/components/Badge";

const OBSTACLE_SIZE = 25;

const obstacleStyle: Record<string, { fill: string; stroke: string; label: string }> = {
  common: { fill: "#facc15", stroke: "#a16207", label: "Common" },
  minor: { fill: "#fb923c", stroke: "#c2410c", label: "Minor" },
  major: { fill: "#ef4444", stroke: "#991b1b", label: "Major" },
};

interface Props {
  missions: Mission[];
  mapMeta: MapMeta;
  selectedMission: number | null;
  onSelectMission: (id: number | null) => void;
}

function worldToPx(
  wx: number,
  wy: number,
  meta: MapMeta
): { px: number; py: number } {
  const px = (wx - meta.origin[0]) / meta.resolution;
  const py = meta.height - (wy - meta.origin[1]) / meta.resolution;
  return { px, py };
}

export default function MissionMap({
  missions,
  mapMeta,
  selectedMission,
  onSelectMission,
}: Props) {
  const containerRef = useRef<HTMLDivElement>(null);
  const [tooltip, setTooltip] = useState<{
    x: number;
    y: number;
    content: string;
  } | null>(null);
  const [strokeWidth, setStrokeWidth] = useState(15);

  const API = process.env.NEXT_PUBLIC_API_URL || "http://localhost:8100";

  const mission = useMemo(
    () => missions.find((m) => m.id === selectedMission) ?? null,
    [missions, selectedMission]
  );

  const missionOptions = useMemo(() => {
    return missions.filter(
      (m) => m.difficulty === "easy" || m.difficulty === "hard"
    );
  }, [missions]);

  const pathPoints = useMemo(() => {
    if (!mission) return "";
    return mission.waypoints
      .map((wp) => {
        const { px, py } = worldToPx(wp[0], wp[1], mapMeta);
        return `${px},${py}`;
      })
      .join(" ");
  }, [mission, mapMeta]);

  const startPx = useMemo(() => {
    if (!mission) return null;
    return worldToPx(mission.start.x, mission.start.y, mapMeta);
  }, [mission, mapMeta]);

  const goalPx = useMemo(() => {
    if (!mission) return null;
    return worldToPx(mission.goal.x, mission.goal.y, mapMeta);
  }, [mission, mapMeta]);

  const obstaclePxList = useMemo(() => {
    if (!mission?.obstacles) return [];
    return mission.obstacles.map((obs) => ({
      obs,
      ...worldToPx(obs.x, obs.y, mapMeta),
    }));
  }, [mission, mapMeta]);

  const handleObstacleEnter = useCallback(
    (e: React.MouseEvent, obs: Obstacle) => {
      const rect = containerRef.current?.getBoundingClientRect();
      if (!rect) return;
      const name = obs.usd_path.split("/").pop()?.replace(".usd", "") ?? "";
      setTooltip({
        x: e.clientX - rect.left + 12,
        y: e.clientY - rect.top - 8,
        content: [
          name,
          `type: ${obs.type}`,
          `pos: (${obs.x.toFixed(1)}, ${obs.y.toFixed(1)})`,
          `rotation: ${obs.rotation.toFixed(0)}deg`,
        ].join("\n"),
      });
    },
    []
  );

  const handleObstacleLeave = useCallback(() => setTooltip(null), []);

  return (
    <Card padding={false}>
      {/* Controls */}
      <div className="px-4 py-3 border-b border-[var(--border)] flex items-center gap-3 flex-wrap">
        <div className="flex items-center gap-2">
          <Map size={14} className="text-[var(--text-muted)]" />
          <select
            value={selectedMission ?? ""}
            onChange={(e) => {
              const v = e.target.value;
              onSelectMission(v === "" ? null : Number(v));
            }}
            className="px-2.5 py-1.5 text-sm rounded-lg bg-[var(--surface-overlay)] border border-[var(--border)] text-[var(--text-primary)] outline-none focus:ring-2 focus:ring-blue-500/30"
          >
            <option value="">Select mission...</option>
            {missionOptions.map((m) => (
              <option key={m.id} value={m.id}>
                #{m.id + 1} ({m.difficulty})
                {m.obstacles && m.obstacles.length > 0
                  ? ` - ${m.obstacles.length} obstacles`
                  : ""}
              </option>
            ))}
          </select>
        </div>

        <div className="flex items-center gap-2">
          <SlidersHorizontal size={12} className="text-[var(--text-muted)]" />
          <input
            type="range"
            min={1}
            max={30}
            step={1}
            value={strokeWidth}
            onChange={(e) => setStrokeWidth(Number(e.target.value))}
            className="w-20 h-1 accent-blue-500"
          />
        </div>

        {mission && (
          <div className="flex items-center gap-2">
            <Badge variant={mission.difficulty === "hard" ? "danger" : mission.difficulty === "easy" ? "success" : "neutral"}>
              #{mission.id + 1} {mission.difficulty}
            </Badge>
            <span className="text-xs text-[var(--text-muted)]">
              {mission.waypoints.length} waypoints
              {mission.obstacles ? `, ${mission.obstacles.length} obstacles` : ""}
            </span>
          </div>
        )}
      </div>

      {/* Map */}
      <div
        ref={containerRef}
        className="relative bg-[var(--surface)]"
      >
        <svg
          viewBox={`0 0 ${mapMeta.width} ${mapMeta.height}`}
          className="w-full h-auto"
        >
          <image
            href={`${API}${mapMeta.image_url}`}
            x={0}
            y={0}
            width={mapMeta.width}
            height={mapMeta.height}
          />

          {/* Path with glow */}
          {pathPoints && (
            <>
              <polyline
                points={pathPoints}
                fill="none"
                stroke="#3b82f6"
                strokeWidth={strokeWidth + 6}
                strokeLinejoin="round"
                strokeLinecap="round"
                opacity={0.2}
              />
              <polyline
                points={pathPoints}
                fill="none"
                stroke="#3b82f6"
                strokeWidth={strokeWidth}
                strokeLinejoin="round"
                strokeLinecap="round"
                opacity={0.85}
              />
            </>
          )}

          {/* Start: green circle with play triangle */}
          {startPx && (
            <g transform={`translate(${startPx.px}, ${startPx.py})`}>
              <circle r={24} fill="#22c55e" opacity={0.2} />
              <circle r={14} fill="#22c55e" stroke="white" strokeWidth={3} />
              <polygon points="-4,-6 -4,6 6,0" fill="white" />
            </g>
          )}

          {/* Goal: red circle with flag */}
          {goalPx && (
            <g transform={`translate(${goalPx.px}, ${goalPx.py})`}>
              <circle r={24} fill="#ef4444" opacity={0.2} />
              <circle r={14} fill="#ef4444" stroke="white" strokeWidth={3} />
              <rect x={-2} y={-8} width={2} height={16} fill="white" rx={1} />
              <polygon points="0,-8 10,-4 0,0" fill="white" />
            </g>
          )}

          {/* Obstacles with type-specific icons */}
          {obstaclePxList.map(({ obs, px, py }, i) => {
            const style = obstacleStyle[obs.type] || obstacleStyle.common;
            return (
              <g
                key={i}
                transform={`translate(${px}, ${py})`}
                className="cursor-pointer"
                onMouseEnter={(e) => handleObstacleEnter(e, obs)}
                onMouseLeave={handleObstacleLeave}
              >
                <rect
                  x={-OBSTACLE_SIZE}
                  y={-OBSTACLE_SIZE + 2}
                  width={OBSTACLE_SIZE * 2}
                  height={OBSTACLE_SIZE * 2}
                  rx={4}
                  fill="black"
                  opacity={0.2}
                />
                <rect
                  x={-OBSTACLE_SIZE}
                  y={-OBSTACLE_SIZE}
                  width={OBSTACLE_SIZE * 2}
                  height={OBSTACLE_SIZE * 2}
                  rx={4}
                  fill={style.fill}
                  stroke={style.stroke}
                  strokeWidth={2}
                  opacity={0.9}
                />
                {/* Type icon */}
                {obs.type === "major" && (
                  <polygon points="0,-10 -9,8 9,8" fill={style.stroke} opacity={0.8} />
                )}
                {obs.type === "minor" && (
                  <polygon points="0,-8 8,0 0,8 -8,0" fill={style.stroke} opacity={0.6} />
                )}
                {obs.type === "common" && (
                  <circle r={6} fill={style.stroke} opacity={0.5} />
                )}
              </g>
            );
          })}
        </svg>

        {/* Tooltip */}
        {tooltip && (
          <div
            className="absolute z-20 px-3 py-2 text-xs rounded-lg shadow-xl whitespace-pre pointer-events-none bg-[var(--surface-raised)] border border-[var(--border)] text-[var(--text-primary)]"
            style={{ left: tooltip.x, top: tooltip.y }}
          >
            {tooltip.content}
          </div>
        )}

        {/* Legend */}
        {mission && (
          <div className="absolute bottom-3 right-3 px-3 py-2.5 rounded-lg text-xs space-y-1.5 bg-[var(--surface-raised)]/90 backdrop-blur border border-[var(--border)] shadow-lg">
            <div className="flex items-center gap-2">
              <span className="w-3 h-3 rounded-full bg-emerald-500 ring-2 ring-emerald-500/30" />
              <span className="text-[var(--text-secondary)]">Start</span>
            </div>
            <div className="flex items-center gap-2">
              <span className="w-3 h-3 rounded-full bg-red-500 ring-2 ring-red-500/30" />
              <span className="text-[var(--text-secondary)]">Goal</span>
            </div>
            <div className="flex items-center gap-2">
              <span className="w-5 h-0.5 rounded bg-blue-500" />
              <span className="text-[var(--text-secondary)]">Path</span>
            </div>
            {mission.obstacles && mission.obstacles.length > 0 && (
              <>
                <div className="border-t border-[var(--border-subtle)] my-1" />
                {Object.entries(obstacleStyle).map(([type, style]) => (
                  <div key={type} className="flex items-center gap-2">
                    <span
                      className="w-3 h-3 rounded-sm"
                      style={{ background: style.fill, border: `1px solid ${style.stroke}` }}
                    />
                    <span className="text-[var(--text-secondary)]">{style.label}</span>
                  </div>
                ))}
              </>
            )}
          </div>
        )}
      </div>
    </Card>
  );
}
