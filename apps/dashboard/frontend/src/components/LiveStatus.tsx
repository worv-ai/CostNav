"use client";
import type { EvalStatus } from "@/types/mission";

interface Props {
  status: EvalStatus | undefined;
}

export default function LiveStatus({ status }: Props) {
  if (!status) {
    return (
      <div className="p-4 rounded border border-gray-800 bg-gray-900">
        <p className="text-gray-500 text-sm">Connecting to API...</p>
      </div>
    );
  }

  const { running, current_mission_id, completed, total, current_status } =
    status;
  const progress = total > 0 ? (completed / total) * 100 : 0;
  const cs = current_status;

  return (
    <div className="p-4 rounded border border-gray-800 bg-gray-900 space-y-3">
      <div className="flex justify-between items-center">
        <h3 className="font-semibold text-sm">Live Status</h3>
        <span
          className={`text-xs px-2 py-0.5 rounded ${
            running ? "bg-green-800 text-green-300" : "bg-gray-800 text-gray-400"
          }`}
        >
          {running ? "Running" : "Idle"}
        </span>
      </div>

      {total > 0 && (
        <>
          <div>
            <div className="flex justify-between text-xs text-gray-400 mb-1">
              <span>
                Progress: {completed}/{total}
              </span>
              <span>{progress.toFixed(0)}%</span>
            </div>
            <div className="h-2 bg-gray-800 rounded overflow-hidden">
              <div
                className="h-full bg-blue-600 transition-all"
                style={{ width: `${progress}%` }}
              />
            </div>
          </div>
        </>
      )}

      {current_mission_id !== null && (
        <div className="text-sm">
          <span className="text-gray-400">Current Mission: </span>
          <span className="font-mono">{current_mission_id + 1}</span>
        </div>
      )}

      {cs && cs.in_progress && (
        <div className="grid grid-cols-2 gap-2 text-xs">
          <Stat label="Distance to Goal" value={`${cs.distance_to_goal.toFixed(1)}m`} />
          <Stat label="Traveled" value={`${cs.traveled_distance.toFixed(1)}m`} />
          <Stat label="Elapsed" value={`${cs.elapsed_time.toFixed(1)}s`} />
          <Stat label="Velocity" value={`${cs.avg_velocity.toFixed(2)}m/s`} />
          <Stat label="Contacts" value={String(cs.total_contact_count)} />
          <Stat label="People Contacts" value={String(cs.people_contact_count)} />
          <Stat label="Impulse" value={`${cs.total_impulse.toFixed(1)}N*s`} />
          <Stat label="Injury Cost" value={`${cs.injury_cost.toFixed(2)}`} />
        </div>
      )}

      {/* Summary after completion */}
      {!running && status.results.length > 0 && (
        <Summary results={status.results} />
      )}
    </div>
  );
}

function Stat({ label, value }: { label: string; value: string }) {
  return (
    <div>
      <div className="text-gray-500">{label}</div>
      <div className="font-mono">{value}</div>
    </div>
  );
}

function Summary({ results }: { results: import("@/types/mission").MissionResult[] }) {
  const completed = results.filter((r) => r.result !== "pending");
  const successes = completed.filter((r) => r.result === "success");
  const rate = completed.length > 0 ? (successes.length / completed.length) * 100 : 0;
  const avgTime =
    completed.length > 0
      ? completed.reduce((s, r) => s + r.elapsed_time, 0) / completed.length
      : 0;
  const avgDist =
    completed.length > 0
      ? completed.reduce((s, r) => s + r.traveled_distance, 0) / completed.length
      : 0;

  return (
    <div className="pt-3 border-t border-gray-800 space-y-1">
      <h4 className="text-xs font-semibold text-gray-400">Summary</h4>
      <div className="grid grid-cols-2 gap-2 text-xs">
        <Stat label="Success Rate" value={`${rate.toFixed(1)}%`} />
        <Stat
          label="Completed"
          value={`${successes.length}/${completed.length}`}
        />
        <Stat label="Avg Time" value={`${avgTime.toFixed(1)}s`} />
        <Stat label="Avg Distance" value={`${avgDist.toFixed(1)}m`} />
      </div>
    </div>
  );
}
