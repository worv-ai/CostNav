"use client";
import {
  Activity,
  Navigation,
  Route,
  Timer,
  Gauge,
  ShieldAlert,
  Users,
  Zap,
  DollarSign,
  TrendingUp,
  TrendingDown,
  CircleDot,
} from "lucide-react";
import type { CostBreakdown, EvalStatus } from "@/types/mission";
import Card, { CardHeader, CardTitle } from "@/components/Card";
import Badge from "@/components/Badge";

interface Props {
  status: EvalStatus | undefined;
}

export default function LiveStatus({ status }: Props) {
  if (!status) {
    return (
      <Card>
        <div className="flex items-center gap-2 text-[var(--text-muted)] text-sm">
          <Activity size={14} className="animate-pulse" />
          Connecting to API...
        </div>
      </Card>
    );
  }

  const { running, current_mission_id, completed, total, current_status } =
    status;
  const progress = total > 0 ? (completed / total) * 100 : 0;
  const cs = current_status;

  return (
    <Card className="space-y-4">
      <CardHeader>
        <CardTitle>Live Status</CardTitle>
        {running ? (
          <Badge variant="success" dot>Running</Badge>
        ) : (
          <Badge variant="neutral">Idle</Badge>
        )}
      </CardHeader>

      {total > 0 && (
        <div>
          <div className="flex justify-between text-xs text-[var(--text-secondary)] mb-1.5">
            <span>
              {completed} / {total} missions
            </span>
            <span className="font-mono">{progress.toFixed(0)}%</span>
          </div>
          <div className="h-2 bg-[var(--surface-overlay)] rounded-full overflow-hidden">
            <div
              className="h-full bg-gradient-to-r from-blue-600 to-blue-400 rounded-full transition-all duration-500"
              style={{ width: `${progress}%` }}
            />
          </div>
        </div>
      )}

      {current_mission_id !== null && (
        <div className="flex items-center gap-2 text-sm">
          <CircleDot size={14} className="text-blue-500" />
          <span className="text-[var(--text-secondary)]">Mission</span>
          <span className="font-mono font-semibold">{current_mission_id + 1}</span>
        </div>
      )}

      {cs && cs.in_progress && (
        <>
          <div className="grid grid-cols-2 gap-3">
            <MetricCard icon={Navigation} label="To Goal" value={`${cs.distance_to_goal.toFixed(1)}m`} />
            <MetricCard icon={Route} label="Traveled" value={`${cs.traveled_distance.toFixed(1)}m`} />
            <MetricCard icon={Timer} label="Elapsed" value={`${cs.elapsed_time.toFixed(1)}s`} />
            <MetricCard icon={Gauge} label="Velocity" value={`${cs.avg_velocity.toFixed(2)}m/s`} />
            <MetricCard icon={ShieldAlert} label="Contacts" value={String(cs.total_contact_count)} accent={cs.total_contact_count > 0 ? "danger" : undefined} />
            <MetricCard icon={Users} label="People" value={String(cs.people_contact_count)} accent={cs.people_contact_count > 0 ? "danger" : undefined} />
            <MetricCard icon={Zap} label="Impulse" value={`${cs.total_impulse.toFixed(1)}`} />
            <MetricCard icon={DollarSign} label="Injury $" value={`$${cs.injury_cost.toFixed(2)}`} accent={cs.injury_cost > 0 ? "danger" : undefined} />
          </div>

          {cs.cost && <CostPanel cost={cs.cost} />}
        </>
      )}

      {!running && status.results.length > 0 && (
        <Summary results={status.results} />
      )}
    </Card>
  );
}

function MetricCard({
  icon: Icon,
  label,
  value,
  accent,
}: {
  icon: React.ComponentType<{ size?: number; className?: string }>;
  label: string;
  value: string;
  accent?: "danger" | "success";
}) {
  const textColor = accent === "danger"
    ? "text-red-500"
    : accent === "success"
      ? "text-emerald-500"
      : "text-[var(--text-primary)]";

  return (
    <div className="flex items-start gap-2 p-2.5 rounded-lg bg-[var(--surface-overlay)]">
      <Icon size={14} className="text-[var(--text-muted)] mt-0.5 shrink-0" />
      <div className="min-w-0">
        <div className="text-[10px] uppercase tracking-wider text-[var(--text-muted)]">{label}</div>
        <div className={`text-sm font-mono font-medium ${textColor}`}>{value}</div>
      </div>
    </div>
  );
}

function CostPanel({ cost }: { cost: CostBreakdown }) {
  const isProfit = cost.profit >= 0;

  return (
    <div className="pt-3 border-t border-[var(--border)] space-y-2">
      <div className="flex items-center gap-1.5 text-xs font-semibold text-[var(--text-secondary)]">
        <DollarSign size={12} />
        Cost Breakdown ($/run)
      </div>
      <div className="grid grid-cols-2 gap-2 text-xs">
        <CostRow label="Revenue" value={`+$${cost.revenue.toFixed(2)}`} positive />
        <CostRow label="Electricity" value={`-$${cost.electricity.toFixed(4)}`} />
        <CostRow label="Injury" value={`-$${cost.injury.toFixed(2)}`} />
        <CostRow label="Property" value={`-$${cost.property_damage.toFixed(2)}`} />
        <CostRow label="Service" value={`-$${cost.service_comp.toFixed(2)}`} />
        <CostRow label="OPEX" value={`-$${cost.total_opex.toFixed(2)}`} />
      </div>
      <div className="flex items-center justify-between pt-2 border-t border-[var(--border-subtle)]">
        <span className="text-xs font-semibold text-[var(--text-secondary)]">Profit</span>
        <div className={`flex items-center gap-1 text-sm font-bold font-mono ${isProfit ? "text-emerald-500" : "text-red-500"}`}>
          {isProfit ? <TrendingUp size={14} /> : <TrendingDown size={14} />}
          {isProfit ? "+" : ""}${cost.profit.toFixed(2)}
        </div>
      </div>
    </div>
  );
}

function CostRow({ label, value, positive }: { label: string; value: string; positive?: boolean }) {
  return (
    <div className="flex justify-between items-center px-2 py-1 rounded bg-[var(--surface-overlay)]">
      <span className="text-[var(--text-muted)]">{label}</span>
      <span className={`font-mono ${positive ? "text-emerald-500" : "text-[var(--text-secondary)]"}`}>
        {value}
      </span>
    </div>
  );
}

function Summary({
  results,
}: {
  results: import("@/types/mission").MissionResult[];
}) {
  const completed = results.filter((r) => r.result !== "pending");
  const successes = completed.filter((r) => r.result === "success");
  const rate = completed.length > 0 ? (successes.length / completed.length) * 100 : 0;
  const avgTime = completed.length > 0
    ? completed.reduce((s, r) => s + r.elapsed_time, 0) / completed.length
    : 0;
  const avgDist = completed.length > 0
    ? completed.reduce((s, r) => s + r.traveled_distance, 0) / completed.length
    : 0;
  const avgProfit = completed.length > 0
    ? completed.reduce((s, r) => s + (r.cost?.profit ?? 0), 0) / completed.length
    : 0;

  return (
    <div className="pt-3 border-t border-[var(--border)] space-y-2">
      <div className="text-xs font-semibold text-[var(--text-secondary)]">Summary</div>
      <div className="grid grid-cols-2 gap-2">
        <MetricCard icon={Activity} label="Success" value={`${rate.toFixed(1)}%`} accent={rate > 50 ? "success" : "danger"} />
        <MetricCard icon={Timer} label="Avg Time" value={`${avgTime.toFixed(1)}s`} />
        <MetricCard icon={Route} label="Avg Dist" value={`${avgDist.toFixed(1)}m`} />
        <MetricCard
          icon={DollarSign}
          label="Avg Profit"
          value={`${avgProfit >= 0 ? "+" : ""}$${avgProfit.toFixed(2)}`}
          accent={avgProfit >= 0 ? "success" : "danger"}
        />
      </div>
    </div>
  );
}
