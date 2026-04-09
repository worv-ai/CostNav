export interface Obstacle {
  usd_path: string;
  type: string;
  x: number;
  y: number;
  z: number;
  rotation: number;
}

export interface Mission {
  id: number;
  difficulty: string;
  start: { x: number; y: number; z?: number; heading?: number };
  goal: { x: number; y: number; z?: number };
  waypoints: number[][];
  obstacles?: Obstacle[];
}

export interface MapMeta {
  resolution: number;
  origin: [number, number, number];
  width: number;
  height: number;
  image_url: string;
}

export interface CostBreakdown {
  revenue: number;
  electricity: number;
  injury: number;
  property_damage: number;
  service_comp: number;
  total_opex: number;
  profit: number;
}

export interface MissionResult {
  mission_id: number;
  mission_number: number;
  difficulty: string | null;
  result: string;
  result_reason: string | null;
  in_progress: boolean;
  distance_to_goal: number;
  traveled_distance: number;
  elapsed_time: number;
  avg_velocity: number;
  avg_mech_power: number;
  total_contact_count: number;
  total_impulse: number;
  people_contact_count: number;
  property_contacts: Record<string, number>;
  delta_v_count: number;
  injury_cost: number;
  food: {
    enabled: boolean;
    initial_pieces: number;
    final_pieces: number;
    loss_fraction: number;
    spoiled: boolean;
  };
  cost?: CostBreakdown;
}

export interface EvalStatus {
  running: boolean;
  current_mission_id: number | null;
  completed: number;
  total: number;
  results: MissionResult[];
  current_status: MissionResult | null;
}
