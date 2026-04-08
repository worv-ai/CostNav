"use client";
import useSWR from "swr";
import type { EvalStatus } from "@/types/mission";

const API = process.env.NEXT_PUBLIC_API_URL || "http://localhost:8100";

const fetcher = (url: string) => fetch(url).then((r) => r.json());

export function useMissionStatus(interval = 1000) {
  return useSWR<EvalStatus>(`${API}/api/status`, fetcher, {
    refreshInterval: interval,
    revalidateOnFocus: false,
  });
}
