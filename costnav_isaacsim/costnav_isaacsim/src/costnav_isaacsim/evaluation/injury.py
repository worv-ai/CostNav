"""Injury cost calculation helpers."""

from __future__ import annotations

import math

# MAIS logistic regression coefficients from crash data
# Format: (intercept, slope) for P(MAIS >= level) = 1 / (1 + exp(-(intercept + slope * delta_v_mph)))
_MAIS_COEFFICIENTS = {
    "all": {
        "mais_1": (-1.3925, 0.0815),
        "mais_2": (-5.1331, 0.1479),
        "mais_3": (-6.9540, 0.1637),
        "mais_4": (-8.2070, 0.1564),
        "mais_5": (-8.7927, 0.1598),
        "fatality": (-8.9819, 0.1603),
    },
}
_MAIS_LEVELS = ("mais_0", "mais_1", "mais_2", "mais_3", "mais_4", "mais_5", "fatality")

MPS_TO_MPH = 2.23694

# Injury cost adjustment factor to scale expected costs
_INJURY_COST_ADJUSTMENT_FACTOR = 0.0110


def _logistic_probability(intercept: float, slope: float, delta_v_mph: float) -> float:
    exponent = intercept + slope * delta_v_mph
    return math.exp(exponent) / (1.0 + math.exp(exponent))


def compute_mais_probabilities(config, delta_v_mps: float) -> dict:
    if delta_v_mps <= 0.0:
        return {level: 1.0 if level == "mais_0" else 0.0 for level in _MAIS_LEVELS}

    delta_v_mph = delta_v_mps * MPS_TO_MPH
    crash_mode = config.injury.crash_mode or "all"
    coefficients = _MAIS_COEFFICIENTS.get(crash_mode, _MAIS_COEFFICIENTS["all"])

    # Compute cumulative probabilities P(MAIS >= level)
    mais_1_plus = _logistic_probability(*coefficients["mais_1"], delta_v_mph)
    mais_2_plus = _logistic_probability(*coefficients["mais_2"], delta_v_mph)
    mais_3_plus = _logistic_probability(*coefficients["mais_3"], delta_v_mph)
    mais_4_plus = _logistic_probability(*coefficients["mais_4"], delta_v_mph)
    mais_5_plus = _logistic_probability(*coefficients["mais_5"], delta_v_mph)
    fatality = _logistic_probability(*coefficients["fatality"], delta_v_mph)

    # Ensure monotonicity
    mais_2_plus = min(mais_2_plus, mais_1_plus)
    mais_3_plus = min(mais_3_plus, mais_2_plus)
    mais_4_plus = min(mais_4_plus, mais_3_plus)
    mais_5_plus = min(mais_5_plus, mais_4_plus)
    fatality = min(fatality, mais_5_plus)

    # Convert cumulative to individual level probabilities
    return {
        "mais_0": 1.0 - mais_1_plus,
        "mais_1": mais_1_plus - mais_2_plus,
        "mais_2": mais_2_plus - mais_3_plus,
        "mais_3": mais_3_plus - mais_4_plus,
        "mais_4": mais_4_plus - mais_5_plus,
        "mais_5": mais_5_plus - fatality,
        "fatality": fatality,
    }


def compute_expected_injury_cost(config, probabilities: dict) -> float:
    costs = config.injury.costs
    raw_cost = (
        probabilities.get("mais_0", 0.0) * costs.mais_0
        + probabilities.get("mais_1", 0.0) * costs.mais_1
        + probabilities.get("mais_2", 0.0) * costs.mais_2
        + probabilities.get("mais_3", 0.0) * costs.mais_3
        + probabilities.get("mais_4", 0.0) * costs.mais_4
        + probabilities.get("mais_5", 0.0) * costs.mais_5
        + probabilities.get("fatality", 0.0) * costs.fatality
    )
    return raw_cost * _INJURY_COST_ADJUSTMENT_FACTOR


def process_collision_injury(state, config, impulse_amount: float, is_character_collision: bool):
    if not config.injury.enabled or not is_character_collision:
        return None

    robot_mass = config.injury.robot_mass
    delta_v_mps = impulse_amount / robot_mass
    state.delta_v_magnitudes_mps.append(delta_v_mps)

    probabilities = compute_mais_probabilities(config, delta_v_mps)
    injury_cost = compute_expected_injury_cost(config, probabilities)
    state.injury_costs.append(injury_cost)
    state.total_injury_cost += injury_cost

    return (delta_v_mps, injury_cost, state.total_injury_cost)
