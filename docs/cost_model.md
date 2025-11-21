# Cost Model and Business Metrics

This document explains CostNav's cost model, which evaluates navigation policies based on business metrics rather than just task success.

## Overview

Traditional robotics benchmarks focus on technical metrics:
- Success rate
- Path length
- Collision rate
- Computation time

CostNav adds **business-oriented metrics**:
- **SLA Compliance**: Meeting delivery time commitments
- **Operational Cost**: Energy, maintenance, repairs
- **Profitability**: Revenue minus costs
- **Break-even Time**: Time to recover initial investment

This enables evaluation of policies based on real-world deployment viability.

## Cost Model Components

### 1. Capital Expenditure (CapEx)

**Robot Hardware Cost**: Initial investment in robot platform

```python
robot_cost = $15,000  # Estimated COCO robot cost
operating_years = 5   # Expected lifetime
annual_capex = robot_cost / operating_years = $3,000/year
```

**Amortization**:
- Hardware cost spread over operating lifetime
- Accounts for depreciation
- Includes sensors, compute, chassis

### 2. Energy Costs

**Power Consumption Model**:

```python
def compute_navigation_energy_step(env):
    # Get robot parameters
    mass = 50.0  # kg (COCO robot mass)
    gravity = 9.81  # m/s²
    
    # Get current velocity
    velocity = norm(robot.root_lin_vel[:, :2])  # Planar velocity
    
    # Power = m * g * v (simplified model)
    power = mass * gravity * velocity  # Watts
    
    # Energy per step
    dt = 0.005  # Simulation timestep (5ms)
    energy = power * dt  # Joules
    
    return energy
```

**Cost Calculation**:

```python
# Accumulate energy over episode
total_energy_joules = sum(energy_per_step)
total_energy_kwh = total_energy_joules / 3.6e6

# Apply electricity rate
electricity_rate = $0.15 / kWh  # US average
energy_cost = total_energy_kwh * electricity_rate
```

**Typical Values**:
- Average power: 50-200 W (depending on speed)
- Energy per delivery: 0.01-0.05 kWh
- Cost per delivery: $0.0015-$0.0075

### 3. Maintenance Costs

**Collision-Based Maintenance**:

```python
def compute_maintenance_cost(episode):
    # Base maintenance (wear and tear)
    base_maintenance = $0.10 per delivery
    
    # Collision damage
    if collision_occurred:
        collision_cost = $5.00  # Average repair cost
    else:
        collision_cost = $0.00
    
    # Total maintenance
    maintenance_cost = base_maintenance + collision_cost
    
    return maintenance_cost
```

**Cost Breakdown**:
- **Routine maintenance**: $0.10 per delivery
  - Tire wear
  - Battery degradation
  - Software updates
  
- **Collision repairs**: $5.00 per incident
  - Body damage
  - Sensor recalibration
  - Downtime costs

**Industry Data**:
- Maintenance represents 33.7% of operating costs
- Collision avoidance is high-leverage improvement

### 4. Revenue Model

**Delivery Revenue**:

Based on food delivery industry data (DoorDash, Uber Eats):

```python
# Per-delivery revenue
revenue_per_delivery = $8.49  # DoorDash average
# or
revenue_per_delivery = $10.00  # Uber Eats average

# Baseline for robot delivery
robot_revenue_per_delivery = $9.00  # Competitive with human couriers
```

**SLA-Based Revenue**:

```python
def compute_revenue(episode):
    base_revenue = $9.00
    
    # SLA compliance bonus/penalty
    if delivery_time <= sla_threshold:
        # On-time delivery
        revenue = base_revenue
    elif delivery_time <= sla_threshold + 10_minutes:
        # Slight delay: partial refund
        revenue = base_revenue - $2.00
    elif delivery_time <= sla_threshold + 30_minutes:
        # Major delay: larger refund
        revenue = base_revenue - $5.00
    else:
        # Extreme delay: full refund
        revenue = $0.00
    
    return revenue
```

### 5. SLA Compliance

**Service Level Agreement**:

```python
# Target delivery time
sla_threshold = 30.0  # minutes

# Actual delivery time
delivery_time = episode_length * dt / 60  # Convert to minutes

# SLA compliance
sla_compliant = delivery_time <= sla_threshold
```

**Compliance Rate**:

```python
# Over multiple episodes
sla_compliance_rate = num_compliant_deliveries / total_deliveries

# Baseline: 43.0% (current RL-Games policy)
# Target: 80%+ (competitive with human couriers)
```

## Business Metrics

### 1. Operating Margin

**Definition**: Profit as percentage of revenue

```python
def compute_operating_margin(episode):
    # Revenue
    revenue = compute_revenue(episode)
    
    # Costs
    energy_cost = compute_energy_cost(episode)
    maintenance_cost = compute_maintenance_cost(episode)
    capex_per_delivery = annual_capex / deliveries_per_year
    
    total_cost = energy_cost + maintenance_cost + capex_per_delivery
    
    # Profit
    profit = revenue - total_cost
    
    # Operating margin
    operating_margin = profit / revenue * 100  # Percentage
    
    return operating_margin
```

**Example Calculation**:

```python
# Successful delivery
revenue = $9.00
energy_cost = $0.005
maintenance_cost = $0.10
capex_per_delivery = $0.50  # Assuming 6000 deliveries/year

total_cost = $0.605
profit = $9.00 - $0.605 = $8.395
operating_margin = $8.395 / $9.00 = 93.3%

# Failed delivery (collision)
revenue = $0.00  # Full refund
energy_cost = $0.003
maintenance_cost = $5.10  # Includes collision repair
capex_per_delivery = $0.50

total_cost = $5.603
profit = $0.00 - $5.603 = -$5.603
operating_margin = -∞ (loss)
```

**Baseline Performance**:
- Current RL policy: 46.5% operating margin
- Target: 60%+ for sustainable business

### 2. Break-Even Time

**Definition**: Time to recover initial robot investment

```python
def compute_break_even_time(policy_performance):
    # Initial investment
    robot_cost = $15,000
    
    # Average profit per delivery
    avg_profit_per_delivery = compute_average_profit(policy_performance)
    
    # Deliveries per day
    deliveries_per_day = 20  # Assuming 8-hour operation
    
    # Daily profit
    daily_profit = avg_profit_per_delivery * deliveries_per_day
    
    # Break-even time
    break_even_days = robot_cost / daily_profit
    break_even_years = break_even_days / 365
    
    return break_even_years
```

**Example Calculation**:

```python
# Scenario 1: High-performing policy
avg_profit_per_delivery = $4.00
deliveries_per_day = 20
daily_profit = $80.00
break_even_time = $15,000 / $80.00 = 187.5 days = 0.51 years

# Scenario 2: Baseline policy
avg_profit_per_delivery = $2.00
deliveries_per_day = 20
daily_profit = $40.00
break_even_time = $15,000 / $40.00 = 375 days = 1.03 years

# Scenario 3: Poor policy (many collisions)
avg_profit_per_delivery = $0.50
deliveries_per_day = 20
daily_profit = $10.00
break_even_time = $15,000 / $10.00 = 1500 days = 4.11 years
```

**Baseline Performance**:
- Current RL policy: 0.90 years break-even
- Target: <1 year for attractive ROI

### 3. Cost Breakdown

**Typical Operating Costs** (from baseline policy):

```
Total Operating Cost: $3.00 per delivery

Breakdown:
- Hardware Amortization: $1.05 (35%)
- Maintenance: $1.01 (34%)
- Energy: $0.15 (5%)
- Other (insurance, etc.): $0.79 (26%)
```

**Key Insights**:
- Maintenance and hardware dominate costs
- Collision avoidance is highest-leverage improvement
- Energy optimization has limited impact

## Integration with Training

### Logging Cost Metrics

Cost metrics are logged during training via `rl_games_helpers.py`:

```python
def on_episode_end(episode_info):
    # Compute cost metrics
    energy = compute_navigation_energy_step(env)
    sla_compliant = check_sla_compliance(episode_info)
    maintenance_cost = compute_maintenance_cost(episode_info)
    
    # Log to TensorBoard
    writer.add_scalar("cost_model/energy", energy, step)
    writer.add_scalar("cost_model/sla_compliance", sla_compliant, step)
    writer.add_scalar("cost_model/maintenance_cost", maintenance_cost, step)
```

### Cost-Aware Reward Shaping

Future work: Incorporate cost metrics directly into reward function

```python
def cost_aware_reward(episode):
    # Task reward (reaching goal)
    task_reward = compute_task_reward(episode)
    
    # Cost penalty
    cost_penalty = (
        energy_cost * energy_weight +
        maintenance_cost * maintenance_weight +
        sla_penalty * sla_weight
    )
    
    # Total reward
    total_reward = task_reward - cost_penalty
    
    return total_reward
```

This would enable learning policies that directly optimize for profitability.

## Comparison with Human Couriers

### Human Courier Economics

**Costs**:
- Pay per delivery: $8.49 (DoorDash) to $10.00 (Uber Eats)
- Platform fees: 15-30% of order value
- No hardware amortization
- Vehicle maintenance (if applicable)

**Performance**:
- SLA compliance: 70-80%
- Delivery time: 26-38 minutes average
- Collision rate: Very low (human judgment)

### Robot Courier Economics

**Costs**:
- Hardware amortization: $0.50 per delivery
- Energy: $0.005 per delivery
- Maintenance: $0.10-$5.10 per delivery (depending on collisions)
- No labor cost

**Performance** (baseline RL policy):
- SLA compliance: 43.0%
- Delivery time: Variable
- Collision rate: Higher than humans

**Break-Even Analysis**:

For robots to be competitive:
- Need SLA compliance >70%
- Need collision rate <5%
- Need operating margin >50%

Current baseline achieves:
- SLA compliance: 43.0% ❌
- Operating margin: 46.5% ❌
- Break-even time: 0.90 years ✓

**Conclusion**: Significant room for improvement in navigation policy to achieve competitive economics.

## Future Enhancements

1. **Cloud Inference Costs**: Model latency and bandwidth costs for cloud-based policies
2. **Dynamic Pricing**: Adjust revenue based on demand, distance, time of day
3. **Fleet Management**: Optimize across multiple robots (routing, charging)
4. **Insurance Costs**: Model liability and insurance based on collision history
5. **Regulatory Compliance**: Factor in permits, inspections, certifications

