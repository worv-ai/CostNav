Ref: https://chatgpt.com/share/691ebb05-6d48-8004-ad22-2736e17a6be3


# **0. Common Environmental Parameters (U.S. Baseline)**

### **(1) Order-Distance Distribution `D_order`**

* For U.S. food delivery (all platforms), the **median delivery distance ≈ 2.5 miles**.

  * Source: Serve Robotics investor materials stating *“U.S. food delivery median distance = 2.5 miles.”*  ([investors.serverobotics.com][1])

* Simulator defaults:

  * `D_order_median ≈ 2.5 mi`
  * `D_order_mean ≈ 2.5–3.0 mi` (with a long-tail including 4–5+ mi orders)

### **(2) Delivery-Time Distribution `T_delivery` (All Platforms)**

Intouch Insight tested 600 orders in the U.S. (200 per app): ([foodondemand.com][2])

* All apps avg: `T_delivery_all ≈ 33.4 min`
* DoorDash: `T_delivery_DD ≈ 26.4 min`
* Uber Eats: `T_delivery_UE ≈ 38.1 min`

Approximate **orders/hour**:

* DoorDash: `≈ 2.27 orders/hour`
* Uber Eats: `≈ 1.58 orders/hour`
* Overall average: `≈ 1.8 orders/hour`
  (Includes restaurant wait, parking, walking, etc.)

---

# **1. Human Couriers (DoorDash / Uber Eats) — Simulator Inputs**

## **1-1. Basic Mobility Metrics**

### **Distance `d_h`**

* Based on U.S. median distance, `d_h ≈ 2.5 mi` is a natural baseline. ([investors.serverobotics.com][1])
* Simulator example:

  * DoorDash & Uber Eats shared baseline:
    `d_h ~ N(μ=2.5 mi, σ≈1.0 mi)` with tail orders >5 mi

### **Time `T_h_total`**

* DoorDash: avg `26.4 min` ([foodondemand.com][2])
* Uber Eats: avg `38.1 min` ([foodondemand.com][2])
* Overall: `33.4 min`

Simulator baseline example:

* `T_h_total_DD_mean = 26.5 min`
* `T_h_total_UE_mean = 38.0 min`
* Noise `±8–10 min` to model restaurant delays & peak hours

---

## **1-2. Revenue & Cost Metrics (Platform + Courier Perspective)**

### **(A) Driver Pay / Platform Payouts**

Gridwise U.S. aggregation (2023Q1–2024Q2): ([Gridwise][3])

**DoorDash**

* Per-task pay: `pay_h_task_DD ≈ $8.49`
* Hourly earnings: `≈ $18.93/h`

**Uber Eats**

* Per-task pay: `pay_h_task_UE ≈ $10.00`
* Hourly earnings: `≈ $24.68/h`

Simulator baseline (platform payout):

* DoorDash: `≈ 8.5 USD/order`
* Uber Eats: `≈ 10.0 USD/order`
  (Tips are customer→driver, not a platform expense)

---

### **(B) Vehicle Cost per Mile**

* **IRS 2025 standard mileage rate: $0.70/mile** ([IRS][4])

Simulator:

* `c_vehicle_per_mile = 0.70 USD/mile`
* Example vehicle cost per order:

  * If `d_h_total ≈ 3.0 mi`:
    `C_vehicle_order = 0.70×3.0 ≈ 2.1 USD`

---

## **1-3. Accident & Safety Metrics**

### **(A) Accident Probability `p_accident_h`**

Sentiance/industry estimate: **~1 accident per 143,000 deliveries**.
([Food Delivery Apparel][5])

* `p_accident_h_per_order ≈ 7.0 × 10⁻⁶`
* DoorDash reports **>99.99% of deliveries completed safely**, consistent with the above.
  ([about.doordash.com][6])

### **(B) Cost per Accident**

U.S. averages:

* Collision repair cost avg: **≈ $4,700+** ([Repairer Driven News][7])
* Liability claim averages:

  * Property damage: `≈ $6,551`
  * Bodily injury: `≈ $26,501` ([III][8])

Simulator simplification:

* Minor accident: `C_accident_minor ≈ 5,000 USD`
* Full average incl. injury: `C_accident_avg ≈ 20k–30k USD`

**Expected accident cost per order:**

* Property-only:
  `E[C_accident_prop] ≈ 0.046 USD/order`
* Conservative full cost:
  `E[C_accident_full] ≈ 0.23 USD/order`

Practical simulator setting:
**`0.05–0.20 USD/order`** as tunable expected accident-cost parameter.

---

## **1-4. SLA & Promotions (Human Delivery)**

### **Delivery Delay / SLA**

Based on Intouch Insight: DoorDash is faster (26.4 min) than Uber Eats (38.1 min). ([foodondemand.com][2])

Informal industry norms:

* Delays **>10–15 min** over ETA → coupon/partial refund common.

Simulator example:

* `L_h > 10 min → C_late_h ≈ $2`
* `L_h > 30 min → $5`
* `L_h > 60 min → $10`

---

### **Promotions (Peak Pay / Surge)**

**DoorDash Peak Pay:**
+`$1–$4/order`, occasionally **+$7.50**. ([foodondemand.com][2])

**Uber Eats Surge/Boost:**
Multipliers (1.3×, 1.5×) or flat fees (+2–5 USD).

Simulator:

* DoorDash: `{0, 1, 2, 3, 4, 7.5} USD`
* UberEats: multiplier `1.0–2.0` or flat `2–5 USD`

---

# **2. Delivery Robots (Starship, Serve, Generic, Nuro) — Simulator Inputs**

## **2-1. Generic Sidewalk Robots (Academic / Industry Source)**

Typical small sidewalk robot (Berkeley CMR): ([California Management Review][9])

* Top speed: `≈ 4 mph`
* Cruise: `3–4 mph`
* Unit cost: `≈ 2,500–5,000 USD`

Simulator:

* `v_r_sidewalk = 3.5 mph`
* `C_robot_unit_generic = 4,000 USD`

---

## **2-2. Starship Technologies (Sidewalk Robots)**

### **Performance**

* Top speed: `≈ 6 km/h (3.7 mph)` ([California Management Review][9])
* Battery life: `≈ 18 hours`
* Daily distance: `≈ 40 km/day` ([TechCrunch][10])
* Payload: `≈ 20 lb` ([Wikipedia][11])

Simulator:

* `H_r_max_per_day_Starship = 18 h`
* `range_r_per_day_Starship = 40 km`
* If each order ≈ 2–3 km → `12–15 orders/day`

### **Capex / Autonomy**

* Starship estimated unit cost:

  * 2018: `≈ 5,500 USD`
  * Target: `≈ 2,250 USD` ([Accio][12])

* Autonomy ≈ **99%** ([starship.xyz][13])

Simulator:

* `C_robot_unit_Starship ≈ 3,000–5,500 USD`
* `autonomy_ratio_Starship ≈ 0.99`
* `p_intervention_r_Starship ≈ 0.01`

---

## **2-3. Serve Robotics (Gen2 / Gen3)**

### **Distance Environment**

* U.S. median food-delivery distance = **2.5 miles**. ([investors.serverobotics.com][1])
* Serve-DoorDash sample:

  * *Median Serve order distance* ≈ **1.3 miles**
  * Delivery time ≈ **18 min**
  * Max speed ≈ **11 mph**
    ([Investors.com summary][14])

Simulator:

* `d_r_Serve ≈ 1.3 mi`
* `T_r_total_Serve ≈ 18 min`
* `v_r_cruise_Serve ≈ 5–8 mph`

---

### **Operating Hours (Gen2 vs Gen3)**

2024Q2 operational data:

* Daily supply hours: **385 h**
* Active robots ≈ 48 → **~8 h/day per robot** ([Restaurant Dive][15])

Gen3 improvements:

* Speed ×2
* Range ×2
* +6 hours operation per day ([Quiver Quantitative][16])

Simulator:

* Gen2: `H_r_max/day ≈ 8 h`
* Gen3: `≈ 14 h`
* with doubled range & speed

---

### **Cost Structure**

Investor presentation:

* Serve long-term goal: **$1.00/order fully-loaded last-mile cost**.
  ([investors.serverobotics.com][1])

Simulator:

* `C_total_r_Serve_target ≈ 1.0 USD/order`
* Early-stage scenarios: 1–3 USD/order

### **Reliability**

Serve reports operational efficiency ~**99.8%** (vs human ~98%).
([Barron's][17])

Simulator:

* `p_failure_r_Serve ≈ 0.002`
* `p_failure_h ≈ 0.02`

---

## **2-4. Nuro R2 (Road-Driving Robots)**

* Road speed: `≈ 25 mph` (R3 → 45 mph) ([SlashGear][18])
* Battery: `≈ 31 kWh`, city range tens to >100 km

Simulator:

* `v_r_road_cruise ≈ 20 mph`
* `range_r_road ≈ 100–150 km/day`
* `C_robot_unit_road ≳ 50,000 USD` (treated as a scenario parameter)

---

# **3. Humans vs Robots — Shared KPI Inputs**

## **3-1. Total Cost per Order `C_total`**

### **Human (DoorDash/Uber Eats)**

* Driver payout: `≈ 8.5–10 USD` ([Gridwise][3])
* Vehicle cost: `≈ 2.1 USD` (3 mi × $0.70) ([IRS][4])
* Accidents: `≈ 0.05–0.20 USD/order` ([Food Delivery Apparel][5])

Baseline:

```
C_total_h_DD ≈ 8.5 + 2.1 + 0.1 = 10.7 USD/order
C_total_h_UE ≈ 10.0 + 2.1 + 0.1 = 12.2 USD/order
```

### **Robots (Serve / Starship)**

* Serve goal: `≈ 1.0 USD/order` ([investors.serverobotics.com][1])

Starship estimation:

* Depreciation: `≈ 0.3–0.8 USD/order`
* Energy: `≈ 0.01–0.03 USD/order` ([TechCrunch][10])
* Maintenance + teleop → target 1–3 USD/order
  ([California Management Review][9])

Baseline:

```
C_total_r_Starship ≈ 1.5–2.0 USD/order
C_total_r_Serve   ≈ 1.0–2.0 USD/order (target: 1.0)
```

---

## **3-2. Productivity / Utilization**

### **Human Drivers**

* DoorDash: `≈ 2.27 orders/h` ([foodondemand.com][2])
* Uber Eats: `≈ 1.58 orders/h`
* Full 8h shift: `≈ 12–18 orders/day`

### **Robots**

* Starship: 18h/day, 40 km/day → `≈ 12–15 orders/day` ([TechCrunch][10])
* Serve Gen2: `≈ 8h/day`
* Serve Gen3: `≈ 14h/day` ([Restaurant Dive][15])

---

# **Usage in Simulator**

Using this style — listing **(DoorDash: X, Uber Eats: Y, Starship: Z, Serve: W)** next to each variable — allows consistent comparison across scenarios:

* Baseline
* Human-only
* Robot-only
* Mixed fleets

If you want, I can also produce:

* **YAML/JSON configuration templates**
* **Python dataclasses for these parameters**
* **Auto-calibration functions for cost curves**

---

**All original references remain unchanged**:

[1] [https://investors.serverobotics.com/static-files/0138a856-bd69-4564-9f8d-0534f9cebb73](https://investors.serverobotics.com/static-files/0138a856-bd69-4564-9f8d-0534f9cebb73)
[2] [https://foodondemand.com/09262024/doordash-tops-intouch-insight-report-on-delivery-performance/](https://foodondemand.com/09262024/doordash-tops-intouch-insight-report-on-delivery-performance/)
[3] [https://gridwise.io/blog/delivery/uber-eats-vs-doordash-pay-how-much-are-drivers-earning/](https://gridwise.io/blog/delivery/uber-eats-vs-doordash-pay-how-much-are-drivers-earning/)
[4] [https://www.irs.gov/tax-professionals/standard-mileage-rates](https://www.irs.gov/tax-professionals/standard-mileage-rates)
[5] [https://fooddeliveryapparel.com/blogs/news/the-hidden-dangers-when-youre-on-the-road-the-reality-of-being-a-food-delivery-driver](https://fooddeliveryapparel.com/blogs/news/the-hidden-dangers-when-youre-on-the-road-the-reality-of-being-a-food-delivery-driver)
[6] [https://about.doordash.com/en-us/news/how-were-making-dashing-even-safer](https://about.doordash.com/en-us/news/how-were-making-dashing-even-safer)
[7] [https://www.repairerdrivennews.com/2024/12/20/ccc-top-2024-trends-perfect-storm-of-softening-economy-increasing-repair-costs-and-aging-vehicle-pool/](https://www.repairerdrivennews.com/2024/12/20/ccc-top-2024-trends-perfect-storm-of-softening-economy-increasing-repair-costs-and-aging-vehicle-pool/)
[8] [https://www.iii.org/fact-statistic/facts-statistics-auto-insurance](https://www.iii.org/fact-statistic/facts-statistics-auto-insurance)
[9] [https://cmr.berkeley.edu/2022/04/self-driving-robots-a-revolution-in-the-local-delivery/](https://cmr.berkeley.edu/2022/04/self-driving-robots-a-revolution-in-the-local-delivery/)
[10] [https://techcrunch.com/2022/01/25/starship-technologies-picks-up-e50m-from-the-eus-investment-arm-to-expand-its-fleet-of-autonomous-delivery-robots/](https://techcrunch.com/2022/01/25/starship-technologies-picks-up-e50m-from-the-eus-investment-arm-to-expand-its-fleet-of-autonomous-delivery-robots/)
[11] [https://en.wikipedia.org/wiki/Starship_Technologies](https://en.wikipedia.org/wiki/Starship_Technologies)
[12] [https://www.accio.com/plp/wheeled-delivery-robots](https://www.accio.com/plp/wheeled-delivery-robots)
[13] [https://www.starship.xyz/](https://www.starship.xyz/)
[14] [https://www.investors.com/news/technology/serve-robotics-doordash-partnership/](https://www.investors.com/news/technology/serve-robotics-doordash-partnership/)
[15] [https://www.restaurantdive.com/news/serve-robotics-debuts-los-angeles-faster-larger-robots-uber-eats-deal/729973/](https://www.restaurantdive.com/news/serve-robotics-debuts-los-angeles-faster-larger-robots-uber-eats-deal/729973/)
[16] [https://www.quiverquant.com/news/Serve%2BRobotics%2BSecures%2BTop%2BSpot%2Bin%2BFast%2BCompany%27s%2B%22Next%2BBig%2BThings%2Bin%2BTech%22%2Bfor%2BThird-Generation%2BAutonomous%2BDelivery%2BRobot](https://www.quiverquant.com/news/Serve%2BRobotics%2BSecures%2BTop%2BSpot%2Bin%2BFast%2BCompany%27s%2B%22Next%2BBig%2BThings%2Bin%2BTech%22%2Bfor%2BThird-Generation%2BAutonomous%2BDelivery%2BRobot)
[17] [https://www.barrons.com/articles/serve-ai-robot-uber-stock-results-beb6ba3d](https://www.barrons.com/articles/serve-ai-robot-uber-stock-results-beb6ba3d)
[18] [https://www.slashgear.com/nuro-r2-self-driving-delivery-pod-hits-us-autonomous-vehicle-milestone-06608901/](https://www.slashgear.com/nuro-r2-self-driving-delivery-pod-hits-us-autonomous-vehicle-milestone-06608901/)
