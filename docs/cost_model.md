# :moneybag: Cost Model

CostNav evaluates navigation policies through a comprehensive economic cost-revenue model grounded in real-world data. All parameters are referenced from industry sources including SEC filings, AIS injury reports, manufacturer specs, and delivery platform pricing.

> For the full derivation, see Section 3.1 of the [paper](https://arxiv.org/abs/2511.20216) and the [cost formula spreadsheet](https://drive.google.com/drive/folders/1j1MXm6NMkd6HHBwTJi_nSde7-shv4laX?usp=sharing).

---

## :dart: Cost Framework Overview

The total profit of a navigation policy is:

\[
\mathbb{P} = \mathbb{R} - (C_{\text{CAPEX}} + C_{\text{OPEX}} \times N)
\]

where \(\mathbb{R}\) is cumulative delivery revenue, CAPEX is upfront capital expenditure, OPEX is per-delivery operational cost, and \(N\) is the number of completed deliveries.

The **contribution margin** per delivery is:

\[
C_{\text{ContributionMargin}} = R - C_{\text{OPEX}}
\]

where \(R = P_{\text{MktRobotDeli}} \times S_{\text{EpisodeTermSLA}}\) (base delivery fee x SLA compliance; zero if timeout).

The **Break-Even Point (BEP)** is the number of deliveries required to recover capital expenditure:

\[
\text{BEP} = \frac{C_{\text{CAPEX}}}{R - C_{\text{OPEX}}}
\]

When contribution margin is negative (\(R < C_{\text{OPEX}}\)), the BEP is undefined --- the system is not economically viable.

---

## :office: Capital Expenditure (CAPEX)

One-time fixed investments incurred before deployment.

### Hardware Cost

\[
C_{\text{Hardware}} = P_{\text{Robot}} + (P_{\text{Lidar}} + P_{\text{GPS}})
\]

| Parameter | Value | Unit | Reference |
|:----------|------:|:-----|:----------|
| Robot Cost (\(P_{\text{Robot}}\)) | 8,000 | $/robot | Retail price, Segway E1 delivery robot [[21]](https://multirotors.store) |
| LiDAR Cost (\(P_{\text{Lidar}}\)) | 3,000 | $/robot | Livox LiDAR for delivery robots [[28]](https://www.segwayrobotics.com) |
| GPS Cost (\(P_{\text{GPS}}\)) | 2,000 | $/robot | RTK Device (with IMU) for delivery robots [[28]](https://www.segwayrobotics.com) |

!!! note "Sensor Configuration"
    Rule-based methods (Nav2) use Robot + LiDAR + GPS = **\$13,000**. Learning-based methods use Robot only = **\$8,000** (CANVAS adds GPS = **\$10,000**).

### Data Collection Cost

For learning-based navigation, the cost of collecting training data:

\[
C_{\text{DataCollection}} = P_{\text{DataCollector}} \times S_{\text{DataCollectionTime}}
\]

| Parameter | Value | Unit | Reference |
|:----------|------:|:-----|:----------|
| Data Collector Wage (\(P_{\text{DataCollector}}\)) | 33 | $/hour | U.S. median hourly wage for data collection personnel [[26]](https://salary.com) |
| Data Collection Time | 65.63 | hours | Total teleoperation hours in CostNav |

### CAPEX Summary (Table 3 from paper)

| Component | Nav2 w/ AMCL | Nav2 w/ GPS | GNM | ViNT | NoMaD | NavDP | CANVAS |
|:----------|-------------:|------------:|----:|-----:|------:|------:|-------:|
| Hardware | 13,000 | 13,000 | 8,000 | 8,000 | 8,000 | 8,000 | 10,000 |
| Data Collection | 0 | 0 | 2,165.79 | 2,165.79 | 2,165.79 | 2,165.79 | 2,165.79 |
| **Total CAPEX** | **13,000** | **13,000** | **10,165.79** | **10,165.79** | **10,165.79** | **10,165.79** | **12,165.79** |

---

## :receipt: Operational Expenditure (OPEX)

Variable costs incurred on a per-delivery basis, directly influenced by navigation behavior. Organized into three categories: direct costs (electricity, repair), service costs (customer compensation), and liability costs (pedestrian and property damage).

### Electricity Cost

\[
C_{\text{Elec}} = P_{\text{Elec}} \times \frac{S_{\text{AvgPower}}}{C_{\text{ElectroMechanicalEff}}} \times T_{\text{AvgDeliveryTime}}
\]

| Parameter | Value | Unit | Reference |
|:----------|------:|:-----|:----------|
| Electricity Rate (\(P_{\text{Elec}}\)) | 0.2704 | $/kWh | U.S. average retail electricity price [[48]](https://www.eia.gov) |
| Electro-Mechanical Efficiency (\(C_{\text{ElectroMechanicalEff}}\)) | 0.73 | unitless | Derived: battery roundtrip (0.95) x motor (0.82) x inverter (0.99) x charge (0.95) [[4]](https://batterydesign.net), [[41]](https://www.st.com), [[42]](https://www.ti.com), [[56]](https://www.zau-motors.com) |

### Repair Cost

\[
C_{\text{RepairRun}} = \frac{P_{\text{Robot}}}{N_{\text{RobotLifeRun}}} \times C_{\text{Repair}} \times \frac{S_{\text{EpisodeTermPhys}}}{C_{\text{PhysicalAssistance}}}
\]

| Parameter | Value | Unit | Reference |
|:----------|------:|:-----|:----------|
| Useful Life of Robots (\(T_{\text{RobotLife}}\)) | 2 | years | Expected operational lifespan per SEC filings [[30]](https://sec.gov) |
| Useful Delivery Run Count (\(N_{\text{RobotLifeRun}}\)) | 18,250 | run/robot | Derived: \(T_{\text{RobotLife}} \times 365 \times N_{\text{Delivery}}\) |
| Repair Rate (\(C_{\text{Repair}}\)) | 10--20 | %/year | Annual maintenance cost as % of robot price [[13]](https://igus.com), [[15]](https://lucasware.com), [[36]](https://standardbots.com) |

### Service Compensation Cost

Failed deliveries incur compensation for customer refunds:

\[
C_{\text{ServiceCompRun}} = S_{\text{EpisodeTermSpoiled}} \times P_{\text{MktFood}} + (S_{\text{EpisodeTermTimeout}} + S_{\text{EpisodeTermPhys}}) \times P_{\text{MktRobotDeli}}
\]

| Parameter | Value | Unit | Reference |
|:----------|------:|:-----|:----------|
| Market Robot Delivery Price (\(P_{\text{MktRobotDeli}}\)) | 3.49 | $/run | Consumer delivery fee for campus robot services [[47]](https://sc.edu/about/offices_and_divisions/dining/) |
| Avg Market Food Price (\(P_{\text{MktFood}}\)) | 31.93 | $/run | Derived: \(P_{\text{Refund}} - P_{\text{MktRobotDeli}}\) |
| Avg Refund Price (\(P_{\text{Refund}}\)) | 35.42 | $/run | Industry average order value for delivery failures [[43]](https://therestauranthq.com) |

### Pedestrian Safety Cost

Modeled using the Abbreviated Injury Scale (AIS), measured from collision delta-v:

\[
C_{\text{PedestrianRun}} = \sum_{\text{AIS}} P(\text{AIS} \mid \Delta v) \times P_{\text{PedDamageAIS}} \times K_{\text{InjuryAdjustment}}
\]

where \(P(\text{AIS} \mid \Delta v)\) is the probability of AIS severity given impact speed [[51]](https://www.nhtsa.gov), and \(K_{\text{InjuryAdjustment}}\) adjusts for vehicle weight differences between the delivery robot and the crash report reference vehicle.

| Parameter | Value | Unit | Reference |
|:----------|------:|:-----|:----------|
| Avg Pedestrian Damage (AIS0) | 380 | $/event | Economic cost per MAIS pedestrian incident [[3]](https://www.aaam.org), [[22]](https://www.nhtsa.gov) |
| Avg Pedestrian Damage (AIS1) | 8,487 | $/event | Same as above |
| Avg Pedestrian Damage (AIS2) | 60,464 | $/event | Same as above |
| Avg Pedestrian Damage (AIS3) | 261,200 | $/event | Same as above |
| MAIS Report Passenger Vehicle Weight | 4,536 | kg | Derived: 10,000 lbs = 4,536 kg, per MAIS Injury Report [[51]](https://www.nhtsa.gov) |

### Property Damage Cost

Collisions with urban infrastructure incur repair or replacement costs:

\[
C_{\text{PropertyRun}} = \sum_j S_{\text{PropContact}_j} \times P_{\text{PropDamage}_j}
\]

| Parameter | Value | Unit | Reference |
|:----------|------:|:-----|:----------|
| Mailbox Damage Cost | 50 | $/event | Municipal replacement cost [[45]](https://townofwrentham.com) |
| Trash Bin Damage Cost | 50 | $/event | Municipal replacement cost [[7]](https://cityhs.net) |
| Building Glass Damage Cost | 300 | $/event | Estimated repair cost [[54]](https://wittenberg.edu) |
| Bollard Damage Cost | 65 | $/event | Estimated replacement cost [[52]](https://wearebollards.com) |

### OPEX Summary (Table 3 from paper, $/run)

| Component | Nav2 w/ AMCL | Nav2 w/ GPS | GNM | ViNT | NoMaD | NavDP | CANVAS |
|:----------|-------------:|------------:|----:|-----:|------:|------:|-------:|
| Electricity | 0.0029 | 0.0031 | 0.0005 | 0.0009 | 0.0003 | 0.0001 | 0.0027 |
| Service Compensation | 5.4021 | 10.1322 | 3.4900 | 3.1410 | 3.4551 | 3.4900 | 3.8910 |
| Pedestrian Safety | 29.3200 | 20.5800 | 11.6400 | 29.8900 | 9.7600 | 9.3000 | 14.3800 |
| Property Damage | 0.0000 | 1.0000 | 3.6500 | 6.0000 | 12.0000 | 0.0000 | 6.0000 |
| Repair | 10.8493 | 7.2329 | 5.2603 | 11.8356 | 6.9041 | 2.9589 | 6.5753 |
| **Total OPEX** | **45.5743** | **38.9482** | **24.0407** | **50.8675** | **32.1195** | **15.7491** | **30.8490** |

---

## :money_with_wings: Revenue

\[
R = P_{\text{MktRobotDeli}} \times S_{\text{EpisodeTermSLA}}
\]

Revenue per delivery is the market delivery fee (\$3.49), modulated by SLA compliance. Deliveries exceeding the timeout receive **zero revenue**. Food intactness determines service completeness.

| Parameter | Value | Unit | Reference |
|:----------|------:|:-----|:----------|
| Delivery Runs per Day (\(N_{\text{Delivery}}\)) | 25 | runs/day | Reported daily throughput per robot, SEC filings [[29]](https://sec.gov) |
| Delivery Failure Rate (\(C_{\text{DeliveryFailure}}\)) | 0.06 | %/run | Operational failure rate per SEC filings [[29]](https://sec.gov) |
| Operator:Robot Ratio (\(N_{\text{Fleet}}\)) | 4 | robot/operator | Level 4 autonomy supervision ratio per SEC filings [[29]](https://sec.gov) |
| Robot Operator Wage (\(P_{\text{Operator}}\)) | 24 | $/hour | U.S. median hourly wage for robot operators [[27]](https://salary.com) |

---

## :robot: Simulation-Measured Inputs

These values come from the Isaac Sim physics simulation and feed into the cost formulas above:

| Parameter | Value | Unit | Reference |
|:----------|------:|:-----|:----------|
| Robot Speed (\(C_{\text{RobotSpeed}}\)) | 6.00 | km/h | Maximum operational velocity, Segway E1 spec [[40]](https://starshiptechnologies.com) |
| Robot Max Speed (\(C_{\text{RobotMaxSpeed}}\)) | 12.80 | km/h | Maximum nominal speed of Segway E1 [[28]](https://www.segwayrobotics.com) |
| Robot Weight (\(C_{\text{RobotWeight}}\)) | 50 | kg | Maximum off-load weight, Segway E1 [[28]](https://www.segwayrobotics.com) |
| Avg Delivery Time (\(T_{\text{AvgDeliveryTime}}\)) | 0.3667 | hr/run | Mean delivery duration from operational deployments [[39]](https://starshiptechnologies.com) |
| Max Delivery Time (\(T_{\text{MaxDeliveryTime}}\)) | 1 | hr/run | Maximum guaranteed delivery time [[1]](https://agvnetwork.com), [[37]](https://starshiptechnologies.com) |
| Avg Delivery Distance (\(T_{\text{AvgDeliveryDistance}}\)) | 4 | km/run | Mean delivery distance per SEC filings [[29]](https://sec.gov) |
| Max Delivery Distance (\(C_{\text{MaxDeliveryDistance}}\)) | 6 | km/run | Maximum operational delivery radius [[38]](https://starshiptechnologies.com) |
| Charge Efficiency (\(\eta_{\text{charge}}\)) | 0.95 | unitless | Estimated charge efficiency [[41]](https://www.st.com) |
| Battery Roundtrip Efficiency (\(\eta_{\text{batteryroundtrip}}\)) | 0.95 | unitless | Estimated battery roundtrip efficiency [[4]](https://batterydesign.net) |
| Inverter Efficiency (\(\eta_{\text{inverter}}\)) | 0.99 | unitless | Estimated inverter efficiency [[42]](https://www.ti.com) |
| Motor Efficiency (\(\eta_{\text{motor}}\)) | 0.82 | unitless | Estimated motor efficiency [[56]](https://www.zau-motors.com) |

---

## :crystal_ball: Future Enhancements

| Enhancement | Description |
|:------------|:------------|
| :brain: **Cost-Aware Reward Shaping** | Use the cost model as a reward signal for RL, enabling agents to directly optimize for profitability |
| :cloud: **Cloud Inference Costs** | Model latency and bandwidth costs for cloud-based VLA policies |
| :chart_with_upwards_trend: **Dynamic Pricing** | Adjust revenue based on demand, distance, time of day |
| :truck: **Fleet Management** | Optimize across multiple robots (routing, charging) |
| :traffic_light: **Traffic Rule Compliance** | Factor in fines and liability costs from traffic violations |

---

## :books: References

| # | Reference |
|:--|:----------|
| [1] | AGV Network. Starship Technologies FAQs. [FAQ page](https://agvnetwork.com), 2024. |
| [3] | Association for the Advancement of Automotive Medicine. *Abbreviated Injury Scale: 2015 Revision*. Chicago, IL, 6 edition, 2018. |
| [4] | Battery Design. Round-trip efficiency of lithium-ion batteries. [Battery Design article](https://batterydesign.net), 2024. |
| [7] | City of Hot Springs. Frequently asked questions. [Solid Waste FAQ](https://cityhs.net), 2026. |
| [13] | Jared Worth. Maximizing humanoid robot longevity: the power of maintenance-free components. [igus Toolbox article](https://igus.com), 2024. |
| [15] | Lucas. The ROI of autonomous mobile robots in your DC. [Lucasware article](https://lucasware.com), 2020. |
| [21] | Multirotors.store. Segway outdoor delivery robot. [Store Listing](https://multirotors.store), 2025. |
| [22] | National Highway Traffic Safety Administration. The economic and societal impact of motor vehicle crashes, 2019. Report No. DOT HS 813 403, 2023. |
| [26] | Salary.com. Data collector salary information. [Salary.com listing](https://salary.com), 2025. |
| [27] | Salary.com. Robot operator salary information. [Salary.com listing](https://salary.com), 2025. |
| [28] | Segway Robotics. Segway E1 robot specifications (E1 delivery parameters). [Specification PDF](https://www.segwayrobotics.com), 2025. |
| [29] | Serve Robotics Inc. Serve robotics inc. form 10-k for fiscal year 2023. U.S. Securities and Exchange Commission. Available at [SEC EDGAR database](https://sec.gov). |
| [30] | Serve Robotics Inc. Serve robotics inc. form 10-q. U.S. Securities and Exchange Commission, 2024. Available at [SEC EDGAR database](https://sec.gov). |
| [31] | Serve Robotics Inc. Serve robotics inc. form 424b4 prospectus. [SEC EDGAR filing](https://sec.gov), 2024. |
| [36] | Standard Bots. How much do robots cost? 2026 price breakdown. [Standard Bots article](https://standardbots.com), 2026. |
| [37] | Starship Technologies. Starship technologies launches commercial rollout of autonomous delivery. [Press release](https://starshiptechnologies.com), 2024. |
| [38] | Starship Technologies. Starship dimensions. [Dimensions spec page](https://starshiptechnologies.com), 2024. |
| [39] | Starship Technologies. Starship technologies celebrates one year of robot deliveries in Finland. [Press release](https://starshiptechnologies.com), 2024. |
| [40] | Starship Technologies. Starship robot technical specifications. [Starship Deliveries industry page](https://starshiptechnologies.com), 2024. |
| [41] | STMicroelectronics. GaN-based Li-ion Battery Charger Demo. [Seminar PDF](https://www.st.com), APEC 2023. |
| [42] | Texas Instruments. TIDA-010056: 48V 3-Phase Inverter. [TI TIDA design page](https://www.ti.com), 2024. |
| [43] | The Restaurant HQ. Food delivery statistics. [Article](https://therestauranthq.com), 2024. |
| [45] | Town of Wrentham, MA. Mailbox damage claims (news flash). [Article](https://townofwrentham.com), 2026. |
| [47] | University of South Carolina. Grubhub and Starship delivery. [USC Dining Services page](https://sc.edu/about/offices_and_divisions/dining/), 2024. |
| [48] | U.S. Energy Information Administration. Electric power report. [EIA electricity data page](https://www.eia.gov), November 2025. |
| [51] | Wang, J.-S. Mais(05/08) injury probability curves as functions of delta v. [NHTSA report PDF](https://www.nhtsa.gov), May 2022. |
| [52] | We Are Bollards. How much does it cost to install a bollard? [Blog post](https://wearebollards.com), 2024. |
| [54] | Wittenberg University. Wittenberg university damage charge schedule 2024-2025. [Damage Charge Schedule PDF](https://wittenberg.edu), 2024. |
| [56] | ZAU. M250W Hub Motor Specifications. [ZAU motor spec page](https://www.zau-motors.com), 2024. |
