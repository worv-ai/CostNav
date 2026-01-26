#!/bin/bash
# Unified Evaluation Script for Teleop and Nav2
# Runs consecutive missions and generates comprehensive evaluation logs
#
# Usage: ./eval.sh <MODE> [TIMEOUT] [NUM_MISSIONS]
#   MODE: Either 'teleop' or 'nav2'
#   TIMEOUT: Mission timeout in seconds (default: 20)
#   NUM_MISSIONS: Number of missions to run (default: 10)
#
# Requires: A running instance (make run-teleop or make run-nav2)
#
# Controls:
#   Right Arrow (→): Skip current mission

set -e

# Mode validation
MODE="${1:-}"
if [ -z "$MODE" ] || { [ "$MODE" != "teleop" ] && [ "$MODE" != "nav2" ]; }; then
    echo "ERROR: Mode must be 'teleop' or 'nav2'"
    echo "Usage: $0 <teleop|nav2> [TIMEOUT] [NUM_MISSIONS]"
    exit 1
fi

# Configuration based on mode
if [ "$MODE" = "teleop" ]; then
    CONTAINER_NAME="costnav-ros2-teleop"
    MODE_DISPLAY="Teleop"
else
    CONTAINER_NAME="costnav-ros2-nav2"
    MODE_DISPLAY="Nav2"
fi

# Configuration
TIMEOUT="${2:-20}"
NUM_MISSIONS="${3:-10}"
CONTAINER=""
LOG_DIR="${LOG_DIR:-./logs}"
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
LOG_FILE="${LOG_DIR}/${MODE}_evaluation_${TIMESTAMP}.log"

# Statistics tracking
SUCCESS_SLA=0
FAILURE_TIMEOUT=0
FAILURE_PHYSICALASSISTANCE=0
FAILURE_FOODSPOILED=0
SKIPPED=0
declare -a MISSION_RESULTS
declare -a MISSION_START_TIMES
declare -a MISSION_END_TIMES
declare -a MISSION_ERRORS
declare -a MISSION_TRAVELED_DISTANCES
declare -a MISSION_ELAPSED_TIMES
declare -a MISSION_AVG_VELOCITIES
declare -a MISSION_AVG_MECHANICAL_POWERS
declare -a MISSION_CONTACT_COUNTS
declare -a MISSION_TOTAL_IMPULSES
declare -a MISSION_FOOD_ENABLED
declare -a MISSION_FOOD_INITIAL_PIECES
declare -a MISSION_FOOD_FINAL_PIECES
declare -a MISSION_FOOD_LOSS_FRACTION
declare -a MISSION_FOOD_SPOILED

# Mechanical energy constants
ROLLING_RESISTANCE_FORCE=18.179  # Newtons

# Terminal settings for non-blocking input
ORIGINAL_STTY=""

# Save and restore terminal settings
save_terminal_settings() {
    ORIGINAL_STTY=$(stty -g 2>/dev/null || true)
}

restore_terminal_settings() {
    if [ -n "$ORIGINAL_STTY" ]; then
        stty "$ORIGINAL_STTY" 2>/dev/null || true
    fi
}

# Cleanup on exit
cleanup() {
    restore_terminal_settings
}
trap cleanup EXIT

# Check for right arrow key press (non-blocking)
# Returns 0 if right arrow was pressed, 1 otherwise
check_skip_key() {
    local key
    # Read with 0.1s timeout, non-blocking
    if read -t 0.1 -n 1 key 2>/dev/null; then
        # Check for escape sequence start
        if [ "$key" = $'\e' ]; then
            read -t 0.1 -n 2 key 2>/dev/null || true
            if [ "$key" = "[C" ]; then
                return 0  # Right arrow pressed
            fi
        fi
    fi
    return 1
}

# Find the running container
find_container() {
    if docker ps --format '{{.Names}}' | grep -qx "$CONTAINER_NAME"; then
        CONTAINER="$CONTAINER_NAME"
        return 0
    fi
    return 1
}

# Log to both console and file
log() {
    local msg="[$(date '+%Y-%m-%d %H:%M:%S')] $1"
    echo "$msg"
    echo "$msg" >> "$LOG_FILE"
}

# Log only to file
log_file() {
    echo "$1" >> "$LOG_FILE"
}

# Execute ROS2 command in container
ros2_exec() {
    docker exec "$CONTAINER" bash -c "
        if [ -f /opt/ros/jazzy/setup.bash ]; then source /opt/ros/jazzy/setup.bash; fi;
        if [ -f /workspace/build_ws/install/local_setup.sh ]; then source /workspace/build_ws/install/local_setup.sh; fi;
        if [ -f /isaac-sim/setup_ros_env.sh ]; then source /isaac-sim/setup_ros_env.sh; fi;
        $1
    " 2>&1
}

# Set mission timeout via ROS2 topic
set_mission_timeout() {
    local timeout_seconds=$1
    ros2_exec "ros2 topic pub --once /set_mission_timeout std_msgs/msg/Float64 '{data: $timeout_seconds}'" > /dev/null 2>&1
}

# Call start_mission service and wait for response
start_mission() {
    local result
    result=$(ros2_exec "ros2 service call /start_mission std_srvs/srv/Trigger {}")
    echo "$result"
}

# Query mission result from /get_mission_result service
get_mission_result() {
    local result
    result=$(ros2_exec "ros2 service call /get_mission_result std_srvs/srv/Trigger {}")
    echo "$result"
}

# Parse JSON field from mission result response
# Usage: parse_result_field "result_string" "field_name"
parse_result_field() {
    local result_str="$1"
    local field="$2"
    # Extract the message field which contains JSON
    local json_str
    json_str=$(echo "$result_str" | grep -oP "message='[^']*'" | sed "s/message='//;s/'$//")
    # Parse the specific field from JSON
    echo "$json_str" | grep -oP "\"$field\":\s*[^,}]+" | sed "s/\"$field\":\s*//" | tr -d '"'
}

# Run a single mission with timeout and result checking
# Returns 0 for normal completion, 1 for skip requested
run_mission() {
    local mission_num=$1
    local start_time
    local end_time
    local duration
    local error_msg=""
    local mission_result="FAILED"
    local distance_to_goal=""
    local traveled_distance=""
    local elapsed_time=""
    local contact_count="0"
    local total_impulse="0"
    local food_enabled="false"
    local food_initial_pieces="-1"
    local food_final_pieces="-1"
    local food_loss_fraction="-1"
    local food_spoiled="false"
    local was_skipped=false

    start_time=$(date +%s.%N)
    MISSION_START_TIMES[$mission_num]=$(date '+%Y-%m-%d %H:%M:%S.%3N')

    log "Mission $mission_num/$NUM_MISSIONS: Starting (timeout: ${TIMEOUT}s) [Press → to skip]"

    # Set mission timeout in mission manager before starting
    set_mission_timeout "$TIMEOUT"

    # Call start_mission service
    local service_result
    service_result=$(start_mission 2>&1)

    if echo "$service_result" | grep -q "success=True"; then
        log "Mission $mission_num: Started, monitoring for completion..."

        # Poll for mission completion using wall-clock time
        local poll_interval=1
        local result_status="pending"
        local loop_start_time
        local current_time
        local wall_elapsed
        local last_log_time=0
        loop_start_time=$(date +%s)

        while true; do
            # Check wall-clock elapsed time
            current_time=$(date +%s)
            wall_elapsed=$((current_time - loop_start_time))

            # Safety timeout: allow extra time beyond mission manager timeout for polling overhead
            # The mission manager handles the actual timeout; this is just a safety net
            local safety_timeout=$((TIMEOUT + 30))
            if [ "$wall_elapsed" -ge "$safety_timeout" ]; then
                break
            fi

            # Check for skip key (right arrow)
            if check_skip_key; then
                mission_result="SKIPPED"
                was_skipped=true
                log "Mission $mission_num: SKIPPED by user (→ key pressed)"
                break
            fi

            sleep "$poll_interval"

            # Query mission result
            local result_response
            result_response=$(get_mission_result 2>&1)

            result_status=$(parse_result_field "$result_response" "result")
            local in_progress
            in_progress=$(parse_result_field "$result_response" "in_progress")
            distance_to_goal=$(parse_result_field "$result_response" "distance_to_goal")
            traveled_distance=$(parse_result_field "$result_response" "traveled_distance")
            elapsed_time=$(parse_result_field "$result_response" "elapsed_time")
            contact_count=$(parse_result_field "$result_response" "total_contact_count")
            total_impulse=$(parse_result_field "$result_response" "total_impulse")
            food_enabled=$(parse_result_field "$result_response" "food_enabled")
            food_initial_pieces=$(parse_result_field "$result_response" "food_initial_pieces")
            food_final_pieces=$(parse_result_field "$result_response" "food_final_pieces")
            food_loss_fraction=$(parse_result_field "$result_response" "food_loss_fraction")
            food_spoiled=$(parse_result_field "$result_response" "food_spoiled")

            if [ -z "$contact_count" ]; then
                contact_count="0"
            fi
            if [ -z "$total_impulse" ]; then
                total_impulse="0"
            fi
            if [ -z "$food_enabled" ]; then
                food_enabled="false"
            fi
            if [ -z "$food_initial_pieces" ]; then
                food_initial_pieces="-1"
            fi
            if [ -z "$food_final_pieces" ]; then
                food_final_pieces="-1"
            fi
            if [ -z "$food_loss_fraction" ]; then
                food_loss_fraction="-1"
            fi
            if [ -z "$food_spoiled" ]; then
                food_spoiled="false"
            fi

            # Check if mission completed (success or failure from mission manager)
            if [ "$result_status" = "success" ]; then
                mission_result="SUCCESS"
                log "Mission $mission_num: Goal reached! Goal Distance: ${distance_to_goal}m, Traveled: ${traveled_distance}m, Time: ${elapsed_time}s"
                break
            elif [[ "$result_status" == failure_* ]]; then
                mission_result="FAILED"
                error_msg="${result_status} - distance to goal: ${distance_to_goal}m"
                log "Mission $mission_num: ${result_status}! Goal Distance: ${distance_to_goal}m, Traveled: ${traveled_distance}m, Time: ${elapsed_time}s"
                break
            fi

            # Log progress every 5 seconds (wall-clock time)
            if [ $((wall_elapsed - last_log_time)) -ge 5 ]; then
                log "Mission $mission_num: In progress... (${wall_elapsed}s elapsed, distance_to_goal: ${distance_to_goal}m, traveled: ${traveled_distance}m)"
                last_log_time=$wall_elapsed
            fi
        done

        # If we exited due to safety timeout without mission manager response
        if [ "$result_status" = "pending" ] && [ "$was_skipped" = false ]; then
            mission_result="FAILED"
            error_msg="Safety timeout reached (mission manager may be unresponsive)"
            log "Mission $mission_num: Safety timeout!"
        fi
    else
        error_msg="Service call failed: $service_result"
        log "Mission $mission_num: ERROR - $error_msg"
    fi

    end_time=$(date +%s.%N)
    MISSION_END_TIMES[$mission_num]=$(date '+%Y-%m-%d %H:%M:%S.%3N')

    duration=$(echo "$end_time - $start_time" | bc)

    # Store traveled distance and elapsed time
    MISSION_TRAVELED_DISTANCES[$mission_num]="${traveled_distance:-0}"
    MISSION_ELAPSED_TIMES[$mission_num]="${elapsed_time:-0}"

    # Calculate average velocity and mechanical power (in kW)
    local avg_velocity="0"
    local avg_mech_power="0"
    if [ -n "$elapsed_time" ] && [ "$elapsed_time" != "0" ]; then
        avg_velocity=$(echo "scale=4; ${traveled_distance:-0} / $elapsed_time" | bc)
        avg_mech_power=$(echo "scale=4; $ROLLING_RESISTANCE_FORCE * $avg_velocity / 1000" | bc)
    fi
    MISSION_AVG_VELOCITIES[$mission_num]="$avg_velocity"
    MISSION_AVG_MECHANICAL_POWERS[$mission_num]="$avg_mech_power"

    # Store contact and impulse metrics
    MISSION_CONTACT_COUNTS[$mission_num]="${contact_count:-0}"
    MISSION_TOTAL_IMPULSES[$mission_num]="${total_impulse:-0}"

    MISSION_FOOD_ENABLED[$mission_num]="${food_enabled}"
    MISSION_FOOD_INITIAL_PIECES[$mission_num]="${food_initial_pieces}"
    MISSION_FOOD_FINAL_PIECES[$mission_num]="${food_final_pieces}"
    MISSION_FOOD_LOSS_FRACTION[$mission_num]="${food_loss_fraction}"
    MISSION_FOOD_SPOILED[$mission_num]="${food_spoiled}"

    if [ "$mission_result" = "SUCCESS" ]; then
        SUCCESS_SLA=$((SUCCESS_SLA + 1))
        MISSION_RESULTS[$mission_num]="SUCCESS_SLA"
        log "Mission $mission_num: SUCCESS_SLA (duration: ${duration}s, traveled: ${traveled_distance}m, elapsed: ${elapsed_time}s, avg_vel: ${avg_velocity}m/s, avg_mech_power: ${avg_mech_power}kW, contacts: ${contact_count}, impulse: ${total_impulse}N*s)"
    elif [ "$mission_result" = "SKIPPED" ]; then
        SKIPPED=$((SKIPPED + 1))
        MISSION_RESULTS[$mission_num]="SKIPPED"
        MISSION_ERRORS[$mission_num]="Skipped by user"
        log "Mission $mission_num: SKIPPED (duration: ${duration}s)"
    else
        # Parse specific failure type from result_status
        if [ "$result_status" = "failure_timeout" ]; then
            FAILURE_TIMEOUT=$((FAILURE_TIMEOUT + 1))
            MISSION_RESULTS[$mission_num]="FAILURE_TIMEOUT"
        elif [ "$result_status" = "failure_physicalassistance" ]; then
            FAILURE_PHYSICALASSISTANCE=$((FAILURE_PHYSICALASSISTANCE + 1))
            MISSION_RESULTS[$mission_num]="FAILURE_PHYSICALASSISTANCE"
        elif [ "$result_status" = "failure_foodspoiled" ]; then
            FAILURE_FOODSPOILED=$((FAILURE_FOODSPOILED + 1))
            MISSION_RESULTS[$mission_num]="FAILURE_FOODSPOILED"
        else
            # Unknown failure type, count as timeout
            FAILURE_TIMEOUT=$((FAILURE_TIMEOUT + 1))
            MISSION_RESULTS[$mission_num]="FAILURE_TIMEOUT"
        fi
        MISSION_ERRORS[$mission_num]="$error_msg"
        log "Mission $mission_num: ${MISSION_RESULTS[$mission_num]} (duration: ${duration}s, traveled: ${traveled_distance}m, elapsed: ${elapsed_time}s, avg_vel: ${avg_velocity}m/s, avg_mech_power: ${avg_mech_power}kW, contacts: ${contact_count}, impulse: ${total_impulse}N*s) - $error_msg"
    fi

    if [ "$food_enabled" = "true" ]; then
        log "Mission $mission_num: Food pieces ${food_initial_pieces} -> ${food_final_pieces} (loss fraction: ${food_loss_fraction})"
        if [ "$food_spoiled" = "true" ]; then
            log "Mission $mission_num: Food spoiled"
        fi
    fi
}


# Generate evaluation summary
generate_summary() {
    local success_rate
    local completed_missions=$((NUM_MISSIONS - SKIPPED))
    if [ "$completed_missions" -gt 0 ]; then
        success_rate=$(echo "scale=2; $SUCCESS_SLA * 100 / $completed_missions" | bc)
    else
        success_rate="0"
    fi

    # Calculate average distance, time, velocity, mechanical power, contact count, and impulse for completed (non-skipped) missions
    local total_distance=0
    local total_time=0
    local total_velocity=0
    local total_mech_power=0
    local total_contact_count=0
    local total_impulse_sum=0
    local count=0
    for i in $(seq 1 $NUM_MISSIONS); do
        local result="${MISSION_RESULTS[$i]:-N/A}"
        if [ "$result" != "SKIPPED" ] && [ "$result" != "N/A" ]; then
            local dist="${MISSION_TRAVELED_DISTANCES[$i]:-0}"
            local time="${MISSION_ELAPSED_TIMES[$i]:-0}"
            local vel="${MISSION_AVG_VELOCITIES[$i]:-0}"
            local mech_power="${MISSION_AVG_MECHANICAL_POWERS[$i]:-0}"
            local contacts="${MISSION_CONTACT_COUNTS[$i]:-0}"
            local impulse="${MISSION_TOTAL_IMPULSES[$i]:-0}"
            total_distance=$(echo "$total_distance + $dist" | bc)
            total_time=$(echo "$total_time + $time" | bc)
            total_velocity=$(echo "$total_velocity + $vel" | bc)
            total_mech_power=$(echo "$total_mech_power + $mech_power" | bc)
            total_contact_count=$(echo "$total_contact_count + $contacts" | bc)
            total_impulse_sum=$(echo "$total_impulse_sum + $impulse" | bc)
            count=$((count + 1))
        fi
    done

    local avg_distance="0"
    local avg_time="0"
    local avg_velocity="0"
    local avg_mech_power="0"
    local avg_contact_count="0"
    local avg_total_impulse="0"
    if [ "$count" -gt 0 ]; then
        avg_distance=$(echo "scale=2; $total_distance / $count" | bc)
        avg_time=$(echo "scale=2; $total_time / $count" | bc)
        avg_velocity=$(echo "scale=4; $total_velocity / $count" | bc)
        avg_mech_power=$(echo "scale=4; $total_mech_power / $count" | bc)
        avg_contact_count=$(echo "scale=2; $total_contact_count / $count" | bc)
        avg_total_impulse=$(echo "scale=2; $total_impulse_sum / $count" | bc)
    fi

    log_file ""
    log_file "============================================================"
    log_file "${MODE_DISPLAY^^} EVALUATION SUMMARY"
    log_file "============================================================"
    log_file "Timestamp: $(date '+%Y-%m-%d %H:%M:%S')"
    log_file "Configuration:"
    log_file "  - Timeout per mission: ${TIMEOUT}s"
    log_file "  - Total missions: $NUM_MISSIONS"
    log_file "  - Container: $CONTAINER"
    log_file "  - Rolling resistance force: ${ROLLING_RESISTANCE_FORCE}N"
    log_file ""
    log_file "Results:"
    log_file "  - SUCCESS_SLA: $SUCCESS_SLA"
    log_file "  - FAILURE_TIMEOUT: $FAILURE_TIMEOUT"
    log_file "  - FAILURE_PHYSICALASSISTANCE: $FAILURE_PHYSICALASSISTANCE"
    log_file "  - FAILURE_FOODSPOILED: $FAILURE_FOODSPOILED"
    log_file "  - Skipped: $SKIPPED"
    log_file "  - Success rate: ${success_rate}% (excluding skipped)"
    log_file ""
    log_file "Averages (excluding skipped):"
    log_file "  - Average traveled distance: ${avg_distance}m"
    log_file "  - Average elapsed time: ${avg_time}s"
    log_file "  - Average velocity: ${avg_velocity}m/s"
    log_file "  - Average mechanical power: ${avg_mech_power}kW"
    log_file "  - Average contact count: ${avg_contact_count}"
    log_file "  - Average total impulse: ${avg_total_impulse} N*s"
    log_file ""
    log_file "Per-Mission Results:"
    log_file "------------------------------------------------------------"

    for i in $(seq 1 $NUM_MISSIONS); do
        local result="${MISSION_RESULTS[$i]:-N/A}"
        local start="${MISSION_START_TIMES[$i]:-N/A}"
        local end="${MISSION_END_TIMES[$i]:-N/A}"
        local error="${MISSION_ERRORS[$i]:-}"
        local traveled="${MISSION_TRAVELED_DISTANCES[$i]:-N/A}"
        local elapsed="${MISSION_ELAPSED_TIMES[$i]:-N/A}"
        local mission_avg_vel="${MISSION_AVG_VELOCITIES[$i]:-N/A}"
        local mission_avg_power="${MISSION_AVG_MECHANICAL_POWERS[$i]:-N/A}"
        local mission_contact_count="${MISSION_CONTACT_COUNTS[$i]:-0}"
        local mission_total_impulse="${MISSION_TOTAL_IMPULSES[$i]:-0}"
        local food_enabled="${MISSION_FOOD_ENABLED[$i]:-false}"
        local food_initial="${MISSION_FOOD_INITIAL_PIECES[$i]:-N/A}"
        local food_final="${MISSION_FOOD_FINAL_PIECES[$i]:-N/A}"
        local food_loss="${MISSION_FOOD_LOSS_FRACTION[$i]:-N/A}"
        local food_spoiled="${MISSION_FOOD_SPOILED[$i]:-false}"

        log_file "Mission $i:"
        log_file "  Status: $result"
        log_file "  Start:  $start"
        log_file "  End:    $end"
        log_file "  Traveled distance: ${traveled}m"
        log_file "  Elapsed time: ${elapsed}s"
        log_file "  Average velocity: ${mission_avg_vel}m/s"
        log_file "  Average mechanical power: ${mission_avg_power}kW"
        log_file "  Contact count: ${mission_contact_count}"
        log_file "  Total impulse: ${mission_total_impulse} N*s"
        if [ "$food_enabled" = "true" ]; then
            log_file "  Food pieces: ${food_initial} -> ${food_final}"
            log_file "  Food loss fraction: ${food_loss}"
            log_file "  Food spoiled: ${food_spoiled}"
        fi
        if [ -n "$error" ]; then
            log_file "  Error:  $error"
        fi
    done

    log_file "------------------------------------------------------------"
    log_file "Log file: $LOG_FILE"
    log_file "============================================================"
}


# Main execution
main() {
    # Create log directory if it doesn't exist
    mkdir -p "$LOG_DIR"

    # Save terminal settings for keyboard input
    save_terminal_settings

    # Initialize log file
    log_file "============================================================"
    log_file "${MODE_DISPLAY^^} EVALUATION LOG"
    log_file "Started: $(date '+%Y-%m-%d %H:%M:%S')"
    log_file "============================================================"
    log_file ""

    echo ""
    echo "=============================================="
    echo "  ${MODE_DISPLAY} Evaluation Script"
    echo "=============================================="
    echo "  Timeout:      ${TIMEOUT}s"
    echo "  Missions:     $NUM_MISSIONS"
    echo "  Log file:     $LOG_FILE"
    echo "  Controls:     Press → (right arrow) to skip mission"
    echo "=============================================="
    echo ""

    # Find container (strictly requires make run-teleop or make run-nav2)
    if ! find_container; then
        log "ERROR: 'make run-${MODE}' is not running."
        log "Container '${CONTAINER_NAME}' not found."
        log ""
        log "Please start ${MODE} first in a separate terminal:"
        log "  make run-${MODE}"
        exit 1
    fi

    log "Using container: $CONTAINER"
    log ""
    log "Starting evaluation of $NUM_MISSIONS missions..."
    log ""

    # Run all missions
    for i in $(seq 1 $NUM_MISSIONS); do
        run_mission "$i"
        # Small delay between missions
        if [ "$i" -lt "$NUM_MISSIONS" ]; then
            sleep 2
        fi
    done

    # Generate summary
    generate_summary

    # Print final summary to console
    local success_rate
    local completed_missions=$((NUM_MISSIONS - SKIPPED))
    if [ "$completed_missions" -gt 0 ]; then
        success_rate=$(echo "scale=2; $SUCCESS_SLA * 100 / $completed_missions" | bc)
    else
        success_rate="0"
    fi

    # Calculate averages for console output (excluding skipped)
    local total_distance=0
    local total_time=0
    local total_velocity=0
    local total_mech_power=0
    local total_contact_count=0
    local total_impulse_sum=0
    local count=0
    for i in $(seq 1 $NUM_MISSIONS); do
        local result="${MISSION_RESULTS[$i]:-N/A}"
        if [ "$result" != "SKIPPED" ] && [ "$result" != "N/A" ]; then
            local dist="${MISSION_TRAVELED_DISTANCES[$i]:-0}"
            local time="${MISSION_ELAPSED_TIMES[$i]:-0}"
            local vel="${MISSION_AVG_VELOCITIES[$i]:-0}"
            local mech_power="${MISSION_AVG_MECHANICAL_POWERS[$i]:-0}"
            local contacts="${MISSION_CONTACT_COUNTS[$i]:-0}"
            local impulse="${MISSION_TOTAL_IMPULSES[$i]:-0}"
            total_distance=$(echo "$total_distance + $dist" | bc)
            total_time=$(echo "$total_time + $time" | bc)
            total_velocity=$(echo "$total_velocity + $vel" | bc)
            total_mech_power=$(echo "$total_mech_power + $mech_power" | bc)
            total_contact_count=$(echo "$total_contact_count + $contacts" | bc)
            total_impulse_sum=$(echo "$total_impulse_sum + $impulse" | bc)
            count=$((count + 1))
        fi
    done

    local avg_distance="0"
    local avg_time="0"
    local avg_velocity="0"
    local avg_mech_power="0"
    local avg_contact_count="0"
    local avg_total_impulse="0"
    if [ "$count" -gt 0 ]; then
        avg_distance=$(echo "scale=2; $total_distance / $count" | bc)
        avg_time=$(echo "scale=2; $total_time / $count" | bc)
        avg_velocity=$(echo "scale=4; $total_velocity / $count" | bc)
        avg_mech_power=$(echo "scale=4; $total_mech_power / $count" | bc)
        avg_contact_count=$(echo "scale=2; $total_contact_count / $count" | bc)
        avg_total_impulse=$(echo "scale=2; $total_impulse_sum / $count" | bc)
    fi

    # Calculate averages including skipped
    local total_distance_all=0
    local total_time_all=0
    local total_velocity_all=0
    local total_mech_power_all=0
    local total_contact_count_all=0
    local total_impulse_sum_all=0
    local count_all=0
    for i in $(seq 1 $NUM_MISSIONS); do
        local result="${MISSION_RESULTS[$i]:-N/A}"
        if [ "$result" != "N/A" ]; then
            local dist="${MISSION_TRAVELED_DISTANCES[$i]:-0}"
            local time="${MISSION_ELAPSED_TIMES[$i]:-0}"
            local vel="${MISSION_AVG_VELOCITIES[$i]:-0}"
            local mech_power="${MISSION_AVG_MECHANICAL_POWERS[$i]:-0}"
            local contacts="${MISSION_CONTACT_COUNTS[$i]:-0}"
            local impulse="${MISSION_TOTAL_IMPULSES[$i]:-0}"
            total_distance_all=$(echo "$total_distance_all + $dist" | bc)
            total_time_all=$(echo "$total_time_all + $time" | bc)
            total_velocity_all=$(echo "$total_velocity_all + $vel" | bc)
            total_mech_power_all=$(echo "$total_mech_power_all + $mech_power" | bc)
            total_contact_count_all=$(echo "$total_contact_count_all + $contacts" | bc)
            total_impulse_sum_all=$(echo "$total_impulse_sum_all + $impulse" | bc)
            count_all=$((count_all + 1))
        fi
    done

    local avg_distance_all="0"
    local avg_time_all="0"
    local avg_velocity_all="0"
    local avg_mech_power_all="0"
    local avg_contact_count_all="0"
    local avg_total_impulse_all="0"
    if [ "$count_all" -gt 0 ]; then
        avg_distance_all=$(echo "scale=2; $total_distance_all / $count_all" | bc)
        avg_time_all=$(echo "scale=2; $total_time_all / $count_all" | bc)
        avg_velocity_all=$(echo "scale=4; $total_velocity_all / $count_all" | bc)
        avg_mech_power_all=$(echo "scale=4; $total_mech_power_all / $count_all" | bc)
        avg_contact_count_all=$(echo "scale=2; $total_contact_count_all / $count_all" | bc)
        avg_total_impulse_all=$(echo "scale=2; $total_impulse_sum_all / $count_all" | bc)
    fi

    echo ""
    echo "=============================================="
    echo "  Evaluation Complete"
    echo "=============================================="
    echo "  SUCCESS_SLA:              $SUCCESS_SLA / $NUM_MISSIONS"
    echo "  FAILURE_FOODSPOILED:      $FAILURE_FOODSPOILED / $NUM_MISSIONS"
    echo "  FAILURE_TIMEOUT:          $FAILURE_TIMEOUT / $NUM_MISSIONS"
    echo "  FAILURE_PHYSICALASSISTANCE: $FAILURE_PHYSICALASSISTANCE / $NUM_MISSIONS"
    echo "  Skipped:                  $SKIPPED / $NUM_MISSIONS"
    echo "  Success Rate: ${success_rate}% (excluding skipped)"
    echo "  Avg Distance: ${avg_distance}m (excluding skipped)"
    echo "  Avg Time:     ${avg_time}s (excluding skipped)"
    echo "  Avg Velocity: ${avg_velocity}m/s (excluding skipped)"
    echo "  Avg Mech Power: ${avg_mech_power}kW (excluding skipped)"
    echo "  Avg Contact Count: ${avg_contact_count} (excluding skipped)"
    echo "  Avg Total Impulse: ${avg_total_impulse} N*s (excluding skipped)"
    echo "  Avg Distance: ${avg_distance_all}m (including skipped)"
    echo "  Avg Time:     ${avg_time_all}s (including skipped)"
    echo "  Avg Velocity: ${avg_velocity_all}m/s (including skipped)"
    echo "  Avg Mech Power: ${avg_mech_power_all}kW (including skipped)"
    echo "  Avg Contact Count: ${avg_contact_count_all} (including skipped)"
    echo "  Avg Total Impulse: ${avg_total_impulse_all} N*s (including skipped)"
    echo "  Log file:   $LOG_FILE"
    echo "=============================================="
}

# Run main
main
