#!/bin/bash
# Nav2 Evaluation Script
# Runs consecutive missions and generates comprehensive evaluation logs
#
# Usage: ./eval_nav2.sh [TIMEOUT] [NUM_MISSIONS]
#   TIMEOUT: Mission timeout in seconds (default: 20)
#   NUM_MISSIONS: Number of missions to run (default: 10)
#
# Requires: A running nav2 instance (make run-nav2)

set -e

# Configuration
TIMEOUT="${1:-20}"
NUM_MISSIONS="${2:-10}"
CONTAINER=""
LOG_DIR="${LOG_DIR:-./logs}"
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
LOG_FILE="${LOG_DIR}/nav2_evaluation_${TIMESTAMP}.log"

# Statistics tracking
SUCCESSFUL=0
FAILED=0
declare -a MISSION_RESULTS
declare -a MISSION_START_TIMES
declare -a MISSION_END_TIMES
declare -a MISSION_ERRORS

# Find the running nav2 container (strictly requires costnav-ros2-nav2)
find_container() {
    if docker ps --format '{{.Names}}' | grep -qx "costnav-ros2-nav2"; then
        CONTAINER="costnav-ros2-nav2"
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
run_mission() {
    local mission_num=$1
    local start_time
    local end_time
    local duration
    local error_msg=""
    local mission_result="FAILED"
    local distance_to_goal=""

    start_time=$(date +%s.%N)
    MISSION_START_TIMES[$mission_num]=$(date '+%Y-%m-%d %H:%M:%S.%3N')

    log "Mission $mission_num/$NUM_MISSIONS: Starting (timeout: ${TIMEOUT}s)"

    # Call start_mission service
    local service_result
    service_result=$(start_mission 2>&1)

    if echo "$service_result" | grep -q "success=True"; then
        log "Mission $mission_num: Started, monitoring for completion..."

        # Poll for mission completion or timeout
        local poll_interval=1
        local elapsed=0
        local result_status="pending"

        while [ "$elapsed" -lt "$TIMEOUT" ]; do
            sleep "$poll_interval"
            elapsed=$((elapsed + poll_interval))

            # Query mission result
            local result_response
            result_response=$(get_mission_result 2>&1)

            result_status=$(parse_result_field "$result_response" "result")
            local in_progress
            in_progress=$(parse_result_field "$result_response" "in_progress")
            distance_to_goal=$(parse_result_field "$result_response" "distance_to_goal")

            # Check if mission completed (success or failure)
            if [ "$result_status" = "success" ]; then
                mission_result="SUCCESS"
                log "Mission $mission_num: Goal reached! Distance: ${distance_to_goal}m"
                break
            elif [ "$result_status" = "failure" ]; then
                mission_result="FAILED"
                error_msg="Timeout - distance to goal: ${distance_to_goal}m"
                log "Mission $mission_num: Timeout! Distance: ${distance_to_goal}m"
                break
            fi

            # Log progress every 5 seconds
            if [ $((elapsed % 5)) -eq 0 ]; then
                log "Mission $mission_num: In progress... (${elapsed}s elapsed, distance_to_goal: ${distance_to_goal}m)"
            fi
        done

        # If we exited the loop due to our timeout (not mission manager's), mark as failed
        if [ "$result_status" = "pending" ]; then
            mission_result="FAILED"
            error_msg="Evaluation timeout reached"
            log "Mission $mission_num: Evaluation timeout!"
        fi
    else
        error_msg="Service call failed: $service_result"
        log "Mission $mission_num: ERROR - $error_msg"
    fi

    end_time=$(date +%s.%N)
    MISSION_END_TIMES[$mission_num]=$(date '+%Y-%m-%d %H:%M:%S.%3N')

    duration=$(echo "$end_time - $start_time" | bc)

    if [ "$mission_result" = "SUCCESS" ]; then
        SUCCESSFUL=$((SUCCESSFUL + 1))
        MISSION_RESULTS[$mission_num]="SUCCESS"
        log "Mission $mission_num: SUCCESS (duration: ${duration}s, distance: ${distance_to_goal}m)"
    else
        FAILED=$((FAILED + 1))
        MISSION_RESULTS[$mission_num]="FAILED"
        MISSION_ERRORS[$mission_num]="$error_msg"
        log "Mission $mission_num: FAILED (duration: ${duration}s) - $error_msg"
    fi
}

# Generate evaluation summary
generate_summary() {
    local success_rate
    if [ "$NUM_MISSIONS" -gt 0 ]; then
        success_rate=$(echo "scale=2; $SUCCESSFUL * 100 / $NUM_MISSIONS" | bc)
    else
        success_rate="0"
    fi

    log_file ""
    log_file "============================================================"
    log_file "NAV2 EVALUATION SUMMARY"
    log_file "============================================================"
    log_file "Timestamp: $(date '+%Y-%m-%d %H:%M:%S')"
    log_file "Configuration:"
    log_file "  - Timeout per mission: ${TIMEOUT}s"
    log_file "  - Total missions: $NUM_MISSIONS"
    log_file "  - Container: $CONTAINER"
    log_file ""
    log_file "Results:"
    log_file "  - Successful missions: $SUCCESSFUL"
    log_file "  - Failed missions: $FAILED"
    log_file "  - Success rate: ${success_rate}%"
    log_file ""
    log_file "Per-Mission Results:"
    log_file "------------------------------------------------------------"

    for i in $(seq 1 $NUM_MISSIONS); do
        local result="${MISSION_RESULTS[$i]:-N/A}"
        local start="${MISSION_START_TIMES[$i]:-N/A}"
        local end="${MISSION_END_TIMES[$i]:-N/A}"
        local error="${MISSION_ERRORS[$i]:-}"

        log_file "Mission $i:"
        log_file "  Status: $result"
        log_file "  Start:  $start"
        log_file "  End:    $end"
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

    # Initialize log file
    log_file "============================================================"
    log_file "NAV2 EVALUATION LOG"
    log_file "Started: $(date '+%Y-%m-%d %H:%M:%S')"
    log_file "============================================================"
    log_file ""

    echo ""
    echo "=============================================="
    echo "  Nav2 Evaluation Script"
    echo "=============================================="
    echo "  Timeout:      ${TIMEOUT}s"
    echo "  Missions:     $NUM_MISSIONS"
    echo "  Log file:     $LOG_FILE"
    echo "=============================================="
    echo ""

    # Find container (strictly requires make run-nav2)
    if ! find_container; then
        log "ERROR: 'make run-nav2' is not running."
        log "Container 'costnav-ros2-nav2' not found."
        log ""
        log "Please start nav2 first in a separate terminal:"
        log "  make run-nav2"
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
    if [ "$NUM_MISSIONS" -gt 0 ]; then
        success_rate=$(echo "scale=2; $SUCCESSFUL * 100 / $NUM_MISSIONS" | bc)
    else
        success_rate="0"
    fi

    echo ""
    echo "=============================================="
    echo "  Evaluation Complete"
    echo "=============================================="
    echo "  Successful: $SUCCESSFUL / $NUM_MISSIONS"
    echo "  Failed:     $FAILED / $NUM_MISSIONS"
    echo "  Success Rate: ${success_rate}%"
    echo "  Log file:   $LOG_FILE"
    echo "=============================================="
}

# Run main
main

