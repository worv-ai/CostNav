#!/bin/bash
# =============================================================================
# 3DGRUT Multi-Experiment Batch Launcher — zed_pipeline_round_2026_0222
# =============================================================================
# 데이터: /mnt/harbor/users/samwoo/zed_trial_3/zed_pipeline_round_2026_0222_colmap
#         54장 이미지, 5662 COLMAP sparse points, mesh.ply 초기화
#
# 사용법:
#   bash submit_experiments_round2.sh                   # 전체 실험 제출
#   bash submit_experiments_round2.sh --dry-run         # 제출 없이 미리보기
#   bash submit_experiments_round2.sh --only mcmc_mild gs_strong
#   bash submit_experiments_round2.sh --group antighost best
# =============================================================================
set -uo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SBATCH_SCRIPT="$SCRIPT_DIR/slurm_3dgrut_antighost.sbatch"

# ★ zed_pipeline_round_2026_0222 전용 경로 ★
COLMAP_PATH="/mnt/harbor/users/samwoo/zed_trial_3/zed_pipeline_round_2026_0222_colmap"
MESH_PATH="/mnt/harbor/users/samwoo/zed_trial_3/zed_pipeline_round_2026_0222/nvblox/mesh.ply"
OUTPUT_BASE="/mnt/harbor/users/samwoo/3dgrut_output_round2"
PARTITION="${PARTITION:-h100}"

# ★ 사용할 노드 고정 (3대만 사용) ★
ALLOWED_NODES="${ALLOWED_NODES:-DGX-H100-1,DGX-H100-2,DGX-H100-3}"
EXCLUDE_NODES="${EXCLUDE_NODES:-DGX-H100-4,DGX-H100-5,DGX-H100-6,DGX-H100-7,DGX-H100-8,DGX-H100-9,DGX-H100-10,DGX-H100-11,DGX-H100-12}"

# ★ 54장 데이터셋 전용 설정 ★
# - test_split_interval=6 → test ~9장, train ~45장
# - iterations: 50k (base), 25k (short), 75k (long)
# - MAX_GAUSSIANS: 500k~150k (중규모)
# - 이미지당 학습 횟수: 50k/45train ≈ 1,111회
TEST_SPLIT_INTERVAL=6

# ★ 노드 3대 × GPU 8개 = 최대 24 job 동시 실행 ★
# 의존성 체인 없이 전부 제출 → SLURM이 GPU 리소스 기반으로 스케줄링

RED='\033[0;31m'; GRN='\033[0;32m'; YLW='\033[1;33m'
BLU='\033[0;34m'; CYN='\033[0;36m'; BLD='\033[1m'; NC='\033[0m'
log_ok()    { echo -e "${GRN}[OK]${NC}    $*"; }
log_warn()  { echo -e "${YLW}[WARN]${NC}  $*"; }
log_err()   { echo -e "${RED}[ERROR]${NC} $*"; }
log_title() { echo -e "\n${BLD}${CYN}$*${NC}"; }

# --- 노드 리소스 확인 ---
AVAILABLE_NODES=()
check_node_resources() {
    log_title "========== SLURM 노드 리소스 현황 =========="
    echo -e "\n${BLD}[전체 노드 상태]${NC}"
    sinfo -p "$PARTITION" --format="%20N %10T %10c %15G %10e %10m" --noheader 2>/dev/null \
        | sort || { log_err "sinfo 실행 실패"; return 1; }
    echo -e "\n${BLD}[사용 노드 (고정)]${NC}"
    log_ok "허용 노드: $ALLOWED_NODES"
    log_ok "동시 실행: 최대 24개 (3노드 × 8GPU, SLURM 자동 스케줄링)"
    IFS=',' read -ra AVAILABLE_NODES <<< "$ALLOWED_NODES"
    echo -e "\n${BLD}[현재 실행 중인 작업]${NC}"
    squeue -u "$USER" -o "%10i %20j %10T %15N %10M" --noheader 2>/dev/null || true
    echo ""
}

# --- 실험 정의 (33개) ---
declare -a EXPERIMENTS=()
declare -A EXP_EXTRA=()

define_experiments() {
    # ★ 54장 데이터셋 스케일링 ★
    # iterations: 50k (base), 25k (short), 75k (long)
    # MAX_GAUSSIANS: 500k~150k (중규모, mesh.ply 초기화 고려)

    EXPERIMENTS=(
        # GROUP A: Anti-Ghosting Sweep (10)
        "mcmc_soft|apps/cusfm_3dgut_mcmc|50000|0.00014|500000|true|true|0.008|0.05|0.005|antighost"
        "mcmc_mild|apps/cusfm_3dgut_mcmc|50000|0.00012|400000|true|true|0.01|0.1|0.01|antighost"
        "mcmc_moderate|apps/cusfm_3dgut_mcmc|50000|0.0001|350000|true|true|0.01|0.1|0.01|antighost"
        "mcmc_strong|apps/cusfm_3dgut_mcmc|50000|0.0001|300000|true|true|0.01|0.1|0.01|antighost"
        "mcmc_aggressive|apps/cusfm_3dgut_mcmc|50000|0.00008|150000|true|true|0.015|0.2|0.02|antighost"
        "gs_soft|apps/cusfm_3dgut|50000|0.00014|500000|true|true|0.008|0.05|0.005|antighost"
        "gs_mild|apps/cusfm_3dgut|50000|0.00012|400000|true|true|0.01|0.1|0.01|antighost"
        "gs_moderate|apps/cusfm_3dgut|50000|0.0001|350000|true|true|0.01|0.1|0.01|antighost"
        "gs_strong|apps/cusfm_3dgut|50000|0.0001|300000|true|true|0.01|0.1|0.01|antighost"
        "gs_aggressive|apps/cusfm_3dgut|50000|0.00008|150000|true|true|0.015|0.2|0.02|antighost"
        # GROUP B: Optimizer (2)
        "mcmc_adam|apps/cusfm_3dgut_mcmc|50000|0.0001|350000|true|true|0.01|0.1|0.01|optimizer"
        "gs_adam|apps/cusfm_3dgut|50000|0.0001|350000|true|true|0.01|0.1|0.01|optimizer"
        # GROUP C: Progressive Training (4)
        "mcmc_sh1|apps/cusfm_3dgut_mcmc|50000|0.0001|350000|true|true|0.01|0.1|0.01|progressive"
        "mcmc_sh2|apps/cusfm_3dgut_mcmc|50000|0.0001|350000|true|true|0.01|0.1|0.01|progressive"
        "mcmc_sh_fast|apps/cusfm_3dgut_mcmc|50000|0.0001|350000|true|true|0.01|0.1|0.01|progressive"
        "mcmc_sh_slow|apps/cusfm_3dgut_mcmc|50000|0.0001|350000|true|true|0.01|0.1|0.01|progressive"
        # GROUP D: Training Duration (4)
        "mcmc_iter_short|apps/cusfm_3dgut_mcmc|25000|0.0001|350000|true|true|0.01|0.1|0.01|duration"
        "mcmc_iter_long|apps/cusfm_3dgut_mcmc|75000|0.0001|350000|true|true|0.01|0.1|0.01|duration"
        "gs_iter_short|apps/cusfm_3dgut|25000|0.0001|350000|true|true|0.01|0.1|0.01|duration"
        "gs_iter_long|apps/cusfm_3dgut|75000|0.0001|350000|true|true|0.01|0.1|0.01|duration"
        # GROUP E: MCMC Strategy Tuning (3)
        "mcmc_noise_low|apps/cusfm_3dgut_mcmc|50000|0.0001|350000|true|true|0.01|0.1|0.01|mcmc_tune"
        "mcmc_noise_high|apps/cusfm_3dgut_mcmc|50000|0.0001|350000|true|true|0.01|0.1|0.01|mcmc_tune"
        "mcmc_add_late|apps/cusfm_3dgut_mcmc|50000|0.0001|350000|true|true|0.01|0.1|0.01|mcmc_tune"
        # GROUP F: GS Densification Tuning (3)
        "gs_densify_tight|apps/cusfm_3dgut|50000|0.0001|350000|true|true|0.01|0.1|0.01|gs_tune"
        "gs_densify_loose|apps/cusfm_3dgut|50000|0.0001|350000|true|true|0.01|0.1|0.01|gs_tune"
        "gs_densify_long|apps/cusfm_3dgut|50000|0.0001|350000|true|true|0.01|0.1|0.01|gs_tune"
        # GROUP G: Loss Balance (3)
        "mcmc_ssim_heavy|apps/cusfm_3dgut_mcmc|50000|0.0001|350000|true|true|0.01|0.1|0.01|loss"
        "mcmc_l1_heavy|apps/cusfm_3dgut_mcmc|50000|0.0001|350000|true|true|0.01|0.1|0.01|loss"
        "gs_ssim_heavy|apps/cusfm_3dgut|50000|0.0001|350000|true|true|0.01|0.1|0.01|loss"
        # GROUP H: Sorted Rendering (2)
        "mcmc_sorted|apps/cusfm_3dgut_mcmc|50000|0.0001|350000|true|true|0.01|0.1|0.01|render"
        "gs_sorted|apps/cusfm_3dgut|50000|0.0001|350000|true|true|0.01|0.1|0.01|render"
        # GROUP I: Combined Best Practice (2)
        "mcmc_best|apps/cusfm_3dgut_mcmc|75000|0.0001|350000|true|true|0.01|0.1|0.01|best"
        "gs_best|apps/cusfm_3dgut|75000|0.0001|350000|true|true|0.01|0.1|0.01|best"
    )

    # --- Extra args ---
    EXP_EXTRA[mcmc_adam]="optimizer.type=adam"
    EXP_EXTRA[gs_adam]="optimizer.type=adam"
    EXP_EXTRA[mcmc_sh1]="model.progressive_training.max_n_features=1"
    EXP_EXTRA[mcmc_sh2]="model.progressive_training.max_n_features=2"
    EXP_EXTRA[mcmc_sh_fast]="model.progressive_training.increase_frequency=150"
    EXP_EXTRA[mcmc_sh_slow]="model.progressive_training.increase_frequency=600"
    EXP_EXTRA[mcmc_iter_short]="scheduler.positions.max_steps=25000"
    EXP_EXTRA[mcmc_iter_long]="scheduler.positions.max_steps=75000"
    EXP_EXTRA[gs_iter_short]="scheduler.positions.max_steps=25000"
    EXP_EXTRA[gs_iter_long]="scheduler.positions.max_steps=75000"
    EXP_EXTRA[mcmc_noise_low]="strategy.perturb.noise_lr=30000"
    EXP_EXTRA[mcmc_noise_high]="strategy.perturb.noise_lr=300000"
    EXP_EXTRA[mcmc_add_late]="strategy.relocate.end_iteration=45000 strategy.add.end_iteration=45000"
    EXP_EXTRA[gs_densify_tight]="strategy.densify.clone_grad_threshold=0.0004 strategy.densify.split_grad_threshold=0.0004"
    EXP_EXTRA[gs_densify_loose]="strategy.densify.clone_grad_threshold=0.0001 strategy.densify.split_grad_threshold=0.0001"
    EXP_EXTRA[gs_densify_long]="strategy.densify.end_iteration=40000 strategy.prune.end_iteration=40000"
    EXP_EXTRA[mcmc_ssim_heavy]="loss.lambda_l1=0.6 loss.lambda_ssim=0.4"
    EXP_EXTRA[mcmc_l1_heavy]="loss.lambda_l1=0.95 loss.lambda_ssim=0.05"
    EXP_EXTRA[gs_ssim_heavy]="loss.lambda_l1=0.6 loss.lambda_ssim=0.4"
    EXP_EXTRA[mcmc_sorted]="render.splat.k_buffer_size=32"
    EXP_EXTRA[gs_sorted]="render.splat.k_buffer_size=32"
    EXP_EXTRA[mcmc_best]="scheduler.positions.max_steps=75000 render.splat.k_buffer_size=32 loss.lambda_l1=0.6 loss.lambda_ssim=0.4 model.progressive_training.increase_frequency=150"
    EXP_EXTRA[gs_best]="scheduler.positions.max_steps=75000 render.splat.k_buffer_size=32 loss.lambda_l1=0.6 loss.lambda_ssim=0.4 strategy.densify.clone_grad_threshold=0.0001 strategy.densify.split_grad_threshold=0.0001"
}




# --- 실험 제출 ---
LAST_JOBID=""
submit_one() {
    local exp="$1"; local tgt="${2:-}"
    IFS='|' read -r NM CFG NI PLR MG US UO OT LS LO TG <<< "$exp"
    local ts=$(date +%Y%m%d_%H%M%S)
    local plrf; plrf=$(awk "BEGIN {printf \"%.2e\", $PLR/100}")
    [ "$US" = "false" ] && LS="0.0"; [ "$UO" = "false" ] && LO="0.0"
    local extra="${EXP_EXTRA[$NM]:-}"

    local o=(--job-name="r2-${NM}" --gres=gpu:1 --output="${SCRIPT_DIR}/logs/r2_${NM}.%j.out" --error="${SCRIPT_DIR}/logs/r2_${NM}.%j.err")
    o+=(--exclude="$EXCLUDE_NODES")
    local ev="ALL,COLMAP_PATH=${COLMAP_PATH},MESH_PATH=${MESH_PATH}"
    ev+=",OUTPUT_DIR=${OUTPUT_BASE}/${NM}_${ts},EXPERIMENT_NAME=${NM}_${ts}"
    ev+=",N_ITERATIONS=${NI},CONFIG_NAME=${CFG},TEST_SPLIT_INTERVAL=${TEST_SPLIT_INTERVAL}"
    ev+=",POSITION_LR=${PLR},POSITION_LR_FINAL=${plrf},MAX_GAUSSIANS=${MG}"
    ev+=",USE_SCALE_LOSS=${US},LAMBDA_SCALE=${LS}"
    ev+=",USE_OPACITY_LOSS=${UO},LAMBDA_OPACITY=${LO},OPACITY_THRESHOLD=${OT}"
    [ -n "$extra" ] && ev+=",EXTRA_ARGS=${extra}"
    o+=(--export="$ev")

    if [ "${DRY_RUN:-false}" = "true" ]; then
        LAST_JOBID="DRY-${NM}"
        echo -e "  ${YLW}[DRY-RUN]${NC} sbatch ${o[*]} $SBATCH_SCRIPT"
        [ -n "$extra" ] && echo -e "    ${CYN}  EXTRA: ${extra}${NC}"
        return 0; fi
    local r; r=$(sbatch "${o[@]}" "$SBATCH_SCRIPT" 2>&1) || true
    local jobid; jobid=$(echo "$r" | grep -oE '[0-9]+' || echo "?")
    LAST_JOBID="$jobid"
    if echo "$r" | grep -q "Submitted"; then
        log_ok "${NM} → ${tgt:-auto} (Job ${jobid})"
        echo "    ${CFG} | ${NI}iter | PosLR:${PLR} | MaxG:${MG}"
        [ -n "$extra" ] && echo "    EXTRA: ${extra}" || true
    else log_err "${NM} 제출 실패: $r"; fi
}

# --- 메인 ---
main() {
    local dry=false; local only=(); local groups=()
    while [[ $# -gt 0 ]]; do case "$1" in
        --dry-run) dry=true; shift;;
        --only) shift; while [[ $# -gt 0 ]] && [[ "$1" != --* ]]; do only+=("$1"); shift; done;;
        --group) shift; while [[ $# -gt 0 ]] && [[ "$1" != --* ]]; do groups+=("$1"); shift; done;;
        -h|--help) echo "사용법: bash submit_experiments_round2.sh [옵션]"
            echo "  --dry-run            제출 없이 미리보기"
            echo "  --only NAME ...      특정 실험명만 제출"
            echo "  --group TAG ...      특정 그룹만 제출"
            echo "  그룹: antighost optimizer progressive duration mcmc_tune gs_tune loss render best"
            exit 0;;
        *) log_err "알 수 없는 옵션: $1"; exit 1;; esac; done
    export DRY_RUN="$dry"

    log_title "========== 3DGRUT Experiment Launcher — round2 (mesh.ply init) =========="
    echo "시각: $(date)"
    echo "데이터: $COLMAP_PATH (54장, 5662 sparse pts)"
    echo "출력: $OUTPUT_BASE"
    echo -e "${YLW}★ MESH 초기화: ${MESH_PATH}${NC}"
    echo -e "${YLW}★ 54장 스케일링: base=50k iter, short=25k, long=75k, MaxG 150k~500k${NC}"
    echo -e "${YLW}★ test_split_interval=${TEST_SPLIT_INTERVAL} → test ~9장, train ~45장${NC}"
    echo -e "${YLW}★ 노드 고정: ${ALLOWED_NODES} (3노드 × 8GPU = 최대 24 동시 실행)${NC}"
    check_node_resources; define_experiments

    local filtered=()
    for e in "${EXPERIMENTS[@]}"; do
        local nm="${e%%|*}"; local tg="${e##*|}"
        if [ ${#only[@]} -gt 0 ]; then
            local ok=false; for x in "${only[@]}"; do [ "$x" = "$nm" ] && ok=true; done
            [ "$ok" = "false" ] && continue; fi
        if [ ${#groups[@]} -gt 0 ]; then
            local ok=false; for g in "${groups[@]}"; do [ "$g" = "$tg" ] && ok=true; done
            [ "$ok" = "false" ] && continue; fi
        filtered+=("$e"); done

    [ ${#filtered[@]} -eq 0 ] && { log_err "제출할 실험 없음"; exit 1; }
    mkdir -p "$SCRIPT_DIR/logs"

    log_title "========== 제출 예정 (${#filtered[@]}개) =========="
    printf "${BLD}%-3s %-20s %-10s %-7s %-10s %-8s %-7s %-7s %-10s${NC}\n" \
        "#" "실험명" "그룹" "Iters" "PosLR" "MaxG" "λ_s" "λ_o" "노드"
    echo "────────────────────────────────────────────────────────────────────────────────────────"
    local subs=(); local prev_tag=""
    for i in "${!filtered[@]}"; do
        IFS='|' read -r NM CFG NI PLR MG _ _ OT LS LO TG <<< "${filtered[$i]}"
        local tn=""
        if [ "$TG" != "$prev_tag" ]; then
            [ -n "$prev_tag" ] && echo ""
            prev_tag="$TG"
        fi
        printf "%-3s %-20s %-10s %-7s %-10s %-8s %-7s %-7s %-10s\n" \
            "$((i+1))" "$NM" "$TG" "$NI" "$PLR" "$MG" "$LS" "$LO" "${tn:-auto}"
        local extra="${EXP_EXTRA[$NM]:-}"
        [ -n "$extra" ] && printf "    ${CYN}↳ %s${NC}\n" "$extra"
        subs+=("${filtered[$i]}|${tn}"); done
    echo ""

    if [ "$dry" = "false" ]; then
        echo -ne "${BLD}${#filtered[@]}개 실험 제출? (3노드 × 8GPU = 최대 24 동시) [y/N] ${NC}"; read -r ans
        [[ ! "$ans" =~ ^[yY]$ ]] && { log_warn "취소됨."; exit 0; }; fi

    log_title "========== 제출 중 (의존성 없음 — SLURM GPU 기반 스케줄링) =========="
    echo -e "${YLW}→ 24 GPU 가용: 최대 24개 즉시 실행, 나머지 대기${NC}\n"
    local cnt=0

    for s in "${subs[@]}"; do
        submit_one "${s%|*}" "${s##*|}" || true
        cnt=$((cnt+1))
        [ "$dry" = "false" ] && sleep 0.5
    done
    echo ""; log_title "========== 완료 =========="
    log_ok "총 ${cnt}개 실험 제출 (round2 — mesh.ply 초기화)"
    echo -e "  ${BLD}노드: ${ALLOWED_NODES} (3노드 × 8GPU = 24 GPU)${NC}"
    echo "  squeue -u $USER                    # 상태 확인"
    echo "  watch -n 10 squeue -u $USER        # 실시간 모니터링"
    echo "  tail -f $SCRIPT_DIR/logs/r2_*.out  # 로그 확인"
}
main "$@"