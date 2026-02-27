#!/usr/bin/env python3
"""
Step 3: Depth Estimation for cusfm Keyframes using FoundationStereo (PyTorch)

Usage:
    python scripts/run-step3-depth-v2.py \
        --image_dir /workspace/data/PhysicalAI-Robotics-NuRec/nova_carter-galileo/rosbag_mapping_data \
        --output_dir /workspace/output_nurec/nova_carter_galileo/depth \
        --frames_meta_file /workspace/output_nurec/nova_carter_galileo/cusfm_fixed/kpmap/keyframes/frames_meta.json
"""

import os
import sys
import json
import argparse
from pathlib import Path
from collections import defaultdict

FOUNDATION_STEREO_DIR = "/workspace/models/FoundationStereo"
sys.path.insert(0, FOUNDATION_STEREO_DIR)

import torch
import numpy as np
import cv2
import imageio
from omegaconf import OmegaConf

from core.utils.utils import InputPadder
from Utils import set_logging_format, set_seed
from core.foundation_stereo import FoundationStereo

CKPT_PATH = f"{FOUNDATION_STEREO_DIR}/pretrained_models/23-51-11/model_best_bp2-001.pth"


class CameraMetadata:
    """frames_meta.json에서 카메라 파라미터를 읽어오는 클래스"""
    
    def __init__(self, meta_path):
        with open(meta_path, 'r') as f:
            self.data = json.load(f)
        
        self.camera_params = self.data.get('camera_params_id_to_camera_params', {})
        self.stereo_pairs = self.data.get('stereo_pair', [])
        self.keyframes = self.data.get('keyframes_metadata', [])
        
        self.stereo_pair_map = {}
        for pair in self.stereo_pairs:
            # Support both Nova Carter format (left_camera_param_id) and ZED format (no left_camera_param_id)
            left_id = pair.get('left_camera_param_id', '0')  # Default to '0' for ZED
            right_id = pair['right_camera_param_id']
            baseline = pair['baseline_meters']
            self.stereo_pair_map[left_id] = {
                'right_camera_id': right_id,
                'baseline': baseline
            }
    
    def get_focal_length(self, camera_params_id: str) -> float:
        params = self.camera_params.get(camera_params_id)
        if not params:
            raise ValueError(f"Camera params not found for id: {camera_params_id}")
        projection = params['calibration_parameters']['projection_matrix']['data']
        return float(projection[0])
    
    def get_camera_name(self, camera_params_id: str) -> str:
        params = self.camera_params.get(camera_params_id)
        if not params:
            raise ValueError(f"Camera params not found for id: {camera_params_id}")
        return params['sensor_meta_data']['sensor_name']
    
    def get_synced_keyframes(self):
        synced_groups = defaultdict(dict)
        for kf in self.keyframes:
            sync_id = kf.get('synced_sample_id')
            # Default to '0' if camera_params_id is missing (ZED left camera case)
            cam_id = kf.get('camera_params_id', '0')
            if sync_id:
                synced_groups[sync_id][cam_id] = kf
        return synced_groups
    
    def print_camera_info(self):
        print("\nCamera Parameters:")
        print("-" * 60)
        for cam_id, params in sorted(self.camera_params.items(), key=lambda x: int(x[0])):
            name = params['sensor_meta_data']['sensor_name']
            fx = params['calibration_parameters']['projection_matrix']['data'][0]
            print(f"  ID {cam_id}: {name} (fx={fx:.3f})")
        
        print("\nStereo Pairs:")
        for pair in self.stereo_pairs:
            left_id = pair.get('left_camera_param_id', '0')  # Default to '0' for ZED
            baseline = pair['baseline_meters']
            left_name = self.get_camera_name(left_id)
            print(f"  {left_name}: baseline={baseline:.6f}m")


def load_model(ckpt_path):
    print("\nLoading FoundationStereo model...")
    cfg_path = os.path.join(os.path.dirname(ckpt_path), "cfg.yaml")
    cfg = OmegaConf.load(cfg_path)
    if 'vit_size' not in cfg:
        cfg['vit_size'] = 'vitl'
    cfg.ckpt_dir = ckpt_path
    
    model = FoundationStereo(cfg)
    ckpt = torch.load(ckpt_path, weights_only=False)
    model.load_state_dict(ckpt['model'])
    model.cuda()
    model.eval()
    print(f"Model loaded! (epoch: {ckpt['epoch']}, step: {ckpt['global_step']})")
    return model


def process_stereo_pair(model, left_path, right_path, output_path, focal_length, baseline, scale=1.0, verbose=False):
    """단일 스테레오 쌍 처리 - 16-bit PNG depth 생성 (mm 단위)"""
    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    img_left = imageio.v2.imread(left_path)
    img_right = imageio.v2.imread(right_path)
    H_orig, W_orig = img_left.shape[:2]

    if scale != 1.0:
        img_left = cv2.resize(img_left, None, fx=scale, fy=scale)
        img_right = cv2.resize(img_right, None, fx=scale, fy=scale)
    H, W = img_left.shape[:2]

    if verbose:
        print(f"  Image: {W_orig}x{H_orig}, processing: {W}x{H}")
        print(f"  Focal: {focal_length:.3f}, Baseline: {baseline:.6f}m")

    img0 = torch.from_numpy(img_left).permute(2, 0, 1).float()[None].cuda()
    img1 = torch.from_numpy(img_right).permute(2, 0, 1).float()[None].cuda()
    padder = InputPadder(img0.shape, divis_by=32)
    img0, img1 = padder.pad(img0, img1)

    with torch.amp.autocast('cuda'):
        disp = model.forward(img0, img1, iters=32, test_mode=True)

    disp = padder.unpad(disp.float())
    disp = disp.data.cpu().numpy().reshape(H, W)

    if scale != 1.0:
        disp = cv2.resize(disp, (W_orig, H_orig), interpolation=cv2.INTER_NEAREST)
        disp = disp / scale

    MIN_DISPARITY = 0.01
    MAX_DEPTH_MM = 65535.0
    
    depth_mm = np.zeros_like(disp, dtype=np.float32)
    valid_mask = disp > MIN_DISPARITY
    
    if np.any(valid_mask):
        depth_mm[valid_mask] = (1000.0 * focal_length * baseline) / disp[valid_mask]
        depth_mm[depth_mm > MAX_DEPTH_MM] = 0

    cv2.imwrite(output_path, depth_mm.astype(np.uint16))

    if verbose:
        valid_depth = depth_mm[depth_mm > 0]
        if len(valid_depth) > 0:
            print(f"  Depth range: {valid_depth.min():.1f} - {valid_depth.max():.1f} mm")

    del img0, img1, padder
    torch.cuda.empty_cache()
    return True


def main():
    parser = argparse.ArgumentParser(description="Step 3: Depth Estimation using FoundationStereo")
    parser.add_argument("--image_dir", required=True)
    parser.add_argument("--output_dir", required=True)
    parser.add_argument("--frames_meta_file", required=True)
    parser.add_argument("--scale", type=float, default=1.0)
    parser.add_argument("--skip_existing", action="store_true", default=True)
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    set_logging_format()
    set_seed(0)
    torch.autograd.set_grad_enabled(False)

    print("=" * 60)
    print("Step 3: Depth Estimation (PyTorch FoundationStereo)")
    print("=" * 60)

    meta = CameraMetadata(args.frames_meta_file)
    meta.print_camera_info()

    synced_groups = meta.get_synced_keyframes()
    print(f"\nTotal synced groups: {len(synced_groups)}")

    model = load_model(CKPT_PATH)

    processed = 0
    skipped = 0
    errors = 0

    for left_cam_id, pair_info in meta.stereo_pair_map.items():
        right_cam_id = pair_info['right_camera_id']
        baseline = pair_info['baseline']
        focal_length = meta.get_focal_length(left_cam_id)
        camera_name = meta.get_camera_name(left_cam_id)

        print(f"\n{'=' * 60}")
        print(f"Processing {camera_name}")
        print("=" * 60)

        pairs_for_camera = []
        for sync_id, cameras in synced_groups.items():
            if left_cam_id in cameras and right_cam_id in cameras:
                pairs_for_camera.append({
                    'sync_id': sync_id,
                    'left': cameras[left_cam_id],
                    'right': cameras[right_cam_id]
                })

        print(f"Found {len(pairs_for_camera)} stereo pairs")

        for i, pair in enumerate(pairs_for_camera):
            left_kf = pair['left']
            right_kf = pair['right']
            left_file = left_kf['image_name']
            
            output_path = os.path.join(args.output_dir, left_file.replace('.jpeg', '.png').replace('.jpg', '.png'))

            if args.skip_existing and os.path.exists(output_path):
                if args.verbose:
                    print(f"[{i+1}/{len(pairs_for_camera)}] Skipping: {left_file}")
                skipped += 1
                continue

            left_path = os.path.join(args.image_dir, left_file)
            right_path = os.path.join(args.image_dir, right_kf['image_name'])

            if not os.path.exists(left_path) or not os.path.exists(right_path):
                print(f"[{i+1}/{len(pairs_for_camera)}] ERROR: Image not found - {left_file}")
                errors += 1
                continue

            print(f"[{i+1}/{len(pairs_for_camera)}] Processing: {left_file}")

            try:
                process_stereo_pair(model, left_path, right_path, output_path,
                                    focal_length, baseline, args.scale, args.verbose)
                processed += 1
            except Exception as e:
                print(f"  ERROR: {e}")
                errors += 1
                torch.cuda.empty_cache()

    print("\n" + "=" * 60)
    print("Step 3 Complete!")
    print("=" * 60)
    print(f"Processed: {processed}")
    print(f"Skipped: {skipped}")
    print(f"Errors: {errors}")

    return 0 if errors == 0 else 1


if __name__ == "__main__":
    sys.exit(main())