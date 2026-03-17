import re
from pathlib import Path
from typing import Optional

import numpy as np
import torch
from PIL import Image
from transformers import (
    BitsAndBytesConfig,
    InternVLForConditionalGeneration,
    InternVLProcessor,
    PreTrainedModel,
    ProcessorMixin,
)

from canvas.model_worker.action_tokenizer import UniversalActionProcessor
from canvas.model_worker.base_model import BaseModelWithYamlJsonFromTo


class InternVL3PipelineConfig(BaseModelWithYamlJsonFromTo):
    pipeline_class: str = "InternVL3Pipeline"


class InternVL3Pipeline:
    def __init__(
        self,
        model: PreTrainedModel,
        processor: ProcessorMixin,
        action_tokenizer: UniversalActionProcessor,
        config: InternVL3PipelineConfig,
        action_history_tokenizer: UniversalActionProcessor,
    ):
        self.model = model
        self.processor = processor
        self.action_tokenizer = action_tokenizer
        self.action_history_tokenizer = action_history_tokenizer
        self.config = config

    def save_pretrained(
        self,
        save_directory: str,
    ):
        if not isinstance(save_directory, Path):
            save_directory = Path(save_directory)
        self.model.save_pretrained(save_directory)
        self.processor.save_pretrained(save_directory / "processor")
        self.action_tokenizer.save_pretrained(save_directory / "action_tokenizer")
        self.action_history_tokenizer.save_pretrained(save_directory / "action_history_tokenizer")
        self.config.to_json(f"{save_directory}/pipeline_config.json")

    @classmethod
    def from_pretrained(cls, pretrained_model_name_or_path: str, device: Optional[str] = None, **kwargs):
        if not isinstance(pretrained_model_name_or_path, Path):
            pretrained_model_name_or_path = Path(pretrained_model_name_or_path)

        config = InternVL3PipelineConfig.model_validate_json(
            (pretrained_model_name_or_path / "pipeline_config.json").read_text()
        )
        model = cls.load_model(pretrained_model_name_or_path, device=device, **kwargs)
        processor = InternVLProcessor.from_pretrained(pretrained_model_name_or_path / "processor")
        model.eval()
        action_tokenizer = UniversalActionProcessor.from_pretrained(pretrained_model_name_or_path / "action_tokenizer")

        # Load action history tokenizer
        action_history_tokenizer_path = pretrained_model_name_or_path / "action_history_tokenizer"
        action_history_tokenizer = UniversalActionProcessor.from_pretrained(action_history_tokenizer_path)

        return cls(model, processor, action_tokenizer, config, action_history_tokenizer)

    @classmethod
    def load_model(
        cls,
        pretrained_model_name_or_path: str,
        device: Optional[str] = None,
        load_8bit=False,
        load_4bit=False,
        use_flash_attn=False,
    ):
        kwargs = {}
        if load_8bit:
            kwargs["load_in_8bit"] = True
        elif load_4bit:
            kwargs["quantization_config"] = BitsAndBytesConfig(
                load_in_4bit=True,
                bnb_4bit_compute_dtype=torch.float16,
                bnb_4bit_use_double_quant=True,
                bnb_4bit_quant_type="nf4",
            )
        else:
            kwargs["torch_dtype"] = torch.bfloat16

        if use_flash_attn:
            kwargs["attn_implementation"] = "flash_attention_2"

        model = InternVLForConditionalGeneration.from_pretrained(
            pretrained_model_name_or_path, device_map=device, **kwargs
        )
        return model

    def to(self, device):
        return self.model.to(device)

    @torch.no_grad()
    def __call__(
        self,
        message_list: list[list[dict]],
        images_list: list[list[Image.Image]],
        action_history: list[list[list[float]]],
        return_traj: Optional[bool] = False,
    ):
        """
        call model with message and images

        Args:
            message_list: list of messages, [B, messages]
            images_list: list of images, [B, images]
            action_history: list of action history, [B, history_length, 2]
            return_traj: return trajectory if True
        """

        # we don't use batch inference for run model worker
        if len(message_list) != 1:
            raise ValueError("No batch api call allowed for InternVL3Pipeline")

        message = message_list[0]
        images = images_list[0]

        # Convert action history to numpy array and tokenize
        action_history_array = np.array(action_history[0])  # Shape: (history_length, 2)
        # Tokenize the action history
        history_tokens = self.action_history_tokenizer(action_history_array[None, ...])[0]  # Add batch dimension
        # Convert tokens to text format for the model
        history_tokens_text = "".join([f"<ACTION_{token}>" for token in history_tokens])

        # Add history tokens to the message
        # message structure varies based on configured cameras (NavDatasetConfig.camera_keys):
        # N camera images + 1 map image, e.g. front-only → 2 images, front+left+right → 4 images
        #     {"role": "user", "content": [{"type": "image"}, ..., {"type": "text", "text": latency_text}]}

        # Prepend history tokens to the text content (always the last item in content list)
        message[1]["content"][-1]["text"] = history_tokens_text + message[1]["content"][-1]["text"]

        # Apply chat template and add action history if available
        prompt = self.processor.apply_chat_template(message, add_generation_prompt=True)

        inputs = self.processor(
            text=prompt,
            images=images,
            return_tensors="pt",
        ).to(self.model.device, dtype=self.model.dtype)

        # Generate response
        generate_ids = self.model.generate(**inputs)[:, inputs["input_ids"].shape[1] :]
        generated_texts = self.processor.batch_decode(generate_ids, skip_special_tokens=True)

        if return_traj:
            pred_action = re.findall(r"<ACTION_(\d+)>", generated_texts[0])
            pred_action = np.array(pred_action, dtype=np.int64)
            if len(pred_action.shape) == 1:
                pred_action = pred_action[None, :]
            decoded = self.action_tokenizer.decode(pred_action)
            return decoded.tolist(), pred_action.flatten().tolist()
        else:
            return generated_texts
