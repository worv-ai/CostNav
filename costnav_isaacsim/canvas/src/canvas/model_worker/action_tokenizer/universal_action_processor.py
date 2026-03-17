"""
Adapted from the FAST project`s action-tokenizer implementation:
https://huggingface.co/physical-intelligence/fast/blob/main/processing_action_tokenizer.py
"""

import logging
from pathlib import Path
from typing import ClassVar

import numpy as np
from scipy.fft import dct, idct
from tokenizers import ByteLevelBPETokenizer
from tokenizers.trainers import BpeTrainer
from transformers import PreTrainedTokenizerFast
from transformers.processing_utils import ProcessorMixin


# ---------------- Normalization helpers ---------------- #
def _compute_quantile_stats(actions: list[np.ndarray], q_low: float = 0.01, q_high: float = 99.99):
    """Return per-dimension (q_low, q_high) as float arrays."""
    cat = np.concatenate([a.reshape(-1, a.shape[-1]) for a in actions], axis=0)
    return (np.percentile(cat, q_low, axis=0), np.percentile(cat, q_high, axis=0))


def _normalize(arr: np.ndarray, q_low: np.ndarray, q_high: np.ndarray):
    """Map values so that q_low→-1 and q_high→+1 (clip outside)."""
    num = arr - q_low
    den = np.where(q_high - q_low == 0, 1e-8, q_high - q_low)
    scaled = -1 + 2.0 * (num / den)
    return np.clip(scaled, -1.0, 1.0)


def _denormalize(norm: np.ndarray, q_low: np.ndarray, q_high: np.ndarray):
    """Inverse of _normalize."""
    den = np.where(q_high - q_low == 0, 1e-8, q_high - q_low)
    return q_low + (norm + 1.0) * 0.5 * den


class UniversalActionProcessor(ProcessorMixin):
    attributes: ClassVar[list[str]] = ["bpe_tokenizer"]
    bpe_tokenizer_class: str = "AutoTokenizer"

    def __init__(
        self,
        bpe_tokenizer: PreTrainedTokenizerFast,
        scale: float = 10,
        vocab_size: int = 1024,
        min_token: int = 0,
        action_dim: int | None = None,
        time_horizon: int | None = None,
        norm_stats: dict[str, np.ndarray] | None = None,
    ):
        self.scale = scale
        self.vocab_size = vocab_size
        self.min_token = min_token

        # Normalization stats: dict with keys "q01" and "q99"
        self.norm_stats = norm_stats or {"q01": None, "q99": None}

        # Action horizon and dimension needed during decoding. These can be specified
        # in three ways (in order of priority):
        # 1. passed in as kwargs to decode()
        # 2. in the constructor
        # 3. cached from the last time decode() was called
        self.time_horizon = time_horizon
        self.action_dim = action_dim
        self.called_time_horizon = time_horizon
        self.called_action_dim = action_dim

        super().__init__(bpe_tokenizer)

    def __call__(self, action_chunk: np.array) -> np.array:
        assert action_chunk.ndim <= 3, "Only 3 dimensions supported: [batch, timesteps, action_dim]"
        if action_chunk.ndim == 2:
            action_chunk = action_chunk[None, ...]

        # Cache the time horizon and action dimension for decoding
        self.called_time_horizon = action_chunk.shape[-2]
        self.called_action_dim = action_chunk.shape[-1]

        # --- Quantile normalization to [-1, 1] --------------------------- #
        assert self.norm_stats["q01"] is not None and self.norm_stats["q99"] is not None, (
            "norm_stats not set; call fit() first."
        )
        action_chunk = _normalize(action_chunk, self.norm_stats["q01"], self.norm_stats["q99"])

        dct_coeff = dct(action_chunk, axis=1, norm="ortho")
        dct_coeff = np.around(dct_coeff * self.scale)
        tokens = []
        for elem in dct_coeff:
            token_str = "".join(map(chr, np.maximum(elem.flatten() - self.min_token, 0).astype(int)))
            tokens.append(self.bpe_tokenizer(token_str)["input_ids"])
        return tokens

    def decode(
        self,
        tokens: list[list[int]],
        *,
        time_horizon: int | None = None,
        action_dim: int | None = None,
    ) -> np.array:
        self.time_horizon = time_horizon or self.time_horizon or self.called_time_horizon
        self.action_dim = action_dim or self.action_dim or self.called_action_dim

        # Cache the time horizon and action dimension for the next call
        self.called_time_horizon = self.time_horizon
        self.called_action_dim = self.action_dim

        assert self.time_horizon is not None and self.action_dim is not None, (
            "Tokenizer not initialized, call encode() once or pass in time_horizon and action_dim."
        )

        decoded_actions = []
        for token in tokens:
            try:
                decoded_tokens = self.bpe_tokenizer.decode(token)
                decoded_dct_coeff = np.array(list(map(ord, decoded_tokens))) + self.min_token
                decoded_dct_coeff = decoded_dct_coeff.reshape(-1, self.action_dim)
                assert decoded_dct_coeff.shape == (
                    self.time_horizon,
                    self.action_dim,
                ), (
                    f"Decoded DCT coefficients have shape {decoded_dct_coeff.shape}, expected ({self.time_horizon}, {self.action_dim})"
                )
                norm_act = idct(decoded_dct_coeff / self.scale, axis=0, norm="ortho")
                action = _denormalize(norm_act, self.norm_stats["q01"], self.norm_stats["q99"])
            except Exception as e:
                print(f"Error decoding tokens: {e}")
                print(f"Tokens: {token}")
                action = np.full((self.time_horizon, self.action_dim), np.nan)
            decoded_actions.append(action)
        return np.stack(decoded_actions)

    @classmethod
    def fit(
        cls,
        action_data: list[np.array],
        scale: float = 10,
        vocab_size: int = 1024,
        *,
        time_horizon: int | None = None,
        action_dim: int | None = None,
    ) -> "UniversalActionProcessor":
        # Compute per-dimension 0.01st and 99.99th quantiles for normalization
        q01, q99 = _compute_quantile_stats(action_data, 0.01, 99.99)
        norm_action_data = _normalize(action_data, q01, q99)

        # --- Normalize each action array to [-1, 1] BEFORE DCT -------------
        dct_tokens = [dct(a, axis=0, norm="ortho").flatten() for a in norm_action_data]

        # Quantize and find min token
        max_token = int(np.around(np.concatenate(dct_tokens) * scale).max())
        min_token = int(np.around(np.concatenate(dct_tokens) * scale).min())
        min_vocab_size = max_token - min_token

        assert min_vocab_size <= vocab_size, (
            f"Vocab size {vocab_size} is too small for the range of tokens {min_vocab_size}"
        )
        if min_vocab_size + 100 > vocab_size:
            logging.warning(
                f"Initial alphabet size {min_vocab_size} is almost as large as the vocab"
                f"size {vocab_size}, consider increasing vocab size"
            )

        # Make token iterator for BPE training
        def _token_iter():
            for tokens in dct_tokens:
                rounded_tokens = np.around(tokens * scale) - min_token
                rounded_tokens = rounded_tokens.astype(int)
                string = "".join(map(chr, rounded_tokens))
                yield string

        # Train BPE tokenizer
        bpe = ByteLevelBPETokenizer()

        # Set up the entire range of possible tokens as the initial alphabet
        alphabet = [chr(i) for i in range(max_token - min_token + 1)]
        trainer = BpeTrainer(
            vocab_size=vocab_size,
            min_frequency=2,
            show_progress=True,
            special_tokens=[],
            initial_alphabet=alphabet,
            max_token_length=10000,
        )

        # Train the inner tokenizer (don't use ByteLevelBPETokenizer.train_from_iterator()
        # because it doesn't support custom alphabets)
        bpe._tokenizer.train_from_iterator(_token_iter(), trainer=trainer)

        return cls(
            PreTrainedTokenizerFast(tokenizer_object=bpe, clean_up_tokenization_spaces=False),
            scale=scale,
            vocab_size=vocab_size,
            min_token=min_token,
            time_horizon=time_horizon,
            action_dim=action_dim,
            norm_stats={"q01": q01, "q99": q99},
        )

    # ------------------------------------------------------------------ #
    # Serialization helpers                                              #
    # ------------------------------------------------------------------ #
    def to_dict(self):
        """
        Override ProcessorMixin.to_dict() to ensure norm_stats is JSON-serializable
        while preserving the base-class filtering (e.g. removing tokenizer).
        """
        data = super().to_dict()  # Let ProcessorMixin do its cleanup first
        if "norm_stats" in data and isinstance(data["norm_stats"], dict):
            data["norm_stats"] = {
                k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in data["norm_stats"].items()
            }
        return data

    @classmethod
    def from_pretrained(cls, pretrained_model_name_or_path, **kwargs):
        processor = super().from_pretrained(pretrained_model_name_or_path, **kwargs)
        # Ensure norm_stats values are numpy arrays
        if hasattr(processor, "norm_stats") and isinstance(processor.norm_stats, dict):
            for key in ("q01", "q99"):
                val = processor.norm_stats.get(key)
                if val is not None and not isinstance(val, np.ndarray):
                    processor.norm_stats[key] = np.array(val, dtype=np.float32)
        return processor

    def visualize_action_tokens(self, actions, output_path, visualize_num=20):
        import matplotlib.pyplot as plt
        import numpy as np

        rng = np.random.default_rng(42)
        idxs = rng.choice(len(actions), visualize_num, replace=False)
        all_original = []
        all_decoded = []

        for idx in idxs:
            original = actions[idx]
            encoded = self(original)
            decoded = self.decode(encoded)[0]
            all_original.append(original)
            all_decoded.append(decoded)

        tokens_list = [self(original) for original in all_original]

        time_steps = np.arange(all_original[0].shape[0])
        action_dim = all_original[0].shape[1]
        fig, axes = plt.subplots(visualize_num, 1, figsize=(10, 2.5 * visualize_num), sharex=True, sharey=True)

        if visualize_num == 1:
            axes = [axes]

        for ax, original, decoded, token in zip(axes, all_original, all_decoded, tokens_list):
            for d in range(action_dim):
                ax.plot(time_steps, original[:, d], label=f"original dim {d}", alpha=0.8)
                ax.plot(time_steps, decoded[:, d], linestyle="--", label=f"decoded dim {d}", alpha=0.8)
            ax.text(
                0.01,
                0.95,
                f"len(action) = {original.size}, len(token) = {sum(len(t) for t in token)}\ntokens: {token[0]}",
                transform=ax.transAxes,
                fontsize=10,
                verticalalignment="top",
                horizontalalignment="left",
                bbox=dict(facecolor="white", alpha=0.5, edgecolor="gray"),
            )
            ax.grid(True)

        axes[0].legend(loc="upper right")
        fig.suptitle("Original vs Decoded Actions", fontsize=14)
        fig.supxlabel("Time Step")
        fig.supylabel("Action Value")
        plt.tight_layout(rect=[0, 0, 1, 0.96])
        # save the figure
        fig.savefig(output_path, bbox_inches="tight", dpi=300)
        plt.close(fig)


def pad_and_save(seq_list, path: Path, pad_val=-1):
    if not isinstance(path, Path):
        path = Path(path)
    assert path.suffix == ".npz", "Path must end with .npz"
    max_len = max(len(s) for s in seq_list)
    data = np.full((len(seq_list), max_len), pad_val, dtype=np.int32)
    mask = np.zeros_like(data, dtype=bool)
    for i, s in enumerate(seq_list):
        data[i, : len(s)] = s
        mask[i, : len(s)] = True
    np.savez(path, data=data, mask=mask)


def load_and_unpad(path: Path):
    if not isinstance(path, Path):
        path = Path(path)
    assert path.suffix == ".npz", "Path must end with .npz"
    npz = np.load(path)
    data, mask = npz["data"], npz["mask"]
    return [row[m].tolist() for row, m in zip(data, mask)]


def calculate_compression_ratio(actions: np.ndarray, tokenized_actions: list[list[int]] | None = None):
    """
    Calculate the compression ratio of the BPE tokenizer on the given actions.
    Compression ratio is defined as the ratio of the number of tokens to the number of original action values.
    """
    total_tokens = 0
    total_values = sum(a.size for a in actions)
    token_lengths = [len(tokens) for tokens in tokenized_actions]
    total_tokens = sum(token_lengths)
    mean_token_length = np.mean(token_lengths).item()
    compression_rate = total_values / total_tokens if total_tokens > 0 else float("inf")
    return total_tokens, mean_token_length, total_values, compression_rate
