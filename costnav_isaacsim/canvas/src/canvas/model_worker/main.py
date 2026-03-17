import asyncio
import os
import time
from contextlib import asynccontextmanager

import numpy as np
from fastapi import FastAPI, Request

from canvas.model_worker.internvl3 import InternVL3Pipeline
from canvas.model_worker.image_utils import b64_to_image, timed

# Global model and semaphore for thread safety
model = None
model_semaphore = None


def sanitize_for_json(obj):
    """
    Recursively sanitize numpy arrays and lists to remove inf, -inf, and nan values.
    Replaces them with None or 0.0 for JSON serialization.
    """
    if isinstance(obj, np.ndarray):
        obj = obj.tolist()

    if isinstance(obj, list):
        return [sanitize_for_json(item) for item in obj]
    elif isinstance(obj, dict):
        return {k: sanitize_for_json(v) for k, v in obj.items()}
    elif isinstance(obj, float):
        if np.isnan(obj) or np.isinf(obj):
            return None
        return obj
    else:
        return obj


@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup
    global model, model_semaphore
    try:
        import flash_attn  # noqa: F401

        use_flash_attn = True
    except ImportError:
        print("flash_attn not installed; not using flash attention.")
        use_flash_attn = False
    MODEL_PATH = os.environ.get("MODEL_PATH", "/app/model")
    model = InternVL3Pipeline.from_pretrained(MODEL_PATH, device="cuda", use_flash_attn=use_flash_attn)
    model_semaphore = asyncio.Semaphore(1)  # Only allow 1 concurrent inference
    print("Model loaded!")
    yield
    # Shutdown
    print("Shutting down...")


app = FastAPI(lifespan=lifespan)


@app.get("/")
async def root():
    return {"message": "Hello World"}


@app.post("/inference")
async def inference(request: Request):
    payload = await request.json()
    messages = payload["messages"]
    assert isinstance(payload["images"][0], list), (
        "The images should be a list of lists. Aware that image should be nested-nested-list!"
    )
    images = [[*map(b64_to_image, sub_images)] for sub_images in payload["images"]]

    # Assert that action history is provided
    assert "actions_history" in payload, "actions_history must be provided in the payload"
    action_history = payload["actions_history"]
    assert isinstance(action_history, list), "actions_history must be a list"
    # Track waiting time for semaphore acquisition
    wait_start = time.time()
    async with model_semaphore:
        wait_time = time.time() - wait_start
        print(f"Wait time: {wait_time:.4f}s")

        # Run the blocking model inference in a thread pool to avoid blocking the event loop
        loop = asyncio.get_event_loop()
        with timed("model"):
            try:
                action, tokens = await loop.run_in_executor(
                    None, lambda: model(messages, images, action_history, return_traj=True)
                )
            except Exception as e:
                print(f"Model inference failed: {e}")
                action = np.zeros((1, model.action_tokenizer.time_horizon, model.action_tokenizer.action_dim)).tolist()
                tokens = model.action_tokenizer.tokenize(np.array(action)).tolist()[0]

    print(f"{action=}, {tokens=}")

    # Sanitize action and tokens to remove inf, -inf, and nan values for JSON serialization
    action = sanitize_for_json(action)
    tokens = sanitize_for_json(tokens)

    return {"action": action, "tokens": tokens}
