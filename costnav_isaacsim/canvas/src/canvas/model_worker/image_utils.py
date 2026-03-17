import base64
import time
from contextlib import contextmanager
from io import BytesIO

from PIL import Image


def b64_to_image(b64: str) -> Image:
    return Image.open(BytesIO(base64.b64decode(b64)))


@contextmanager
def timed(name: str):
    start = time.time()
    yield
    print(f"{name} took {(time.time() - start) * 1000:.2f}ms")
