import base64
from io import BytesIO

from PIL import Image


def image_to_b64(image: Image) -> str:
    buffered = BytesIO()
    # convert RGBA to RGB
    if image.mode == "RGBA":
        image = image.convert("RGB")
    image.save(buffered, format="JPEG")
    return base64.b64encode(buffered.getvalue()).decode("utf-8")
