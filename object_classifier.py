#!/usr/bin/env python3
"""Explicit, single-part image classification for the Realtime assistant.

AprilTags own identity and geometry. This module never scans automatically,
creates scene objects, or changes coordinates and dimensions.
"""

from __future__ import annotations

import base64
import json
import os
import urllib.request
from typing import Any, Dict

import cv2
import numpy as np


def _response_text(payload: Dict[str, Any]) -> str:
    if payload.get("output_text"):
        return str(payload["output_text"])
    for item in payload.get("output") or []:
        for content in item.get("content") or []:
            if content.get("text"):
                return str(content["text"])
    raise RuntimeError("The classification response contained no text.")


def classify_visible_part(jpeg: bytes, part: Dict[str, Any], model: str = "gpt-5.5") -> Dict[str, Any]:
    api_key = os.environ.get("OPENAI_API_KEY")
    if not api_key:
        raise RuntimeError("OPENAI_API_KEY is not set.")
    image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError("Could not decode the camera frame.")
    box = part.get("bboxPx") or {}
    x, y = int(box.get("x") or 0), int(box.get("y") or 0)
    width, height = int(box.get("width") or 0), int(box.get("height") or 0)
    if width > 0 and height > 0:
        padding = max(width, height) * 2
        x0, y0 = max(0, int(x - padding)), max(0, int(y - padding))
        x1, y1 = min(image.shape[1], int(x + width + padding)), min(image.shape[0], int(y + height + padding))
        image = image[y0:y1, x0:x1]
    ok, encoded = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), 88])
    if not ok:
        raise RuntimeError("Could not encode the selected part image.")
    schema = {
        "type": "object", "additionalProperties": False,
        "properties": {
            "label": {"type": "string"},
            "shape": {"type": "string", "enum": ["box", "cylinder", "sphere", "rectangle", "circle", "unknown"]},
            "confidence": {"type": "number"},
            "description": {"type": "string"},
        },
        "required": ["label", "shape", "confidence", "description"],
    }
    prompt = (
        "Identify only the physical object carrying the selected AprilTag. Return a concise useful name and basic shape. "
        "Do not estimate position, orientation, size, or dimensions. Do not identify the robot, table, bins, or reference tags. "
        f"Current operator name: {part.get('label') or 'unnamed'}; configured shape: {part.get('type') or 'unknown'}."
    )
    body = {
        "model": model,
        "input": [{"role": "user", "content": [
            {"type": "input_text", "text": prompt},
            {"type": "input_image", "image_url": "data:image/jpeg;base64," + base64.b64encode(bytes(encoded)).decode("ascii"), "detail": "high"},
        ]}],
        "text": {"format": {"type": "json_schema", "name": "tagged_part_identity", "strict": True, "schema": schema}},
    }
    request = urllib.request.Request(
        "https://api.openai.com/v1/responses", data=json.dumps(body).encode("utf-8"), method="POST",
        headers={"Content-Type": "application/json", "Authorization": f"Bearer {api_key}"},
    )
    with urllib.request.urlopen(request, timeout=30) as response:
        parsed = json.loads(_response_text(json.loads(response.read().decode("utf-8"))))
    return {
        "ok": True, "partId": part.get("id"), "suggestion": {
            "label": str(parsed.get("label") or part.get("label") or "Tagged Part"),
            "shape": str(parsed.get("shape") or "unknown"),
            "confidence": max(0.0, min(1.0, float(parsed.get("confidence") or 0.0))),
            "description": str(parsed.get("description") or ""),
        },
    }
