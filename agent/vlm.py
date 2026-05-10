"""Bedrock client for Claude vision + agent reasoning.

Uses the Bedrock Converse API, which supports both image inputs and tool use
in a single call shape — convenient for our agent loop.

Auth: set AWS_BEARER_TOKEN_BEDROCK in .env (the new Bedrock API key form).
boto3 picks it up automatically.
"""

from __future__ import annotations

import base64
import os
from functools import lru_cache
from typing import Optional

import boto3


@lru_cache(maxsize=1)
def make_client():
    """Cached singleton — boto3 clients hold a urllib3 connection pool, so
    rebuilding one per call throws away keep-alive TLS sessions to Bedrock."""
    region = os.environ.get("AWS_REGION", "us-east-1")
    return boto3.client("bedrock-runtime", region_name=region)


def _model_id() -> str:
    return os.environ.get(
        "BEDROCK_MODEL_ID", "us.anthropic.claude-opus-4-7-20251101-v1:0"
    )


def vlm_describe(
    client,
    jpeg_b64: str,
    prompt: str,
    model: Optional[str] = None,
    max_tokens: int = 512,
) -> str:
    """Single-shot VLM call: prompt + image → text."""
    image_bytes = base64.b64decode(jpeg_b64)
    resp = client.converse(
        modelId=model or _model_id(),
        messages=[
            {
                "role": "user",
                "content": [
                    {"text": prompt},
                    {"image": {"format": "jpeg", "source": {"bytes": image_bytes}}},
                ],
            }
        ],
        inferenceConfig={"maxTokens": max_tokens},
    )
    parts = resp["output"]["message"]["content"]
    return "".join(p.get("text", "") for p in parts)
