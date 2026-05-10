"""VLM client abstraction for the new pipeline.

Uses the openai package pointed at any OpenAI-compatible endpoint
(AWS Bedrock compat, Qwen, etc.) via AGENT_BASE_URL / AGENT_API_KEY / AGENT_MODEL.

Falls back to legacy vlm.vlm_describe() if AGENT_BASE_URL is not set.
"""

from __future__ import annotations

import os
from typing import Protocol, runtime_checkable


@runtime_checkable
class VLMClientProtocol(Protocol):
    def describe(self, jpeg_b64: str, prompt: str, max_tokens: int = 512) -> str: ...


class OpenAICompatVLMClient:
    """VLM via any OpenAI-compatible endpoint (Bedrock compat or Qwen)."""

    def __init__(self, client, model: str) -> None:
        self._client = client
        self._model = model

    def describe(self, jpeg_b64: str, prompt: str, max_tokens: int = 512) -> str:
        data_uri = f"data:image/jpeg;base64,{jpeg_b64}"
        response = self._client.chat.completions.create(
            model=self._model,
            messages=[
                {
                    "role": "user",
                    "content": [
                        {"type": "image_url", "image_url": {"url": data_uri}},
                        {"type": "text", "text": prompt},
                    ],
                }
            ],
            max_tokens=max_tokens,
        )
        return response.choices[0].message.content or ""


class LegacyBedrockVLMClient:
    """Thin wrapper around legacy vlm.vlm_describe() for backward compat."""

    def __init__(self, bedrock_client, model_id: str | None = None) -> None:
        self._bedrock_client = bedrock_client
        self._model_id = model_id

    def describe(self, jpeg_b64: str, prompt: str, max_tokens: int = 512) -> str:
        from vlm import vlm_describe
        return vlm_describe(self._bedrock_client, jpeg_b64, prompt, model=self._model_id, max_tokens=max_tokens)


def make_vlm_client() -> VLMClientProtocol:
    """Build the appropriate VLM client from env vars.

    If AGENT_BASE_URL is set, use OpenAI-compatible endpoint.
    Otherwise fall back to legacy Bedrock vlm.py.
    """
    base_url = os.environ.get("AGENT_BASE_URL", "").strip()

    if base_url:
        import openai
        client = openai.OpenAI(
            base_url=base_url,
            api_key=os.environ.get("AGENT_API_KEY", "unused"),
        )
        model = os.environ.get("AGENT_MODEL", "")
        if not model:
            raise RuntimeError("AGENT_MODEL must be set when AGENT_BASE_URL is configured")
        return OpenAICompatVLMClient(client=client, model=model)

    # Legacy fallback
    from vlm import make_client
    bedrock_client = make_client()
    return LegacyBedrockVLMClient(bedrock_client=bedrock_client)
