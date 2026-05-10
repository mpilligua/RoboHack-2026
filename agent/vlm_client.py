"""VLM client abstraction for the Bedrock-native agent."""

from __future__ import annotations

from typing import Protocol, runtime_checkable


@runtime_checkable
class VLMClientProtocol(Protocol):
    def describe(self, jpeg_b64: str, prompt: str, max_tokens: int = 512) -> str: ...


class LegacyBedrockVLMClient:
    """Thin wrapper around legacy vlm.vlm_describe()."""

    def __init__(self, bedrock_client, model_id: str | None = None) -> None:
        self._bedrock_client = bedrock_client
        self._model_id = model_id

    def describe(self, jpeg_b64: str, prompt: str, max_tokens: int = 512) -> str:
        from vlm import vlm_describe

        return vlm_describe(
            self._bedrock_client,
            jpeg_b64,
            prompt,
            model=self._model_id,
            max_tokens=max_tokens,
        )


def make_vlm_client() -> VLMClientProtocol:
    from vlm import make_client

    bedrock_client = make_client()
    return LegacyBedrockVLMClient(bedrock_client=bedrock_client)

