"""Environment-backed gateway configuration."""

from dataclasses import dataclass
import json
import os
from pathlib import Path


def _expand(value: str) -> str:
    return str(Path(value).expanduser().resolve())


def _enabled(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _csv(value: str) -> tuple[str, ...]:
    return tuple(item.strip() for item in value.split(",") if item.strip())


def _identity_bindings(value: str) -> tuple[tuple[str, str], ...]:
    if not value.strip():
        return ()
    payload = json.loads(value)
    if not isinstance(payload, dict):
        raise ValueError("AEROMIND_IDENTITY_BINDINGS 必须是 JSON 对象")
    return tuple(
        (str(alias).strip(), str(principal).strip())
        for alias, principal in payload.items()
        if str(alias).strip() and str(principal).strip()
    )


@dataclass(frozen=True)
class OpenAIProviderConfig:
    name: str
    base_url: str
    api_key: str
    models: tuple[str, ...]


def _openai_providers(value: str) -> tuple[OpenAIProviderConfig, ...]:
    if value.strip():
        payload = json.loads(value)
        if not isinstance(payload, dict):
            raise ValueError("AEROMIND_OPENAI_PROVIDERS 必须是 JSON 对象")
    else:
        payload = {
            "deepseek": {
                "base_url": os.environ.get(
                    "DEEPSEEK_API_URL", "https://api.deepseek.com/v1"
                ),
                "api_key_env": "DEEPSEEK_API_KEY",
                "models": ["deepseek-chat"],
            },
            "qwen": {
                "base_url": os.environ.get(
                    "DASHSCOPE_API_URL",
                    "https://dashscope.aliyuncs.com/compatible-mode/v1",
                ),
                "api_key_env": "DASHSCOPE_API_KEY",
                "models": ["qwen-plus"],
            },
        }
    providers = []
    for name, item in payload.items():
        if not isinstance(item, dict):
            continue
        clean_name = str(name).strip().lower()
        base_url = str(item.get("base_url", "")).strip().rstrip("/")
        models = tuple(
            str(model).strip()
            for model in item.get("models", [])
            if str(model).strip()
        )
        api_key_env = str(item.get("api_key_env", "")).strip()
        api_key = str(item.get("api_key", "")).strip()
        if not api_key and api_key_env:
            api_key = os.environ.get(api_key_env, "").strip()
        if clean_name and base_url and models:
            providers.append(
                OpenAIProviderConfig(clean_name, base_url, api_key, models)
            )
    return tuple(providers)


@dataclass(frozen=True)
class GatewayConfig:
    host: str
    port: int
    database_path: str
    workspace: str
    access_token: str
    default_model: str
    max_turns: int
    max_budget_usd: float
    default_provider: str = "claude"
    openai_providers: tuple[OpenAIProviderConfig, ...] = ()
    feishu_enabled: bool = False
    feishu_app_id: str = ""
    feishu_app_secret: str = ""
    feishu_allowed_open_ids: tuple[str, ...] = ()
    feishu_image_max_bytes: int = 10 * 1024 * 1024
    vlm_api_url: str = ""
    vlm_api_key: str = ""
    vlm_model: str = ""
    vlm_timeout_sec: float = 30.0
    identity_bindings: tuple[tuple[str, str], ...] = ()
    memory_enabled: bool = True
    memory_message_limit: int = 12
    semantic_memory_enabled: bool = False
    semantic_memory_interval: int = 8
    semantic_memory_model: str = "haiku"
    semantic_memory_budget_usd: float = 0.05

    def resolve_identity(self, user_id: str) -> str:
        return dict(self.identity_bindings).get(user_id, user_id)

    def openai_provider(self, name: str) -> OpenAIProviderConfig | None:
        return next(
            (provider for provider in self.openai_providers if provider.name == name),
            None,
        )

    @classmethod
    def from_env(cls) -> "GatewayConfig":
        return cls(
            host=os.environ.get("AEROMIND_AGENT_HOST", "0.0.0.0"),
            port=int(os.environ.get("AEROMIND_AGENT_PORT", "8090")),
            database_path=_expand(
                os.environ.get(
                    "AEROMIND_AGENT_DB",
                    "~/.aeromind/agent_gateway.db",
                )
            ),
            workspace=_expand(
                os.environ.get("AEROMIND_WORKSPACE", "~/aeromind_ws")
            ),
            access_token=os.environ.get("AEROMIND_AGENT_TOKEN", "").strip(),
            default_model=os.environ.get(
                "AEROMIND_AGENT_MODEL",
                os.environ.get("CLAUDE_AGENT_MODEL", "sonnet"),
            ).strip()
            or "sonnet",
            max_turns=max(1, int(os.environ.get("CLAUDE_AGENT_MAX_TURNS", "12"))),
            max_budget_usd=max(
                0.0, float(os.environ.get("CLAUDE_AGENT_MAX_BUDGET_USD", "1.0"))
            ),
            default_provider=os.environ.get(
                "AEROMIND_AGENT_PROVIDER", "claude"
            ).strip().lower()
            or "claude",
            openai_providers=_openai_providers(
                os.environ.get("AEROMIND_OPENAI_PROVIDERS", "")
            ),
            feishu_enabled=_enabled(os.environ.get("FEISHU_ENABLED", "false")),
            feishu_app_id=os.environ.get("FEISHU_APP_ID", "").strip(),
            feishu_app_secret=os.environ.get("FEISHU_APP_SECRET", "").strip(),
            feishu_allowed_open_ids=_csv(
                os.environ.get("FEISHU_ALLOWED_OPEN_IDS", "")
            ),
            feishu_image_max_bytes=max(
                1024,
                int(os.environ.get("FEISHU_IMAGE_MAX_BYTES", str(10 * 1024 * 1024))),
            ),
            vlm_api_url=os.environ.get("AEROMIND_VLM_API_URL", "").strip(),
            vlm_api_key=os.environ.get("AEROMIND_VLM_API_KEY", "").strip(),
            vlm_model=os.environ.get("AEROMIND_VLM_MODEL", "").strip(),
            vlm_timeout_sec=max(
                1.0, float(os.environ.get("AEROMIND_VLM_TIMEOUT_SEC", "30"))
            ),
            identity_bindings=_identity_bindings(
                os.environ.get("AEROMIND_IDENTITY_BINDINGS", "")
            ),
            memory_enabled=_enabled(
                os.environ.get("AEROMIND_MEMORY_ENABLED", "true")
            ),
            memory_message_limit=max(
                2, min(50, int(os.environ.get("AEROMIND_MEMORY_MESSAGE_LIMIT", "12")))
            ),
            semantic_memory_enabled=_enabled(
                os.environ.get("AEROMIND_SEMANTIC_MEMORY_ENABLED", "false")
            ),
            semantic_memory_interval=max(
                2,
                min(
                    50,
                    int(os.environ.get("AEROMIND_SEMANTIC_MEMORY_INTERVAL", "8")),
                ),
            ),
            semantic_memory_model=os.environ.get(
                "AEROMIND_SEMANTIC_MEMORY_MODEL", "haiku"
            ).strip()
            or "haiku",
            semantic_memory_budget_usd=max(
                0.01,
                float(
                    os.environ.get("AEROMIND_SEMANTIC_MEMORY_BUDGET_USD", "0.05")
                ),
            ),
        )
