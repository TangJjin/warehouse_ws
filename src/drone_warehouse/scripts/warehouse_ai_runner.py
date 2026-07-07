#!/usr/bin/env python3
import json
import os
import sys
from pathlib import Path

import requests


def get_media_type(image_format: str) -> str:
    normalized = image_format.strip().lower().lstrip(".")
    if normalized in ("jpg", "jpeg"):
        return "image/jpeg"
    if normalized == "png":
        return "image/png"
    if normalized == "webp":
        return "image/webp"
    if normalized == "gif":
        return "image/gif"
    return "image/jpeg"


def main() -> int:
    if len(sys.argv) != 4:
        print(
            "usage: warehouse_ai_runner.py <prompt_path> <image_meta_path> <output_path>",
            file=sys.stderr,
        )
        return 1

    prompt_path = Path(sys.argv[1])
    image_meta_path = Path(sys.argv[2])
    output_path = Path(sys.argv[3])

    if not prompt_path.exists():
        print(f"prompt file not found: {prompt_path}", file=sys.stderr)
        return 1

    if not image_meta_path.exists():
        print(f"image meta file not found: {image_meta_path}", file=sys.stderr)
        return 1

    api_key = os.environ.get("ANTHROPIC_AUTH_TOKEN", "").strip()
    base_url = os.environ.get("ANTHROPIC_BASE_URL", "").strip().rstrip("/")
    model_name = os.environ.get("ANTHROPIC_MODEL", "").strip()

    if not api_key:
        print("ANTHROPIC_AUTH_TOKEN is empty", file=sys.stderr)
        return 1

    if not base_url:
        print("ANTHROPIC_BASE_URL is empty", file=sys.stderr)
        return 1

    if not model_name:
        print("ANTHROPIC_MODEL is empty", file=sys.stderr)
        return 1

    prompt_text = prompt_path.read_text(encoding="utf-8")

    try:
        image_meta = json.loads(image_meta_path.read_text(encoding="utf-8"))
    except Exception as e:
        print(f"failed to parse image meta json: {e}", file=sys.stderr)
        return 1

    mode = str(image_meta.get("mode", "")).strip()

    if mode == "text_summary":
        payload = {
            "model": model_name,
            "max_tokens": 1200,
            "temperature": 0.2,
            "messages": [
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "text",
                            "text": prompt_text,
                        }
                    ],
                }
            ],
        }
    else:
        image_base64 = str(image_meta.get("image_base64", "")).strip()
        image_format = str(image_meta.get("image_format", "")).strip()

        if not image_base64:
            print("image_base64 is empty", file=sys.stderr)
            return 1

        if not image_format:
            print("image_format is empty", file=sys.stderr)
            return 1

        slot = str(image_meta.get("slot", "")).strip()
        manual_package = str(image_meta.get("manual_package", "")).strip()
        manual_category = str(image_meta.get("manual_category", "")).strip()
        observed_package = str(image_meta.get("observed_package", "")).strip()
        observed_category = str(image_meta.get("observed_category", "")).strip()
        observed_slot = str(image_meta.get("observed_slot", "")).strip()
        observed_time = str(image_meta.get("observed_time", "")).strip()

        context_lines = [
            prompt_text,
            "",
            "槽位上下文：",
            f"- slot={slot}",
            f"- manual_package={manual_package}",
            f"- manual_category={manual_category}",
            f"- observed_package={observed_package}",
            f"- observed_category={observed_category}",
            f"- observed_slot={observed_slot}",
            f"- observed_time={observed_time}",
            "",
            "要求：",
            "- 优先依据图片本身判断，不要只复述上下文字段。",
            "- 如果图片无法支持判断，请明确写无法确认。",
            "- 不要臆造未看见的内容。",
        ]
        final_prompt_text = "\n".join(context_lines)

        payload = {
            "model": model_name,
            "max_tokens": 2000,
            "temperature": 0.2,
            "messages": [
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "text",
                            "text": final_prompt_text,
                        },
                        {
                            "type": "image",
                            "source": {
                                "type": "base64",
                                "media_type": get_media_type(image_format),
                                "data": image_base64,
                            },
                        },
                    ],
                }
            ],
        }

    url = f"{base_url}/v1/messages"
    headers = {
        "x-api-key": api_key,
        "anthropic-version": "2023-06-01",
        "content-type": "application/json",
    }

    try:
        response = requests.post(url, headers=headers, json=payload, timeout=120)
    except Exception as e:
        print(f"request failed: {e}", file=sys.stderr)
        return 1

    if not response.ok:
        print(f"status={response.status_code}", file=sys.stderr)
        print(response.text, file=sys.stderr)
        return 1

    try:
        data = response.json()
    except Exception as e:
        print(f"invalid json response: {e}", file=sys.stderr)
        print(response.text, file=sys.stderr)
        return 1

    text_parts = []
    for block in data.get("content", []):
        if isinstance(block, dict) and block.get("type") == "text":
            text_parts.append(block.get("text", ""))

    final_text = "\n".join(text_parts).strip()
    output_path.write_text(final_text, encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())