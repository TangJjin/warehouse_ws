#!/usr/bin/env python3
import json
import os
import sys
from pathlib import Path

import requests


def main() -> int:
    if len(sys.argv) != 3:
        print("usage: warehouse_ai_runner.py <prompt_path> <output_path>", file=sys.stderr)
        return 1

    prompt_path = Path(sys.argv[1])
    output_path = Path(sys.argv[2])

    if not prompt_path.exists():
        print(f"prompt file not found: {prompt_path}", file=sys.stderr)
        return 1

    api_key = os.environ.get("ANTHROPIC_API_KEY", "")
    if not api_key:
        print("ANTHROPIC_API_KEY is empty", file=sys.stderr)
        return 1

    prompt = prompt_path.read_text(encoding="utf-8")

    url = "https://api.anthropic.com/v1/messages"
    headers = {
        "x-api-key": api_key,
        "anthropic-version": "2023-06-01",
        "content-type": "application/json",
    }

    payload = {
        "model": "claude-sonnet-4-6",
        "max_tokens": 2000,
        "temperature": 0.2,
        "messages": [
            {
                "role": "user",
                "content": prompt,
            }
        ],
    }

    response = requests.post(url, headers=headers, json=payload, timeout=60)
    response.raise_for_status()
    data = response.json()

    text_parts = []
    for block in data.get("content", []):
        if block.get("type") == "text":
            text_parts.append(block.get("text", ""))

    final_text = "\n".join(text_parts).strip()
    output_path.write_text(final_text, encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
