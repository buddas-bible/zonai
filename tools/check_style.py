from __future__ import annotations

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOTS = ("src", "tests", "sandbox")
SUFFIXES = {".h", ".hpp", ".cpp", ".cxx", ".cc"}


def iter_source_files():
    for root_name in SOURCE_ROOTS:
        root = ROOT / root_name
        if not root.exists():
            continue

        for path in root.rglob("*"):
            if path.is_file() and path.suffix.lower() in SUFFIXES:
                yield path


def main() -> int:
    errors: list[str] = []

    for path in iter_source_files():
        data = path.read_bytes()
        relative = path.relative_to(ROOT)

        if data.startswith(b"\xef\xbb\xbf"):
            errors.append(f"{relative}: UTF-8 BOM is not allowed")
            continue

        try:
            text = data.decode("utf-8")
        except UnicodeDecodeError as error:
            errors.append(f"{relative}: not valid UTF-8 ({error})")
            continue

        if "\r" in text:
            errors.append(f"{relative}: CRLF/CR line endings are not allowed")

        if "\t" in text:
            errors.append(f"{relative}: tab character found")

        lines = text.splitlines()

        for line_number, line in enumerate(lines, start=1):
            if line.rstrip(" ") != line:
                errors.append(
                    f"{relative}:{line_number}: trailing whitespace"
                )

        if text and not text.endswith("\n"):
            errors.append(f"{relative}: missing final newline")

        if path.suffix.lower() in {".h", ".hpp"} and lines:
            if lines[0] == "#pragma once":
                if len(lines) < 2 or lines[1] != "":
                    errors.append(
                        f"{relative}: #pragma once must be followed by one blank line"
                    )

    if errors:
        print("Zonai style check failed:")
        for error in errors:
            print(f"  {error}")
        return 1

    print("Zonai style check passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
