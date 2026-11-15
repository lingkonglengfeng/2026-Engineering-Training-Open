"""One-off scan: non-ASCII string literals in firmware sources.

The project rule is that every *serial* string must be ASCII (the manual's
pitfall table): ARMCC complains about multibyte string literals and, worse, the
bytes that reach the app are up to the app's decoder. Comments may be Chinese.

Usage: python tools/scan_ascii_strings.py
Exit code 0 = no offending literal found.
"""

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

BLOCK_COMMENT = re.compile(r"/\*.*?\*/", re.S)
LINE_COMMENT = re.compile(r"//[^\n]*")


def code_only(text: str) -> str:
    """Strip comments (keeping newlines) so literal text in prose is not flagged."""
    text = BLOCK_COMMENT.sub(lambda m: "\n" * m.group(0).count("\n"), text)
    return LINE_COMMENT.sub("", text)


found = []
for path in sorted(ROOT.glob("Core/**/*.c")) + sorted(ROOT.glob("Core/**/*.h")):
    text = code_only(path.read_text(encoding="utf-8", errors="replace"))
    for lineno, line in enumerate(text.splitlines(), 1):
        for literal in re.findall(r'"((?:[^"\\]|\\.)*)"', line):
            if any(ord(ch) > 127 for ch in literal):
                found.append((path.relative_to(ROOT), lineno, literal))

print(f"non-ASCII string literals (comments stripped): {len(found)}")
for path, lineno, literal in found:
    print(f"  {path}:{lineno}: {literal}")

sys.exit(1 if found else 0)
