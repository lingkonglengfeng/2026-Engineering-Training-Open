"""Find serial format strings that end a line with a bare '\\n' instead of
'\\r\\n'.

A line feed without a carriage return leaves the cursor in the same column in
many terminals and bluetooth serial apps, so the next line starts indented -
which looks exactly like the "characters interleaved / shifted" log corruption
this project has been chasing. Other places in the firmware already use \\r\\n,
so this is about consistency as much as correctness.

Exit code 0 = every print literal uses \\r\\n.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SOURCES = sorted(ROOT.glob("Core/**/*.c"))

BLOCK_COMMENT = re.compile(r"/\*.*?\*/", re.S)
LINE_COMMENT = re.compile(r"//[^\n]*")
LITERAL = re.compile(r'"((?:[^"\\]|\\.)*)"')


def code_only(text: str) -> str:
    text = BLOCK_COMMENT.sub(lambda m: "\n" * m.group(0).count("\n"), text)
    return LINE_COMMENT.sub("", text)


found = []
for path in SOURCES:
    # Scan whole statements, not single lines: the format string of a wrapped
    # snprintf() call usually sits on its own line, far from the call name.
    text = code_only(path.read_text(encoding="utf-8"))
    for match in LITERAL.finditer(text):
        literal = match.group(1)
        if r"\n" in literal and r"\r\n" not in literal:
            lineno = text.count("\n", 0, match.start()) + 1
            found.append((path.relative_to(ROOT), lineno, literal))

print(f"string literals with a bare \\n: {len(found)}")
for path, lineno, literal in found:
    print(f"  {path}:{lineno}: {literal}")

sys.exit(1 if found else 0)
