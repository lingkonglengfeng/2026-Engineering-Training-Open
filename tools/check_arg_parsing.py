"""Guard the bluetooth argument parsing in zdt_motor.c against fixed-offset reads.

Why this exists: the bluetooth tokenizer PRESERVES spaces inside a token (that is
deliberate - it is what keeps `&t1000 2100` and `&startpose 2` in one piece), so
an argument never sits at a guaranteed offset. The old code did

    uint8_t zone = (event.text[9] == '2') ? 2U : 1U;

and index 9 of "startpose 2" is the SPACE, so the documented `&startpose 2`
silently selected zone 1 - only `&startpose2` (no space) worked. In the field
that meant the car sat in the top-right zone while the firmware believed the
bottom-right one: `&leave` printed a perfectly plausible landing pose while
physically shoving the car 250 mm too far right (half off the mat), and the
following `&goto` drove straight off the field.

Rule enforced here: inside the bluetooth command handler, if a handler reads
`<expr>[N]` where N equals the length of the command word it just matched, the
argument is being read at a fixed offset, so the handler MUST parse it through
ZDT_SkipSpaces() (or an explicit skip-whitespace loop) instead of guessing.
Scoped to zdt_motor.c on purpose: grid_nav.c/pid_storage.c parse their own
command strings with their own documented "space required" forms, which is a
different (working) convention.

Exit code 0 = every fixed-offset argument read is guarded.

LIMITATION: this catches `<expr>[N]` reads. A `<expr> + N` pointer offset is not
detected - it is a net, not a proof, so still read the handler when adding a
command with an argument.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
TARGET = ROOT / "Core" / "Src" / "zdt_motor.c"

# Expression compared against a command word, e.g. strncmp(event.text, "nudge", 5)
COMPARE = re.compile(r'(?:strncmp|strcmp)\s*\(\s*([\w.>\-]+)\s*,\s*"([^"]+)"')
# Index into an expression, e.g. event.text[9] / command[5]
INDEX = re.compile(r"([\w.>\-]+)\s*\[\s*(\d+)\s*\]")
# Any explicit whitespace skip counts as a proper argument read.
GUARD = re.compile(r"ZDT_SkipSpaces\s*\(|while\s*\(\s*\*\s*\w+\s*==\s*' '")

# Deliberate exception: `posmode 0/1` is a console-only command whose documented
# form REQUIRES the space, and the console path never goes through the
# space-preserving bluetooth tokenizer, so reading command[7] is safe there.
ALLOWED = {"posmode"}


def block_after(text: str, start: int) -> str:
    """Brace-balanced body of the `if (...) { ... }` starting at/after `start`."""
    brace = text.find("{", start)
    if brace < 0:
        return ""
    depth = 0
    for idx in range(brace, len(text)):
        if text[idx] == "{":
            depth += 1
        elif text[idx] == "}":
            depth -= 1
            if depth == 0:
                return text[brace : idx + 1]
    return text[brace:]


text = TARGET.read_text(encoding="utf-8", errors="replace")

found = []
for match in COMPARE.finditer(text):
    expr, command = match.group(1), match.group(2)
    if command in ALLOWED:
        continue
    # Scope the guard search to THIS handler only: a neighbouring handler that
    # happens to call the helper must not excuse a fixed-offset read here.
    block = block_after(text, match.end())
    for index_match in INDEX.finditer(block):
        if index_match.group(1) != expr:
            continue
        if int(index_match.group(2)) != len(command):
            continue
        if GUARD.search(block):
            continue
        lineno = text.count("\n", 0, match.start()) + 1
        found.append((lineno, command, expr, index_match.group(2)))

print(f"unguarded fixed-offset argument reads in zdt_motor.c: {len(found)}")
for lineno, command, expr, index in found:
    print(f"  zdt_motor.c:{lineno}: {expr}[{index}] reads the argument of "
          f'"{command}" at a fixed offset - spaces inside a bluetooth token make '
          f"that offset wrong; skip whitespace and report usage instead")

sys.exit(1 if found else 0)
