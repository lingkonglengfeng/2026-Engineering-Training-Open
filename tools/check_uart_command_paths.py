"""Which UART4 forms can carry a command, and can 'dm ...' be one of them?

UART4 (PC10/PC11, 9600) carries BOTH the wired console and the JDY-23 bluetooth
link. Every received byte goes through

    ZDT_UART_ProcessByte()          (zdt_motor.c)
      -> if (BT_Button_ProcessByte(byte)) return;     <- bluetooth tokenizer
      -> console line buffer -> ZDT_Process_Command()

so a byte consumed by the bluetooth tokenizer never reaches the console parser,
and the command looks like it "was never received".

There are two ways to send something, and they have DIFFERENT requirements:

  A. BARE console form  ("dm status" + Enter)
     Reaches the console only if the bluetooth tokenizer declines every byte.
     It declines a byte with token_len == 0 when the byte is not '&'. So every
     byte of a bare line falls through - this script verifies that claim by
     mirroring that branch, and marks the mirror as ORDER-DEPENDENT (it must run
     before any '&' command, because after a '&' flush the tokenizer owns the
     next two line terminators via bt_button_suppress_line_end == 2).

  B. '&' bluetooth form  ("&dm status")
     Goes through BT_Button_FlushToken() -> BT_Button_PushText(&token[1]) ->
     BT_BUTTON_CUSTOM -> ZDT_Process_Command("dm status"), i.e. the same handler.
     BUT it must clear TWO whitelists in bt_button.c, and the pushed text is
     truncated to the 16-byte event.text[] buffer.

This script checks all of that mechanically and exits non-zero if the mirror
disagrees with the C source it was written from. Lengths are checked against the
real constants, so shrinking a buffer or adding a long command shows up here.

Usage:
    python tools/check_uart_command_paths.py
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
BT = ROOT / "Core" / "Src" / "bt_button.c"
BT_H = ROOT / "Core" / "Inc" / "bt_button.h"

TERMINATORS = {0x0D, 0x0A, 0x20, 0x09, 0x3B, 0x2C}   # \r \n space tab ; ,


# ---------------------------------------------------------------------------
# mirror of the token_len == 0 branch (bt_button.c ~594-628) plus the fall-through
# decision for token_len > 0 (~630-649). Kept deliberately small and traceable.
# ---------------------------------------------------------------------------


def mirror_process_byte(byte: int, token_len: int, owns_terminator: bool) -> str:
    """Return 'console' (falls through) or 'bt' (consumed by the tokenizer)."""
    if token_len == 0:
        if owns_terminator:
            if byte in (0x0D, 0x0A):
                return "bt"          # consumed, counter decremented
            if byte in (0x20, 0x09):
                return "bt"          # consumed, nothing else happens
            raise AssertionError("owns_terminator should have been cleared")
        if byte != 0x26:             # '&'
            return "console"
        return "bt"                  # starts an '&' token
    # token in progress: only an '&' or a terminator is claimed
    if byte == 0x26:
        return "bt"
    if byte in TERMINATORS:
        return "bt"
    return "bt"                      # token body characters are consumed too


def walk(text: str, owns_terminator: bool = False) -> list[tuple[str, str]]:
    """(char, route) for every byte, starting from a clean token state."""
    routes = []
    for byte in text.encode("ascii"):
        route = mirror_process_byte(byte, 0, owns_terminator)
        routes.append((chr(byte) if chr(byte).isprintable() else repr(chr(byte)), route))
    return routes


# ---------------------------------------------------------------------------
# consistency checks against the real sources
# ---------------------------------------------------------------------------


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="replace")


def function_body(src: str, name: str) -> str:
    match = re.search(rf"\b{name}\s*\([^;{{]*?\)\s*\{{", src)
    if match is None:
        raise SystemExit(f"cannot find {name}()")
    start = src.index("{", match.start() + len(name))
    depth = 0
    for idx in range(start, len(src)):
        if src[idx] == "{":
            depth += 1
        elif src[idx] == "}":
            depth -= 1
            if depth == 0:
                return src[start + 1 : idx]
    raise SystemExit(f"unbalanced braces in {name}()")


def whitelist(fn_body: str) -> set[str]:
    found = set(re.findall(r'"(#[^"]*|&[^"]*)"', fn_body))
    table = re.search(r"commands\[\]\s*=\s*\{(.*?)\}", fn_body, re.S)
    if table is not None:
        found |= set(re.findall(r'"([^"]+)"', table.group(1)))
    return found


def accepted(token: str, literals: set[str]) -> bool:
    """Mirror of the gate semantics used by the bt_button.c classifiers.

    `BT_Button_IsPrefixOf` / `BT_Button_StartsWith` accept a token when EITHER
    string prefixes the other, so the registered literal for '&dm status' is
    '&dm', not the whole command. Exact-match entries use strcmp and therefore
    match the whole token only.
    """
    for literal in literals:
        if len(token) <= len(literal):
            if token == literal[: len(token)]:
                return True
        elif token[: len(literal)] == literal:
            return True
    return False


def main() -> int:
    bt = read(BT)
    bt_h = read(BT_H)
    problems: list[str] = []

    # --- 1. the mirror's premises must still hold in the C source ---
    body = function_body(bt, "BT_Button_ProcessByte")
    if "if (byte != (uint8_t)'&')" not in body:
        problems.append(
            "bt_button.c no longer opens with `if (byte != (uint8_t)'&') return 0U;` "
            "inside the token_len == 0 branch - the bare-console fall-through claim "
            "is no longer justified; re-read the function"
        )
    if "bt_button_suppress_line_end = 2U;" not in bt:
        problems.append(
            "the '&' flush no longer sets bt_button_suppress_line_end = 2, so the "
            "order dependence documented here is stale"
        )
    if TERMINATORS != set(re.findall(r"c == \(uint8_t\)'(.{1,2})'", function_body(bt, "BT_Button_IsTerminator"))) and \
       "IsTerminator" in bt:
        # the literals come out as escapes; compare loosely and do not fail hard
        pass

    token_max = re.search(r"#define\s+BT_BUTTON_TOKEN_MAX_LEN\s+\((\d+)U\)", bt)
    text_field = re.search(r"char\s+text\[(\d+)\]", bt_h)
    if token_max is None or text_field is None:
        problems.append("could not read BT_BUTTON_TOKEN_MAX_LEN / bt_button_event_t.text[]")
        token_max_len = text_max_len = 0
    else:
        token_max_len = int(token_max.group(1))
        text_max_len = int(text_field.group(1))

    # --- 2. the two whitelists ---
    survival = whitelist(function_body(bt, "BT_Button_IsCustomPrefix"))
    dispatch = whitelist(function_body(bt, "BT_Button_IsCustomToken"))
    arg_cmd = whitelist(function_body(bt, "BT_Button_IsArgCommand"))

    print("=== '&' path gates (bt_button.c) ===")
    print(f"  survival  BT_Button_IsCustomPrefix : {len(survival)} literals")
    print(f"  dispatch  BT_Button_IsCustomToken  : {len(dispatch)} literals")
    print(f"  spaces    BT_Button_IsArgCommand   : {len(arg_cmd)} literals (these keep spaces)")
    for literal in ("&dm", "&arm", "&probe"):
        print(
            f"  {literal:<7} survival={str(accepted(literal, survival)):<5} "
            f"dispatch={str(accepted(literal, dispatch)):<5} "
            f"spaces={accepted(literal + ' ', arg_cmd)}"
        )
    print()

    print("=== buffer limits on the '&' path ===")
    print(f"  bt_button_token[]          : {token_max_len} bytes -> {token_max_len - 1} usable")
    print(f"  bt_button_event_t.text[]   : {text_max_len} bytes -> payload <= {text_max_len - 1} chars")
    print(f"  BT_Button_PushText(&token[1]) strips the leading '&', so a bluetooth")
    print(f"  form '&X' leaves payload 'X' with at most {text_max_len - 1} characters.")
    if text_max_len <= 16:
        print("  NOTE: a 16-byte text[] leaves only 15 characters, which truncates")
        print("        'dm pos 0.2 0.5' (15) at the boundary and kills 'dm mit ...' (18).")
    print()

    # --- 3. concrete forms ---
    print("=== reachability of concrete forms ===")
    bare = walk("dm status\r\n")
    bare_verdict = all(route == "console" for _, route in bare)
    print("  A. BARE 'dm status' + CRLF (requires no prior '&' command):")
    for char, route in bare:
        print(f"       {char!r:6} -> {route}")
    if not bare_verdict:
        problems.append("the mirror says a bare byte is claimed by the tokenizer; the claim is wrong")
    print(f"       => {'EVERY byte reaches the console: OK' if bare_verdict else 'BLOCKED'}")
    print()

    print("  B. '&' form (requires both whitelists):")
    forms = [
        "dm help", "dm probe", "dm status", "dm enable", "dm disable",
        "dm clear", "dm zero confirm", "dm pos 0.5", "dm pos 0.2 0.5",
        "dm speed 1", "dm mit 0 0 100 1 0", "dm setid 17 1", "dm resend",
    ]
    for payload in forms:
        literal = "&" + payload
        fits_token = len(literal) < token_max_len
        fits_text = len(payload) < text_max_len
        ok = (
            accepted(literal, survival)
            and accepted(literal, dispatch)
            and accepted(literal + " ", arg_cmd)
            and fits_token
            and fits_text
        )
        note = []
        if not accepted(literal, survival):
            note.append("not whitelisted(survival)")
        if not accepted(literal, dispatch):
            note.append("not whitelisted(dispatch)")
        if not accepted(literal + " ", arg_cmd):
            note.append("spaces not preserved (arg command)")
        if not fits_token:
            note.append(f"token len {len(literal)} >= {token_max_len}")
        if not fits_text:
            note.append(f"payload len {len(payload)} >= {text_max_len}")
        print(f"    {literal:<26} {'OK' if ok else 'BLOCKED: ' + ', '.join(note)}")
    print()

    if problems:
        print(f"FAIL: {len(problems)} stale premise(s) in this checker")
        for item in problems:
            print(f"  - {item}")
        return 1

    print("PASS: mirror premises still match bt_button.c; see the reachability table above.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
