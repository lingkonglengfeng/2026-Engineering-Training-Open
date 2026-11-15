"""Cross-check every command the bluetooth handler can dispatch against the two
whitelists in bt_button.c, **without hardware**.

Why this exists: a bluetooth `&` command has to clear TWO separate gates, and
registering only one of them produces a silent drop that looks exactly like
"the command was never received":

  1. tokenizer survival (per received byte): if none of IsShortAnglePrefix /
     IsArgCommand / IsCustomPrefix / IsCustomToken matches the partial token,
     `bt_button_token_invalid` is set and the token can never be dispatched.
     `IsCustomPrefix` also decides the idle-timeout: a token that is neither a
     grid/value prefix nor a custom prefix gets delay 0, i.e. it is NEVER
     flushed by timeout and only goes out when the next '&' arrives (the classic
     "you have to send it twice / it answers the previous command" bug).
  2. dispatch (at flush time): BT_Button_FlushToken() dispatches a text command
     ONLY if BT_Button_IsCustomToken() accepts the complete token; otherwise it
     falls through to the button table and counts invalid_count++ **printing
     nothing at all**.

This bit us three times (SS4.12 `&cnt`/&gridclear/&show, SS4.20 `&goto raw`,
`&echo`), so the check is mechanical now. Keep it green before flashing.

Exit code 0 = every dispatched command is reachable; 1 = at least one gap.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
BT = ROOT / "Core" / "Src" / "bt_button.c"
ZDT = ROOT / "Core" / "Src" / "zdt_motor.c"

# Commands that are deliberately NOT reachable over bluetooth (console only).
# Keep this list explicit and justified, never "just make the check pass".
INTENTIONAL_CONSOLE_ONLY = {
    # &delete erases the PID sector: too destructive to expose to a phone app.
    "delete",
}


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def function_body(src: str, signature_re: str) -> str:
    """Return the brace-balanced body of the first function matching signature."""
    match = re.search(signature_re, src)
    if match is None:
        raise SystemExit(f"cannot find function matching {signature_re!r}")
    start = src.index("{", match.start())
    depth = 0
    for idx in range(start, len(src)):
        if src[idx] == "{":
            depth += 1
        elif src[idx] == "}":
            depth -= 1
            if depth == 0:
                return src[start : idx + 1]
    raise SystemExit(f"unbalanced braces after {signature_re!r}")


def string_array(body: str) -> list[str]:
    """Literals of a `static const char *commands[] = { "&a", "&b" };` table."""
    table = re.search(r"commands\[\]\s*=\s*\{(.*?)\}", body, re.S)
    if table is None:
        return []
    return re.findall(r'"([^"]+)"', table.group(1))


def prefix_literals(body: str) -> list[str]:
    """Literals passed to BT_Button_IsPrefixOf / BT_Button_StartsWith."""
    return re.findall(r'"(#[^"]*|&[^"]*)"', body)


def compare_literals(body: str) -> list[str]:
    """Literals used in strcmp/strncmp comparisons inside a parser.

    Returns them together with the parameter they are compared against, because
    the two parser shapes in this repo need different treatment:
      * command-word parsers: strcmp(command, "leave")  -> the literal IS the
        bluetooth command word, so it becomes "&leave".
      * verb parsers:         strcmp(verb, "status")    -> dm_motor.c splits
        `dm <verb> ...`, so the literal is a SUB-verb: on its own it is not a
        reachable command, and treating it as one makes the checker demand that
        "&status" be whitelisted. The caller prefixes those with their parent.
    """
    return re.findall(
        r'(?:strcmp|strncmp)\s*\(\s*(\w+)\s*,\s*"([^"]+)"', body
    )


def starts_with(token: str, prefix: str) -> bool:
    """Mirror of BT_Button_StartsWith(): true if either string prefixes the other."""
    if len(token) <= len(prefix):
        return token == prefix[: len(token)]
    return token[: len(prefix)] == prefix


def is_grid_command(token: str, grid: list[str]) -> bool:
    """Mirror of BT_Button_IsGridCommand(): prefix + allowed trailing charset."""
    for command in grid:
        if len(token) >= len(command) and token.startswith(command):
            rest = token[len(command) :]
            if all(c.isdigit() or c.islower() or c in " ,-+" for c in rest):
                return True
    return False


def is_custom_token(token: str, grid: list[str], starts: list[str],
                    exact: list[str]) -> bool:
    """Mirror of BT_Button_IsCustomToken() — the flush-time dispatch gate."""
    if is_grid_command(token, grid):
        return True
    if token in exact:
        return True
    if any(starts_with(token, prefix) for prefix in starts):
        return True
    if len(token) > 3 and token[:3] in ("&kp", "&ki", "&kd"):
        return all(c.isdigit() or c in ".-+" for c in token[3:])
    return False


def main() -> int:
    bt = read(BT)
    zdt = read(ZDT)

    grid = string_array(function_body(bt, r"BT_Button_IsGridPrefix\s*\("))
    value = prefix_literals(function_body(bt, r"BT_Button_IsValuePrefix\s*\("))
    custom_prefix = prefix_literals(function_body(bt, r"BT_Button_IsCustomPrefix\s*\("))
    token_body = function_body(bt, r"BT_Button_IsCustomToken\s*\(")
    custom_starts = prefix_literals(token_body)
    custom_exact = re.findall(r'strcmp\s*\(\s*token\s*,\s*"([^"]+)"\s*\)', token_body)

    case = function_body(zdt, r"case BT_BUTTON_CUSTOM\s*:")
    handled_exact = re.findall(r'strcmp\s*\(\s*event\.text\s*,\s*"([^"]+)"', case)
    handled_prefix = re.findall(r'strncmp\s*\(\s*event\.text\s*,\s*"([^"]+)"', case)
    # The delegated call may carry extra arguments before event.text, e.g.
    # DM_Console_ProcessCommand(ArmTask_GetMotor(), event.text).
    delegated = re.findall(
        r"(\w+)_ProcessCommand\s*\(\s*(?:[^,()]+(?:\([^()]*\))?\s*,\s*)?event\.text\s*\)",
        case,
    )

    module_files = {
        "Nav_Params": ROOT / "Core" / "Src" / "nav_params.c",
        "GridNav": ROOT / "Core" / "Src" / "grid_nav.c",
        "PID_Storage": ROOT / "Core" / "Src" / "pid_storage.c",
        "Vision": ROOT / "Core" / "Src" / "vision.c",
        # 2026-09-14: the Damiao turntable parser is reached over bluetooth as
        # `&dm ...` (registered in bt_button.c's three classifiers and routed in
        # the BT_BUTTON_CUSTOM handler).
        "DM_Console": ROOT / "Core" / "Src" / "dm_motor.c",
    }
    # Modules whose parser splits a verb off the front, so their literals are
    # SUB-verbs and must be prefixed with the parent command word to name a
    # reachable bluetooth command ("dm status", not "status").
    verb_parser_prefix = {"DM_Console": "dm "}

    delegated_commands: dict[str, list[str]] = {}
    for module in delegated:
        path = module_files.get(module)
        if path is None:
            raise SystemExit(f"unknown delegated parser {module!r} in the BT handler")
        body = function_body(read(path), rf"{module}_ProcessCommand\s*\(")
        prefix = verb_parser_prefix.get(module, "")
        names = set()
        for param, literal in compare_literals(body):
            if param in ("command", "token"):
                names.add(literal)
            elif prefix:
                # a sub-verb of a verb parser: only meaningful with its parent
                names.add(prefix + literal)
            # else: a literal compared against some other variable (not the
            # command word), so it does not name a command - skip it instead of
            # inventing an "&<literal>" requirement.
        delegated_commands[module] = sorted(names)

    # Required = anything the BT handler can act on, from any source.
    required: dict[str, str] = {}
    for name in handled_exact:
        required.setdefault(name, "handler strcmp")
    for name in handled_prefix:
        required.setdefault(name, "handler strncmp")
    for module, names in delegated_commands.items():
        for name in names:
            required.setdefault(name, f"{module}_ProcessCommand")
    for param, literal in compare_literals(function_body(zdt, r"static uint8_t ZDT_Align_Command\s*\([^;]*?\)\s*\{")):
        if param == "command":
            required.setdefault(literal, "ZDT_Align_Command")
    for param, literal in compare_literals(function_body(zdt, r"static uint8_t ZDT_Qr_Command\s*\([^;]*?\)\s*\{")):
        if param == "command":
            required.setdefault(literal, "ZDT_Qr_Command")
    # These project-owner handlers are reached before the legacy delegated
    # modules. Include their actual source literals instead of treating new
    # namespaces / convenience aliases as an unhandled whitelist prefix.
    if "ZDT_UserQuery_Command(event.text)" in case:
        body = function_body(zdt, r"static uint8_t ZDT_UserQuery_Command\s*\([^;]*?\)\s*\{")
        for param, literal in compare_literals(body):
            if param == "command": required.setdefault(literal, "ZDT_UserQuery_Command")
        for prefix in re.findall(r'sscanf\s*\(\s*command\s*,\s*"([^%"]+)%', body):
            required.setdefault(prefix.strip(), "ZDT_UserQuery_Command")
    if "ZDT_ArmCommand(event.text)" in case:
        body = function_body(read(ROOT / "Core/Src/arm_calibration.c"), r"uint8_t ArmCalibration_ProcessCommand\s*\([^;]*?\)\s*\{")
        for param, literal in compare_literals(body):
            if param == "line" and literal.startswith("dm "):
                required.setdefault(literal, "ArmCalibration_ProcessCommand")

    match_body=function_body(zdt, r"static uint8_t ZDT_Match_Command\s*\([^;]*?\)\s*\{")
    for param,literal in compare_literals(match_body):
        if param=="command": required.setdefault(literal,"ZDT_Match_Command")
    body=function_body(zdt,r"static uint8_t ZDT_VyawCommand\s*\([^;]*?\)\s*\{")
    for param,literal in compare_literals(body):
        if param=="command": required.setdefault(literal,"ZDT_VyawCommand")
    for name in ("vyaw view 2","vyaw zero 2","vyaw sample 2","vyaw solve 2","vyaw gain 2 -1"):
        required.setdefault(name,"yaw calibration argument examples")
    body=function_body(zdt,r"static uint8_t ZDT_Outer_Command\s*\([^;]*?\)\s*\{")
    for param,literal in compare_literals(body):
        if param=="command": required.setdefault(literal,"ZDT_Outer_Command")
    for name in ("outerarc radius 150","outerarc speed 140"):
        required.setdefault(name,"outer arc RAM trial argument examples")
    for name in ("arm grip test 186","arm grip test 224.5","arm grip angle 238","arm grip open","arm grip close","arm grip status","arm grip disable"):
        required.setdefault(name,"grip angle reference/test examples")
    for name in ("arm round1 status","arm material","arm material red","arm material blue","arm material status","arm material stop",
                 "arm material loop start","arm material loop stop","arm material loop status",
                 "arm material loop limit 0","arm material loop limit 2",
                 "arm material auto on","arm material auto off","arm material window 0 0 12 12",
                 "arm material fresh 150","arm material confirm 1","arm material confirm 2","arm material jump 0"):
        required.setdefault(name,"red material owner command examples")
    # Complete user-facing examples cover argument tokens and the owner parser.
    for name in ("arm color","arm color 1","arm color 6","arm color status","arm color stop"):
        required.setdefault(name,"CAMERA pose and CAM1 color learning examples")
    for name in ("prepare 1","prepare 2","arm query 5","arm query 6","arm query dm",
                 "arm pos 6 -185.1581",
                 "arm zero 5","arm zero 6","arm zero dm","arm enable 5","arm disable 6",
                 "arm start","arm camera","arm pick","arm pick status","arm place","arm place 1","arm place 2","arm place 3","arm place status","arm ring","arm ring 1","arm ring 2","arm ring 3","arm ring confirm","arm ring status","arm table test 385","arm table go 1","arm table go 2","arm table go 3","arm start set -30 -200 10","arm camera set -30 -200 10"):
        required.setdefault(name,"pose/prepare argument examples")

    print(f"bt_button.c whitelists: grid={len(grid)} value={len(value)} "
          f"custom_prefix={len(custom_prefix)} custom_token="
          f"{len(custom_starts) + len(custom_exact)}")
    print(f"bluetooth handler dispatches: {len(required)} commands "
          f"(exact {len(handled_exact)}, prefix {len(handled_prefix)}, "
          f"delegated {sum(len(v) for v in delegated_commands.values())})")
    print()

    named_buttons=set(re.findall(r'\{\s*"(&[^\"]+)"\s*,\s*BT_BUTTON_',bt))
    problems: list[str] = []

    for name in sorted(required):
        token = "&" + name
        dispatches = token in named_buttons or is_custom_token(token, grid, custom_starts, custom_exact)
        survives = dispatches or any(
            starts_with(token, prefix)
            for prefix in custom_prefix + value
        ) or is_grid_command(token, grid)
        if dispatches:
            state = "ok"
        elif name in INTENTIONAL_CONSOLE_ONLY:
            state = "console-only (documented)"
        else:
            state = "DROPPED AS GARBAGE"
            problems.append(
                f"{token} is dispatched by the BT handler "
                f"({required[name]}) but BT_Button_IsCustomToken() rejects it: "
                f"the token is counted invalid and nothing is printed "
                f"(tokenizer survival: {'yes' if survives else 'NO'})"
            )
        print(f"  {token:<12} {state:<26} via {required[name]}")

    print()

    # Prefix-only registration: the tokenizer keeps the token alive, the flush
    # gate then throws it away — the exact failure mode of &echo before v2.36.
    for prefix in sorted(set(custom_prefix)):
        if not is_custom_token(prefix, grid, custom_starts, custom_exact):
            name = prefix.lstrip("&")
            if name in INTENTIONAL_CONSOLE_ONLY:
                continue
            problems.append(
                f"{prefix} is in BT_Button_IsCustomPrefix() but not accepted by "
                f"BT_Button_IsCustomToken(): typing it never flags the token "
                f"invalid (so it looks accepted) and the flush then drops it"
            )
            print(f"  {prefix:<12} PREFIX-ONLY (flush drops it)")

    # The other direction: accepted (and so answered) but nothing handles it,
    # which only prints `[BT] unknown command: '...'`.
    #
    # A registered literal can be a PREFIX of the real commands rather than a
    # command itself: bt_button.c registers "&dm" (the literal for `&dm pos` /
    # `&dm status` / ...), so "dm" has no handler OF ITS OWN but is not dead -
    # the check must ask whether it prefixes anything that IS dispatched.
    def prefixes_a_dispatched(name: str) -> bool:
        return any(
            other == name or other.startswith(name + " ") for other in required
        )

    accepted_only = sorted(
        name
        for name in (p.lstrip("&") for p in custom_prefix + custom_exact)
        if name and name not in required and not prefixes_a_dispatched(name)
    )
    if accepted_only:
        print()
        print(f"  note: accepted by the whitelist but no handler: "
              f"{', '.join(accepted_only)}")
        print("        (these answer `[BT] unknown command: '...'`)")

    print()
    if problems:
        print(f"FAIL: {len(problems)} bluetooth reachability problem(s)")
        for item in problems:
            print(f"  - {item}")
        return 1

    print(f"PASS: all {len(required)} dispatched commands are reachable over "
          f"bluetooth ({len(INTENTIONAL_CONSOLE_ONLY)} documented console-only "
          f"exception(s))")
    return 0


if __name__ == "__main__":
    sys.exit(main())
