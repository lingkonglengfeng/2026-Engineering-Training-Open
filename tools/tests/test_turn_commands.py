"""V234 转向 PID 调参与波形的源码级一致性检查（无需 CPU 仿真）。

这一组只做静态检查，目的是把"命令白名单、分发路径、解析分支、输出格式、版本标记"
四处必须同时成立的条件固定下来 —— 现场最常见的故障就是命令发出去没反应：

  * 新命令 &pid / &tstat / &ttrace 必须同时出现在 bt_button.c 的三张白名单里
    （IsCustomPrefix 保 token 存活、IsCustomToken 负责 flush 派发、IsArgCommand 保留空格）；
  * zdt_motor.c 必须在"蓝牙 token 路径"和"裸串口路径"都认领它们；
  * 打印格式必须给上位机可解析的 key=value 字段；
  * 启动标记与 arm ui 的 firmware 号必须同步。

运行：
    python -m unittest discover -s tools/tests -p test_turn_commands.py
"""
import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
BT = ROOT / "Core" / "Src" / "bt_button.c"
ZDT = ROOT / "Core" / "Src" / "zdt_motor.c"
PID = ROOT / "Core" / "Src" / "pid_storage.c"
ARM = ROOT / "Core" / "Src" / "arm_task.c"
CAL = ROOT / "Core" / "Src" / "arm_calibration.c"
CHASSIS = ROOT / "Core" / "Src" / "chassis_motor.c"
CHASSIS_H = ROOT / "Core" / "Inc" / "chassis_motor.h"


def read(path):
    return path.read_text(encoding="utf-8", errors="replace")


def function_body(src, name, definition_only=False):
    """取出函数体。

    注意：`ZDT_Turn_Command(event.text) {` 这类调用点也会被普通正则匹配到，
    所以对“调用点比定义点先出现”的函数必须用 definition_only=True 锁定定义形式。
    """
    if definition_only:
        pattern = rf"(?:static\s+)?(?:uint8_t|void|float|int)\s+{name}\s*\([^;{{]*?\)\s*\{{"
    else:
        pattern = rf"\b{name}\s*\([^;{{]*?\)\s*\{{"
    match = re.search(pattern, src)
    if match is None:
        raise AssertionError(f"cannot find {name}()")
    start = src.index("{", match.start() + len(name))
    depth = 0
    for idx in range(start, len(src)):
        if src[idx] == "{":
            depth += 1
        elif src[idx] == "}":
            depth -= 1
            if depth == 0:
                return src[start + 1:idx]
    raise AssertionError(f"unbalanced braces in {name}()")


class TurnCommandGates(unittest.TestCase):
    """命令能不能活着走到处理函数。"""

    @classmethod
    def setUpClass(cls):
        cls.bt = read(BT)

    def whitelist(self, fn):
        body = function_body(self.bt, fn, definition_only=True)
        return set(re.findall(r'"(&[^"]*)"', body))

    @staticmethod
    def accepted(token, literals):
        """与 bt_button.c 的 IsPrefixOf/StartsWith 同义：任一方是另一方的前缀。"""
        for literal in literals:
            if len(token) <= len(literal):
                if token == literal[:len(token)]:
                    return True
            elif token[:len(literal)] == literal:
                return True
        return False

    def test_every_new_command_survives_both_gates(self):
        survival = self.whitelist("BT_Button_IsCustomPrefix")
        dispatch = self.whitelist("BT_Button_IsCustomToken")
        for command in ("&pid", "&tstat", "&ttrace"):
            self.assertTrue(self.accepted(command, survival), f"{command} 未登记 IsCustomPrefix")
            self.assertTrue(self.accepted(command, dispatch), f"{command} 未登记 IsCustomToken")

    def test_ttrace_keeps_its_space_argument(self):
        spaces = self.whitelist("BT_Button_IsArgCommand")
        self.assertTrue(self.accepted("&ttrace 1", spaces), "&ttrace 参数会被空格切断")

    def test_bare_console_forms_are_not_claimed_by_tokenizer(self):
        # 裸串口 "tstat" 的每个字节都不是 '&'，tokenizer 必须放过它们；
        # 这里核对 tokenizer 的开头分支没被改写（与 check_uart_command_paths.py 同一前提）。
        body = function_body(self.bt, "BT_Button_ProcessByte", definition_only=True)
        self.assertIn("if (byte != (uint8_t)'&')", body)


class TurnCommandRouting(unittest.TestCase):
    """两条入口都必须认领新命令，避免"有线能用、蓝牙没反应"这类单侧遗漏。"""

    @classmethod
    def setUpClass(cls):
        cls.zdt = read(ZDT)
        cls.pid = read(PID)

    def test_turn_command_helper_is_forward_declared_and_defined(self):
        self.assertIn("static uint8_t ZDT_Turn_Command(const char *command);", self.zdt)
        body = function_body(self.zdt, "ZDT_Turn_Command", definition_only=True)
        self.assertIn('strcmp(command, "tstat")', body)
        self.assertIn('strncmp(command, "ttrace", 6U)', body)

    def test_bluetooth_and_console_paths_both_call_helper(self):
        bt_path = function_body(self.zdt, "ZDT_ProcessBluetoothButtons", definition_only=True)
        console_path = function_body(self.zdt, "ZDT_Process_Command", definition_only=True)
        self.assertIn("ZDT_Turn_Command(event.text)", bt_path)
        self.assertIn("ZDT_Turn_Command(command)", console_path)

    def test_pid_query_reads_ram_current_value(self):
        body = function_body(self.pid, "PID_Storage_ProcessCommand", definition_only=True)
        self.assertIn('strcmp(command, "pid")', body)
        self.assertIn("PID_PrintCurrent()", body)

    def test_pid_set_commands_still_work(self):
        # 新加的 &pid 不能吃掉 &kp/&ki/&kd
        body = function_body(self.pid, "PID_Storage_ProcessCommand", definition_only=True)
        for prefix in ('"kp"', '"ki"', '"kd"'):
            self.assertIn(f"strncmp(command, {prefix}, 2U)", body)


class TurnWaveOutput(unittest.TestCase):
    """波形与状态行的字段名是上位机解析契约，改名会静默打断曲线。"""

    @classmethod
    def setUpClass(cls):
        cls.zdt = read(ZDT)
        cls.body = function_body(cls.zdt, "ZDT_TurnLog_Service", definition_only=True)

    def test_wave_line_has_stable_fields(self):
        self.assertIn("[TURNW] t=%lu tgt=%+.2f act=%+.2f err=%+.3f out=%+.3f", self.body)

    def test_start_and_done_lines_present(self):
        self.assertIn("[TURN] start tgt=%+.2f act=%+.2f profile=%s", self.body)
        self.assertIn("[TURN] done %lums target=%+.2f", self.body)

    def test_trace_is_off_until_enabled(self):
        # 找静态定义；没有显式初值即 0（C 静态存储期），有初值也必须是 0。
        match = re.search(r"static\s+uint8_t\s+zdt_turn_trace_enabled\s*(=\s*([^;]+))?;", self.zdt)
        self.assertIsNotNone(match, "找不到 zdt_turn_trace_enabled 定义")
        if match.group(2):
            self.assertIn("0", match.group(2))

    def test_trace_period_is_documented_and_used(self):
        self.assertIn("#define ZDT_TURN_TRACE_PERIOD_MS (25U)", self.zdt)
        self.assertIn("ZDT_TURN_TRACE_PERIOD_MS", self.body)

    def test_status_line_carries_gains_and_imu(self):
        body = function_body(self.zdt, "ZDT_Turn_Command", definition_only=True)
        for key in ("enabled=%u", "active=%u", "imu=%u", "trace=%u", "rpm=%.0f",
                    "tgt=%+.2f", "act=%+.2f", "err=%+.3f", "out=%+.3f",
                    "kp=%.4f", "ki=%.4f", "kd=%.4f", "src=%s"):
            self.assertIn(key, body, f"&tstat 缺少字段 {key}")


class PidPersistence(unittest.TestCase):
    """V236：&save 过的 PID 必须上电自动装载，且能报告当前值来源。"""

    @classmethod
    def setUpClass(cls):
        cls.pid = read(PID)
        cls.zdt = read(ZDT)

    def test_boot_apply_is_declared_and_implemented(self):
        self.assertIn("PID_Storage_ApplyBoot", read(ROOT / "Core/Inc/pid_storage.h"))
        body = function_body(self.pid, "PID_Storage_ApplyBoot", definition_only=True)
        self.assertIn("PID_Storage_Load", body)
        self.assertIn("ChassisMotor_SetTurnGains", body)

    def test_boot_apply_called_once_from_motor_begin(self):
        begin = function_body(self.zdt, "ZDT_Motor_Begin", definition_only=True)
        self.assertEqual(begin.count("PID_Storage_ApplyBoot()"), 1,
                         "上电只在 ZDT_Motor_Begin 装载一次，避免 &EN 反复覆盖现场手改值")

    def test_source_is_tracked_and_reported(self):
        self.assertIn("PID_Storage_Source", self.pid)
        current = function_body(self.pid, "PID_PrintCurrent", definition_only=True)
        self.assertIn("source=%s", current)
        self.assertIn("PID_Storage_Source()", current)
        # &save 成功、&read 成功、手动 &kp 三条路径都要更新来源。
        process = function_body(self.pid, "PID_Storage_ProcessCommand", definition_only=True)
        self.assertGreaterEqual(process.count("s_pid_source"), 3)


class TurnSpeedLimit(unittest.TestCase):
    """V235：精确档轮速上限改为 2×（90→180），快速档保持 120 不动。"""

    @classmethod
    def setUpClass(cls):
        cls.header = read(CHASSIS_H)
        cls.chassis = read(CHASSIS)

    def define(self, name):
        match = re.search(rf"#define\s+{name}\s+\(([-+0-9.]+)f?\)", self.header)
        self.assertIsNotNone(match, f"找不到 {name} 定义")
        return float(match.group(1))

    def test_precise_turn_cap_is_doubled(self):
        self.assertEqual(self.define("CHASSIS_TURN_MAX_WHEEL_RPM"), 180.0)

    def test_fast_turn_cap_matches_precise(self):
        # V237：现场要求快速档与精确档一样快（导航段前对准不再被 120 RPM 卡住）。
        match = re.search(r"#define\s+CHASSIS_TURN_FAST_WHEEL_RPM\s+\(([^)]+)\)", self.header)
        self.assertIsNotNone(match, "找不到 CHASSIS_TURN_FAST_WHEEL_RPM 定义")
        token = match.group(1).strip()
        self.assertTrue(token == "CHASSIS_TURN_MAX_WHEEL_RPM" or
                        token.rstrip("fF") in ("180.0", "180"),
                        f"快速档上限没有跟精确档一致：{token}")
        precise = re.search(
            r"chassis_turn_profile_precise\s*=\s*\{\s*([^,\n]+)", self.chassis)
        fast = re.search(
            r"chassis_turn_profile_fast\s*=\s*\{\s*([^,\n]+)", self.chassis)
        self.assertIsNotNone(precise, "找不到 precise 档初始化")
        self.assertIsNotNone(fast, "找不到 fast 档初始化")
        # 两档首字段都是同一个轮速上限宏/值，避免只改一处造成“看起来一样、实际不一样”。
        self.assertIn("CHASSIS_TURN_MAX_WHEEL_RPM", precise.group(1))
        self.assertIn("CHASSIS_TURN_FAST_WHEEL_RPM", fast.group(1))

    def test_precise_profile_actually_uses_the_macro(self):
        # 结构体常量不是函数，直接检查初始化首字段用的是这个宏。
        match = re.search(
            r"chassis_turn_profile_precise\s*=\s*\{\s*([^,\n]+)", self.chassis)
        self.assertIsNotNone(match, "找不到 chassis_turn_profile_precise 初始化")
        self.assertIn("CHASSIS_TURN_MAX_WHEEL_RPM", match.group(1))

    def test_max_rpm_getter_is_exposed(self):
        self.assertIn("ChassisMotor_GetTurnMaxWheelRpm", self.header)
        body = function_body(self.chassis, "ChassisMotor_GetTurnMaxWheelRpm",
                             definition_only=True)
        # 必须固定报告精确档（手动 &L/&R 用），不能随最近一次导航快速档漂移。
        self.assertIn("chassis_turn_profile_precise.max_wheel_rpm", body)


class TurnTargetGetter(unittest.TestCase):
    """起转沿就要画目标阶跃，不能等控制周期把 telemetry 刷新一拍。"""

    def test_getter_declared_and_defined(self):
        self.assertIn("ChassisMotor_GetTurnTargetDeg", read(CHASSIS_H))
        body = function_body(read(CHASSIS), "ChassisMotor_GetTurnTargetDeg", definition_only=True)
        self.assertIn("chassis_turn_target_deg", body)
        self.assertIn("chassis_pid_target_deg", body)


class VersionMarkers(unittest.TestCase):
    def test_boot_marker_and_arm_ui_agree(self):
        marker = read(ARM)
        self.assertIn("CAMERA_DEPLOY_V245_", marker)
        self.assertNotIn("CAMERA_DEPLOY_V239_FINAL", marker)
        self.assertIn("firmware=245", read(CAL))
        self.assertNotIn("firmware=239", read(CAL))


if __name__ == "__main__":
    unittest.main()
