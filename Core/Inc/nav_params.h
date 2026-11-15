#ifndef NAV_PARAMS_H
#define NAV_PARAMS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* V174 trial: chassis translation targets only; encoder units stay physical. */
#define NAV_STRAIGHT_TARGET_SCALE (1.03f)
float Nav_Params_StraightDistance(float nominal_mm);
/* Initial-race motion tuning; private W25 banks, legacy NAV layout unchanged. */
typedef struct {
    float arc_mm_s,accel_rpm_s,decel_rpm_s,turn_decel_rpm_s;
    float straight_scale,field_wheel_rpm,arc_yaw_dps;
    float heading_kp,heading_cap,heading_deadband,turn_band_deg,turn_stable_ms;
} nav_motion_parameters_t;
nav_motion_parameters_t *Nav_Motion(void);
uint8_t Nav_Motion_ProcessCommand(const char *command);
/* 1=loaded,0=no usable record/read error,2=SPI storage currently busy. */
uint8_t Nav_Motion_LoadFromFlash(void);

/* 导航行进参数。
 *
 *   speed_scale     速度：轮速(RPM) = speed_scale x CHASSIS_MAX_WHEEL_RPM(250)
 *   time_1000mm_ms  "spd=0.50 时走 1000 mm 需要多少毫秒"——注意自 v2.2x 起距离
 *                   不再按"距离 x t1000"线性换算：zdt_motor.c 用梯形/三角形
 *                   运动学解出驱动时间，t1000 只作为整体时间微调系数
 *                       k = t1000 / 理论时间(1000 mm @ spd 0.50)
 *                       t_ms = plan_seconds(L * (1+dcomp), v) / plan_seconds(1000, v_ref) x t1000
 *                   好处：同一个 t1000 在任意段长、任意 spd 下都成立，改 `spd`
 *                   不再需要重标 t1000。理论值见 `nav` 打印的"理论xxxxms"。
 *   turn90_ms       直角（90°）转向的**实际耗时**，用于 A* 的代价模型 —— 规划器用
 *   turn180_ms      它换算"转一次相当于走几格"，所以这两个值直接决定"绕路 vs 掉头"
 *                   的取舍。现场实测方法：连发几次 `&L90` / `&L180`，读日志里
 *                   `[TURN] done <ms> …` 的毫秒数取平均（那个数已含斜坡与停稳）。
 *   arc90_ms        90° **圆弧过弯**（不停车、边走边转）的目标耗时，毫秒。
 *                   0 = 关闭（默认，行为与以前完全一样）。它是**半径的旋钮**：
 *                   R = 2 * v_arc * T / pi，T 越小半径越小（转得越急）。
 *                   实际半径还会被两件事夹住，都会打在日志里：
 *                     ① 几何：车体旋转时扫过的是 约198 mm半径的外接圆，转角附近必须
 *                        留够（`GridNav_ArcBodyFreeMm()` 逐位姿精确判定）；
 *                     ② 轮速：外轮需要 v + 300*(v/R)，上限 250 RPM；超了就按比例
 *                        把圆弧整体降速（半径不变、时间变长）。
 *   arc_spd_ratio   圆弧时的车体速度 = 当前 `spd` × 这个比例（默认 1.00 = 与直走同速）。
 *                   现场若发现圆弧阶段航向跟不住（陀螺残差大/打滑），先降它。
 *
 * 参数都能在串口或蓝牙上改：`spd<值>` / `t1000<值>` / `cnt<值>` / `t90<值>` /
 * `t180<值>` / `tarc<值>` / `aspd<值>` / `dcomp<值>`，改完 `save` 存进 W25Q64，`read` 读回来。
 * `nav` 打印当前值（含 k 与理论值）。 */
typedef struct
{
    float speed_scale;      /* `spd` 轮速比例 */
    float time_1000mm_ms;   /* `t1000` spd=0.50 下 1000 mm 的等效时间（毫秒） */
    /* 里程回读刻度：`cnt` 每圈多少计数（0x36 位置的单位）。
     * 厂商手册 Emm 口径 65536；实测若不是这个值（用 `&dis` 关使能后手转一圈、
     * `&posref`/`&posd` 量出来），在这里改。 */
    float counts_per_rev;
    uint8_t position_mode;  /* `posmode 1` 时用 0xFD 位置模式（默认 0） */
    float turn90_ms;        /* `t90`  直角转向实际耗时（毫秒），给 A* 代价模型用 */
    float turn180_ms;       /* `t180` 掉头实际耗时（毫秒），给 A* 代价模型用 */
    float arc90_ms;         /* `tarc` 90° 圆弧过弯目标耗时（毫秒），0 = 关闭 */
    float arc_spd_ratio;    /* `aspd` 圆弧车体速度 = spd × 本比例 */
    float distance_compensation; /* `dcomp` 速度/时间平移距离增量比例，0.10 = 多算10% */
} nav_parameters_t;

/* 运行中的参数实例（带默认值，可被 `spd`/`t1000` 就地修改）。 */
nav_parameters_t *Nav_Params(void);

/* 只供速度/时间平移规划调用一次；地图目标、编码器目标和转角保持原值。 */
float Nav_Params_CompensatedDistance(float nominal_mm);

/* 恢复默认值。 */
void Nav_Params_Reset(void);

/* 处理 `nav` / `spd<值>` / `t1000<值>` / `cnt<值>` / `t90<值>` / `t180<值>` /
 * `tarc<值>` / `aspd<值>` / `dcomp<值>` / `navreset`；属于本模块返回 1。 */
uint8_t Nav_Params_ProcessCommand(const char *command);

/* 打印当前参数。 */
void Nav_Params_Print(void);

/* 初始化时上报参考条件（spd=0.50、1000mm）的理论时间，仅用于k计算。
 * 当前运行/重置默认为spd=0.650、t1000=1930ms，理论值不会覆盖现场标定值。
 * 参考速度与运行速度分别定义；调运行速度不能一起改动标定基准。 */
void Nav_Params_SetReferencePlanMs(float reference_ms);

/* 存取到 W25Q64（独立扇区，与 PID 那份互不影响）。返回 1 表示成功。 */
uint8_t Nav_Params_SaveToFlash(void);
uint8_t Nav_Params_LoadFromFlash(void);

#ifdef __cplusplus
}
#endif

#endif /* NAV_PARAMS_H */
