#ifndef __GRID_NAV_H__
#define __GRID_NAV_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ============================ 场地栅格模型 ============================
 * 2026-09-12 重构：由 8x8@300mm 改为 **48x48@50mm**，依据是规则图实测几何：
 *   场地 2400x2400 mm；中央十字通道净宽 400 mm（轴线 X=1200 / Y=1200）；
 *   四块黄色禁行区 450x450；外环带净宽 550；启停区 300x300（右上/右下角）；
 *   粗加工区/暂存区深 150；实车投影 280x280。
 * 场地尺寸都是 50 mm 的整数倍（2400/50=48、400/50=8、450/50=9、
 * 550/50=11、300/50=6、150/50=3），所以 50 mm 是唯一能把所有边界落在格线上的
 * 分辨率。旧 8x8@300 的问题：通道 400mm÷300 不是整数、轴心 1200 落在格边界上
 * （格心永远对不到轴线）、障碍边界跨格导致避障判定失真。
 *
 * ★ 坐标约定（**对外统一用规则图坐标系**）：
 *     原点 (0,0) = 场地**左下角**内边沿交点，x 向右、y **向上**，单位 mm。
 *     格 (cx,cy) 覆盖 x∈[cx*50,(cx+1)*50]、y∈[cy*50,(cy+1)*50]，中心 = cx*50+25。
 *   屏幕绘制的 y 向下翻转只在绘图层做（Dashboard_DrawMap）。
 *   ⚠ 旧固件是"原点左上、y 向下"，改坐标那次事故的根因就是两套口径混用，
 *     所以这里把"对外 mm/y 向上"写死成唯一口径。
 * ==================================================================== */
#define GRID_NAV_SIZE          (48U)
#define GRID_NAV_CELL_MM       (50U)
#define GRID_NAV_FIELD_MM      (2400U)
#define GRID_NAV_RAW_PAD_DEPTH_MM (85U) /* Actual RAW pad intrusion from top field edge. */
#define GRID_NAV_RAW_APPROACH_X_MM (1250U)
#define GRID_NAV_RAW_APPROACH_Y_MM (2100U)
#define GRID_NAV_QR_SCAN_X_MM (2125U)
#define GRID_NAV_QR_SCAN_Y_MM (1130U) /* V269:20mm toward start zone1. */
#define GRID_NAV_MAX_PATH      (GRID_NAV_SIZE * GRID_NAV_SIZE)   /* 2304 */
#define GRID_NAV_MAX_SEGMENTS  (GRID_NAV_MAX_PATH)
/* Empty runtime map: prefer a wider outer route within this extra distance. */
#define GRID_NAV_OUTER_EXTRA_MM (200U)
#define GRID_NAV_CENTER_EXTRA_MM (200U) /* Empty-map centre refinement, as before. */
#define GRID_NAV_CENTER_OBSTACLE_EXTRA_MM (600U) /* Room for both obstacle detour entry/exit joins. */

/* 实车280x280 mm；通行检查按毫米外廓覆盖到的全部格子判定。 */
#define GRID_NAV_CAR_MM        (280U)
#ifndef GRID_NAV_YELLOW_MARGIN_MM
#define GRID_NAV_YELLOW_MARGIN_MM (20U) /* Extra planning clearance around the four internal yellow blocks. */
#endif
#define GRID_NAV_CAR_HALF_CELLS (((GRID_NAV_CAR_MM / 2U) + GRID_NAV_CELL_MM - 1U) / GRID_NAV_CELL_MM) /* ceil(140/50)=3 */

/* 场地几何（mm，均为规则图实测值；要按实物微调就改这一段） */
#define GRID_NAV_CROSS_HALF    (200U)   /* 十字通道半宽：400/2 */
#define GRID_NAV_BAND_MM       (550U)   /* 外环带净宽 */
#define GRID_NAV_BLOCK_LO      (550U)   /* 黄区起点：550 */
#define GRID_NAV_BLOCK_HI      (1850U)  /* 黄区终点：1850 = 2400-550 */
#define GRID_NAV_LANE_MID_MM   (1200U)  /* 十字轴线 */
#define GRID_NAV_LANE_FAR_MM   (2125U)  /* 外环带中心线 = 1850 + 550/2 */
#define GRID_NAV_LANE_NEAR_MM  (275U)   /* 外环带中心线 = 550/2 */

/* 格子类型（同时决定屏幕配色与可通行性） */
typedef enum
{
  GRID_CELL_ROAD = 0U,     /* 中央十字通道 */
  GRID_CELL_RING = 1U,     /* 外环带 */
  GRID_CELL_BLOCKED = 2U,  /* 四块 450x450 黄区：不可通行 */
  GRID_CELL_START = 3U,    /* 启停区（300x300，可通行） */
  GRID_CELL_WORK = 4U,     /* 边缘工位的**功能区**（含停位空间，可通行；只影响配色/语义） */
  /* 边缘工位摆的**实物台面**（原料台/粗加工台/暂存台/二维码板）：车体**不得接触**。
   * 与 WORK 的区别很关键：WORK 是整个功能区的语义覆盖层（含车可以站的停位空间，
   * 必须保持可通行），PAD 是那台设备真正占的地面（150 mm 深；二维码板 50 mm 宽）——
   * 车体压上去就是"接触工位"，比赛要扣分。所以 PAD 与 BLOCKED 一样参与车体膨胀：
   * 车心轴对齐时须留出至少140 mm半车宽；旋转时按实际姿态判断。 */
  GRID_CELL_PAD = 5U
} grid_cell_type_t;

/* 网格坐标，编号范围 0..47。 */
typedef struct
{
  uint8_t x;
  uint8_t y;
} grid_nav_point_t;

/* 连续同方向的一整段路径。length 单位是"格"。 */
typedef struct
{
  uint8_t from_x;
  uint8_t from_y;
  uint8_t to_x;
  uint8_t to_y;
  int8_t dx;
  int8_t dy;
  uint16_t length;
} grid_nav_segment_t;

/* 命名点位（mm，规则图坐标系）。名字在 BT/串口里用 &goto raw 这种形式。 */
typedef struct
{
  const char *name;
  uint16_t x;
  uint16_t y;
} grid_nav_waypoint_t;

void GridNav_Init(void);
void GridNav_PrintMap(void);
uint8_t GridNav_ProcessCommand(const char *command);

/* 规划（格坐标入口，供旧命令与内部使用）。 */
uint8_t GridNav_Plan(uint8_t start_x, uint8_t start_y,
                     uint8_t target_x, uint8_t target_y,
                     grid_nav_point_t *path, uint16_t *path_length);
/* 规划（mm 入口，推荐）。返回的路径是格序列，段长以格计。 */
uint8_t GridNav_PlanMm(uint16_t start_x, uint16_t start_y,
                       uint16_t target_x, uint16_t target_y,
                       grid_nav_point_t *path, uint16_t *path_length);

/* 当前小车所在格（显示用）与对外 mm 位姿。 */
void GridNav_SetPose(uint8_t x, uint8_t y);
void GridNav_GetPose(uint8_t *x, uint8_t *y);
void GridNav_SetPoseMm(uint16_t x_mm, uint16_t y_mm);
void GridNav_GetPoseMm(uint16_t *x_mm, uint16_t *y_mm);

/* 障碍物（按格设置；随机障碍就用它加） */
void GridNav_SetObstacle(uint8_t x, uint8_t y, uint8_t blocked);
/* Add a measured footprint without replacing other runtime obstacles. */
uint8_t GridNav_AddObstacleDiscMm(uint16_t x,uint16_t y,uint16_t radius);
/* Half-open rectangle in world mm; runtime layer, fixed geometry unchanged. */
uint16_t GridNav_AddObstacleRectMm(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1);
/* Navigation owner only: validate all millimetre points BEFORE replacing the
 * runtime map; rebuild once, invalidate the old plan, keep static geometry. */
uint8_t GridNav_ReplaceObstaclesMm(const uint16_t *x,const uint16_t *y,uint8_t count);
uint8_t GridNav_IsObstacle(uint8_t x, uint8_t y);
uint8_t GridNav_HasObstacles(void);
#define GRID_NAV_MANUAL_OBSTACLE_MAX (2U)
#define GRID_NAV_MANUAL_OBSTACLE_RADIUS_MM (50U)
uint8_t GridNav_ManualObstacleMode(void);
uint8_t GridNav_ManualObstacleCount(void);
uint8_t GridNav_SetManualObstaclesMm(const uint16_t *x,const uint16_t *y,uint8_t count);
/* New-round boundary: clear runtime obstacle layer, preserving field, pose and station offsets. */
void GridNav_ClearRuntimeObstacles(void);
/* Two owned vision layers; removing one preserves manual/overlapping cells. */
uint16_t GridNav_SetVisionObstacleRectMm(uint8_t slot,uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1);
uint16_t GridNav_SetVisionObstacleDiscMm(uint8_t slot,uint16_t x,uint16_t y,uint16_t radius);
uint16_t GridNav_ClearVisionObstacle(uint8_t slot);
uint32_t GridNav_GetRevision(void);

/* 场地几何 → 格/ mm 换算与"车道吸附"。
   ⚠ 对内格子行号 cy 向下递增（cy=0 = 场地最下边一行），这样执行器的
     "dy>0 = 前进"方向链不用改；x 轴无翻转。所有 mm 参数都是规则图坐标系。 */
uint8_t GridNav_MmToCellX(uint16_t mm);
uint8_t GridNav_MmToCellY(uint16_t mm_up);
uint16_t GridNav_CellCenterMmX(uint8_t cx);
uint16_t GridNav_CellCenterMmY(uint8_t cy);
uint16_t GridNav_LaneMm(uint16_t mm);
uint8_t GridNav_CellType(uint8_t x, uint8_t y);
uint8_t GridNav_IsWalkable(uint8_t x, uint8_t y);

/* 命名点位查询：返回 1 表示找到并写入 x/y（mm）。 */
uint8_t GridNav_FindWaypoint(const char *name, uint16_t *x_mm, uint16_t *y_mm);
/* Typed planning entry points for application state machines. */
uint8_t GridNav_PlanTargetMm(uint16_t x_mm, uint16_t y_mm);
/* Prefer an axis-aligned transport at actual mm pose, including partial cells.
 * Validates the real 280mm body/obstacles along the line. Returns 0 without
 * storing a plan if diagonal or blocked; caller may use A*. */
uint8_t GridNav_PlanStraightTargetMm(uint16_t x_mm, uint16_t y_mm);
/* Post-alignment return to a lane: shortest transverse component first (<=150
 * mm), followed by the main transport leg. Validate both real-mm body paths
 * and corner rotation; retain exact lengths without cell snapping. */
uint8_t GridNav_PlanRecenterTargetMm(uint16_t x_mm,uint16_t y_mm);
/* Heading-aware: a short strafe/BACK retains heading, so do not demand a
 * fictitious 90-degree sweep at the lane join. */
uint8_t GridNav_PlanRecenterTargetMmHeading(uint16_t x_mm,uint16_t y_mm,float world_yaw);
/* A* detour, with actual-mm translation and executed corner-turn checks. */
uint8_t GridNav_PlanCheckedTargetMmHeading(uint16_t x_mm,uint16_t y_mm,float world_yaw);
/* Executable distance: straight plans retain exact mm; A* uses cells*50. */
uint16_t GridNav_GetSegmentDistanceMm(uint16_t segment_index);
uint8_t GridNav_PlanWaypoint(const char *name);

/* 取最近一次规划结果，供显示层和运动执行层使用。 */
uint8_t GridNav_GetPlan(grid_nav_point_t *path, uint16_t capacity,
                        uint16_t *path_length, uint8_t *target_x,
                        uint8_t *target_y);
uint8_t GridNav_GetSegments(grid_nav_segment_t *segments, uint16_t capacity,
                            uint16_t *segment_count);
/* 规划目标点的 mm 坐标（给执行器用）。 */
void GridNav_GetTargetMm(uint16_t *x_mm, uint16_t *y_mm);

/* 圆弧过弯（`&tarc`）的几何闸门：**可旋转车体**在 mm 位姿 (x_mm, y_mm)、车头朝向
 * yaw_deg（地图角度：0 = +x，逆时针为正）时，是否整块都在安全区内。
 * 返回 1 = 安全。判定是精确矩形（分离轴），包括：场边、四块 450×450 黄区、
 * 四个工位实物台面（PAD）、运行期障碍格（`&obs`）。
 *
 * 与轴对齐车体膨胀表的关系：grid_nav_walkable 判的是"**轴对齐**车体能不能放在格心"，
 * 而圆弧过弯时车体在转 90°，扫过的外接圆半径约198 mm —— 所以这里必须按真实姿态算，
 * 不能查膨胀表。 */
uint8_t GridNav_RotatedBodyFreeMm(uint16_t x_mm, uint16_t y_mm, int16_t yaw_deg);
/* Experimental mixed motion: expand each local body side by margin_mm. */
uint8_t GridNav_RotatedBodyFreeMarginMm(uint16_t x_mm, uint16_t y_mm,
                                       int16_t yaw_deg, uint16_t margin_mm);

/* 启停区**可通行**（2026-09-13 现场更正"启停区可以经过"）。
 *
 * 这里原来有 `GridNav_SetZoneNoTouch()` —— `&leave`/`&cross` 结束时把**刚离开**的那个区
 * 锁成"车体不可接触"，`&startpose N` 才解锁。现在整段删除，原因：
 *   · 现场确认两个启停区就是地面上的两块区域，**车可以照常压过去**；
 *   · 锁着的时候 A* 必须绕开区（顶角那一段路会被迫走远路），
 *     圆弧过弯也会在区口被判"几何不允许"而退回原地转。
 * 仍在的约束只有物理那条：**离场边/黄区/工位台面的距离**（见 GridNav_CellPassable
 * 的毫米外廓膨胀与 GridNav_RotatedBodyFreeMm 的精确判定）。 */

/* A* 的单格代价单位（"格数当量"）。执行器折算"转向耗时相当于走几格"时也用它当单位，
   所以放在头文件里作为唯一来源。 */
#define GRID_NAV_MOVE_COST        (10U)

/* Turn-time preference follows distance (and bounded outer-band preference
 * on an empty runtime map). Raw-cell lane centring is the final tie-break. Executable-mm plans may
 * refine outer centring within200mm (600mm with obstacles), without increasing inner exposure. */
void GridNav_SetCostModel(uint16_t turn90_units, uint16_t turn180_units);
/* Initial heading also preflights optional centring joins/turns; never moves or rebases the car. */
void GridNav_SetInitialHeading(float yaw_deg);

#ifdef __cplusplus
}
#endif

#endif /* __GRID_NAV_H__ */
