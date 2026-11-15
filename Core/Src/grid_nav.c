#include "grid_nav.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "zdt_motor.h"

#define GRID_NAV_NODE_COUNT       (GRID_NAV_SIZE * GRID_NAV_SIZE)
/* GRID_NAV_MOVE_COST 定义在 grid_nav.h（执行器折算转向代价时也要用它当单位）。 */
/* 转向代价默认值（单位 = "格数当量"，move 恒为一格）。执行器会在每次规划前用
 * GridNav_SetCostModel() 按**当前 spd** 把它换成"相当于走几格"：
 *   turn90_units = MOVE_COST × (90° 转向耗时 / 走一格耗时)
 * 这样 A* 直接就在比较"转向时间 + 行驶时间"。慢速时走一格的时间被拉长 → 转向相对便宜；
 * 快速时转向相对更贵 —— 与现场直觉一致。下面这两个只是"还没被刷新"时的兜底值。
 * 代价始终 ≥ 0，曼哈顿启发函数依旧可采纳（不会高估）。 */
#define GRID_NAV_TURN_COST        (120U)
#define GRID_NAV_LANE_COST_PER_CELL  (3U)

static uint16_t grid_nav_turn90_cost = GRID_NAV_TURN_COST;
static uint16_t grid_nav_turn180_cost = (uint16_t)(GRID_NAV_TURN_COST * 2U);
#define GRID_NAV_INVALID_INDEX    (0xFFFFU)

/* A* 搜索节点。48x48=2304 个节点，所以索引/父节点必须是 uint16。
   为省 RAM 用紧凑布局：x,y 各 1B，parent/g/f 各 2B，opened/closed 各 1B。 */
typedef struct
{
  uint8_t x;
  uint8_t y;
  uint8_t opened;
  uint8_t closed;
  uint8_t in_dir;      /* 到达该格时用的方向（0..3），0xFF = 起点，用于算转向代价 */
  uint16_t parent;
  uint16_t g;
  uint16_t f;
} grid_nav_node_t;

/* 四邻域搜索顺序：上(+y)、右(+x)、下(-y)、左(-x)。
   注意 y 在本工程**向上**为正（规则图坐标系）。 */
static const int8_t grid_nav_dx[4] = {0, 1, 0, -1};
static const int8_t grid_nav_dy[4] = {1, 0, -1, 0};

/* 格子类型与"考虑车体膨胀后的可通行性"。两者都要 2304 B。 */
static uint8_t grid_nav_cell_kind[GRID_NAV_SIZE][GRID_NAV_SIZE];
static uint8_t grid_nav_walkable[GRID_NAV_SIZE][GRID_NAV_SIZE];
/* 运行期随机障碍（&obs/&free）：独立于场地固有禁区，清障时只清这一层。 */
static uint8_t grid_nav_obstacle[GRID_NAV_SIZE][GRID_NAV_SIZE];

static uint16_t grid_nav_pose_mm_x;
static uint16_t grid_nav_pose_mm_y;
static uint8_t grid_nav_pose_x;
static uint8_t grid_nav_pose_y;

static grid_nav_node_t grid_nav_nodes[GRID_NAV_NODE_COUNT];
static uint16_t grid_nav_open[GRID_NAV_NODE_COUNT];
static uint16_t grid_nav_open_count;

/* 规划结果的暂存区：**必须 static**。48x48 下按局部变量放栈上要 4.6 KB，
   FreeRTOS 任务栈放不下（原来 8x8 只有 128 B 才敢放栈上）。 */
static grid_nav_point_t grid_nav_scratch_path[GRID_NAV_MAX_PATH];
static uint16_t grid_nav_scratch_reversed[GRID_NAV_MAX_PATH];
static grid_nav_point_t grid_nav_last_path[GRID_NAV_MAX_PATH];
static uint16_t grid_nav_last_path_length;
static uint8_t grid_nav_last_target_x;
static uint8_t grid_nav_last_target_y;
static uint16_t grid_nav_last_target_mm_x;
static uint16_t grid_nav_last_target_mm_y;
static grid_nav_segment_t grid_nav_last_segments[GRID_NAV_MAX_SEGMENTS];
static uint16_t grid_nav_last_segment_count;
static uint16_t grid_nav_exact_distance_mm[GRID_NAV_MAX_SEGMENTS];
static uint8_t grid_nav_has_plan;
static uint32_t grid_nav_revision;
/* 启停区**可通行**（2026-09-13 现场更正"启停区可以经过"）：这里原来有
 * `grid_nav_zone_no_touch`（出库后把刚离开的区锁成"车体不可接触"），已整段删除 ——
 * 锁会让 A* 绕开区、也会让圆弧过弯在区口被拒，而现在两个区跟普通地面一样可以走。 */

/* ---------- 场地禁区的**精确 mm 矩形**（{x0, x1, y0, y1}） ----------
 * 建图（GridNav_BuildFieldMap）与"可旋转车体的碰撞判定"（GridNav_RotatedBodyFreeMm）
 * 共用同一份数据，避免两处漂移。四块黄区由平铺恒等式 550+450+400+450+550=2400 推出，
 * 中间 400 宽的十字通道（1000..1400）**在它们之间**，所以必须分成 4 块、不能合并。 */
static const int16_t grid_nav_zone_block_rects[4][4] =
{
  {    550,   1000,    550,   1000},
  {   1400,   1850,    550,   1000},
  {    550,   1000,   1400,   1850},
  {   1400,   1850,   1400,   1850}
};
/* 工位**实物台面**：车体不得接触（GRID_CELL_PAD，现场判罚项）。 */
static const int16_t grid_nav_pad_rects[4][4] =
{
  {1000, 1400, GRID_NAV_FIELD_MM-GRID_NAV_RAW_PAD_DEPTH_MM, GRID_NAV_FIELD_MM}, /* RAW最大入场85mm */
  {1000, 1400,    0,  150},   /* 粗加工台（底边中央，150 深） */
  {   0,  150, 1050, 1350},   /* 暂存台（左边中部，150 宽） */
  {2350, 2400, 1100, 1300}    /* 二维码板（右边中部，50 宽） */
};

/* 命名点位（mm，规则图坐标系）。现场改点位只改这张表。
 * 依据：
 *   原料区（上方居中，车头 +y 顶到 y=2400）→ 停车中心 (1200, 2250)
 *   粗加工区（下方居中，深 150）→ 车头 -y，前缘顶到 y=150 → 中心 y=300
 *   暂存区（左侧居中，宽 150，3 孔跨距 300）→ 车头 -x，前缘顶到 x=150 → 中心 x=300
 *   二维码板（右侧居中）→ 车头 +x，前缘顶到 x=2400 → 中心 x=2250
 *   启停区 300x300：右上 (2250,2250)、右下 (2250,150)（区中心）
 *   出场让位点 = 外环带中心线 x=2125 与 启停区前沿 y=1950 / 450 */
/* 命名点位（mm，规则图坐标）。
 *
 * ⚠ 工位点（raw/rough/temp/qr）是"**接近点**"，不是台面中心：台面本身是 PAD
 * （车体不得接触），所以这些点落在台面前方那条自由车道的中线上，留着最大余量：
 *   原料台 (y 2250..2400) 的前方自由窗口 = y 2000..2100 → 取 **2050**（前后各 50 mm）
 *   粗加工台 (y 0..150)    的上方自由窗口 = y 300..450   → 取 **350**
 *   暂存台 (x 0..150)      的右侧自由窗口 = x 300..400   → 取 **350**
 *   二维码板 (x 2350..2400) 的左侧自由窗口 = x 2000..2200 → 取 **2125**（与出场车道一致）
 * 这样即使现场把台面实际尺寸做大了 50 mm，车体也不会压上去。 */
static const grid_nav_waypoint_t grid_nav_waypoints[] =
{
  { "center", 1200U, 1200U },   /* 十字交叉口中心 */
  { "raw", GRID_NAV_RAW_APPROACH_X_MM, GRID_NAV_RAW_APPROACH_Y_MM }, /* 同正常流程RAW接近点 */
  { "rough",  1200U,  350U },   /* 粗加工台接近点（中间孔） */
  { "rough1", 1050U,  350U },   /* 粗加工台接近点 3 孔：x=1050/1200/1350 */
  { "rough2", 1200U,  350U },
  { "rough3", 1350U,  350U },
  { "temp",    350U, 1200U },   /* 暂存台接近点（中间孔） */
  { "temp1",   350U, 1350U },   /* 暂存台接近点 3 孔：y=1350/1200/1050（自上而下 1/2/3） */
  { "temp2",   350U, 1200U },
  { "temp3",   350U, 1050U },
  { "qr", GRID_NAV_QR_SCAN_X_MM, GRID_NAV_QR_SCAN_Y_MM },
  { "h1",     2250U,  150U },   /* 启停区1 = **右下角**区中心（官方图两区之一） */
  { "h2",     2250U, 2250U },   /* 启停区2 = **右上角**区中心（**本队实际发车区**，见 ZDT_START_ZONE_DEFAULT） */
  { "exit1",  2125U,  450U },   /* 启停区1 出场落点：右边缘环带中心线 x=2125 */
  { "exit2",  2125U, 1950U }    /* 启停区2 出场落点：右边缘环带中心线 x=2125 */
};

static void GridNav_Print(const char *text)
{
  ZDT_Console_Print(text);
}

static uint16_t GridNav_Index(uint8_t x, uint8_t y)
{
  return (uint16_t)(((uint16_t)y * GRID_NAV_SIZE) + x);
}

/* 曼哈顿距离作为 A* 启发函数，四邻域移动时不会高估代价。 */
static uint16_t GridNav_Heuristic(uint8_t x, uint8_t y,
                                  uint8_t target_x, uint8_t target_y)
{
  uint16_t dx = (x > target_x) ? (uint16_t)(x - target_x)
                               : (uint16_t)(target_x - x);
  uint16_t dy = (y > target_y) ? (uint16_t)(y - target_y)
                               : (uint16_t)(target_y - y);
  return (uint16_t)((dx + dy) * GRID_NAV_MOVE_COST);
}

/* ★坐标翻转换算（**全工程唯一**的 y 轴翻转点）：
 *   对外/规则图：mm，原点左下角，y 向上。
 *   对内/格子与执行器：行号 cy 向下递增（cy=0 是场地最下面一行），
 *   这样 A* 段的 dy>0 仍然等于"朝场地内/向下走"，与现场已验证的
 *   方向链（ZDT_Nav_GetBodyVelocity / SegmentHeadingDeg）**完全兼容**。
 *   cy = (SIZE-1) - floor(y_up/50)；  y_up = (SIZE-1-cy)*50 + 25 */
uint8_t GridNav_MmToCellX(uint16_t mm)
{
  uint16_t cell = (uint16_t)(mm / GRID_NAV_CELL_MM);

  if (cell >= GRID_NAV_SIZE)
  {
    cell = GRID_NAV_SIZE - 1U;
  }
  return (uint8_t)cell;
}

uint8_t GridNav_MmToCellY(uint16_t mm_up)
{
  uint16_t layer = (uint16_t)(mm_up / GRID_NAV_CELL_MM);

  if (layer >= GRID_NAV_SIZE)
  {
    layer = GRID_NAV_SIZE - 1U;
  }
  return (uint8_t)((GRID_NAV_SIZE - 1U) - layer);
}

uint16_t GridNav_CellCenterMmX(uint8_t cx)
{
  return (uint16_t)(((uint16_t)cx * GRID_NAV_CELL_MM) + (GRID_NAV_CELL_MM / 2U));
}

uint16_t GridNav_CellCenterMmY(uint8_t cy)
{
  return (uint16_t)(((GRID_NAV_SIZE - 1U - (uint16_t)cy) * GRID_NAV_CELL_MM) +
                    (GRID_NAV_CELL_MM / 2U));
}

/* 车道吸附：把坐标吸到最近的"车道中心线"上（十字 1200 / 外环 2125 与 275）。
   执行器用它保证车走在车道中线，而不是走 A* 随便挑的那一格的格心。 */
uint16_t GridNav_LaneMm(uint16_t mm)
{
  static const uint16_t lanes[3] = { GRID_NAV_LANE_NEAR_MM,
                                     GRID_NAV_LANE_MID_MM,
                                     GRID_NAV_LANE_FAR_MM };
  uint16_t best = lanes[0];
  uint16_t best_dist = 0xFFFFU;
  uint8_t i;

  for (i = 0U; i < 3U; ++i)
  {
    uint16_t dist = (mm > lanes[i]) ? (uint16_t)(mm - lanes[i])
                                    : (uint16_t)(lanes[i] - mm);
    if (dist < best_dist)
    {
      best_dist = dist;
      best = lanes[i];
    }
  }
  return best;
}

/* 可通行矩阵的重算入口（&obs/&free 与初始化用）。 */
static void GridNav_RebuildWalkable(void);

/* ============ 场地生成：几何 → 格子类型 → 车体膨胀 → 可通行表 ============
 * 分区依据（都能被 50 整除）：
 *   十字通道：x∈[1000,1400] 或 y∈[1000,1400]        （400 净宽）
 *   外环带  ：x<550 或 x≥1850 或 y<550 或 y≥1850    （550 净宽）
 *   黄区   ：x∈[550,1000)∪[1400,1850) 且 y 同          （450×450，四个）
 *   启停区 ：右上角 / 右下角 300×300
 * 三者合起来正好铺满整个 2400×2400：550+450+400+450+550 = 2400 ✓ */
static uint8_t GridNav_InRange(uint16_t v, uint16_t lo, uint16_t hi)
{
  return (uint8_t)((v >= lo) && (v < hi));
}

static void GridNav_BuildFieldMap(void)
{
  uint8_t cx;
  uint8_t cy;

  memset(grid_nav_cell_kind, 0, sizeof(grid_nav_cell_kind));

  for (cy = 0U; cy < GRID_NAV_SIZE; ++cy)
  {
    for (cx = 0U; cx < GRID_NAV_SIZE; ++cx)
    {
      uint16_t x = GridNav_CellCenterMmX(cx);
      uint16_t y = GridNav_CellCenterMmY(cy);
      uint8_t cross = (uint8_t)(GridNav_InRange(x, 1000U, 1400U) ||
                                GridNav_InRange(y, 1000U, 1400U));
      uint8_t ring = (uint8_t)((x < GRID_NAV_BLOCK_LO) ||
                               (x >= GRID_NAV_BLOCK_HI) ||
                               (y < GRID_NAV_BLOCK_LO) ||
                               (y >= GRID_NAV_BLOCK_HI));
      uint8_t kind;

      if (cross != 0U)
      {
        kind = GRID_CELL_ROAD;
      }
      else if (ring != 0U)
      {
        kind = GRID_CELL_RING;
      }
      else
      {
        kind = GRID_CELL_BLOCKED;   /* 四块 450x450 黄区 */
      }

      /* 功能区覆盖层（仍然是可通行的，只影响屏幕配色与语义） */
      if ((GridNav_InRange(x, 2100U, 2400U) &&
           GridNav_InRange(y, 2100U, 2400U)) ||
          (GridNav_InRange(x, 2100U, 2400U) &&
           GridNav_InRange(y, 0U, 300U)))
      {
        kind = GRID_CELL_START;     /* 启停区 1 / 2 */
      }
      else if (kind != GRID_CELL_BLOCKED)
      {
        if (GridNav_InRange(x, 1000U, 1400U) &&
            GridNav_InRange(y, 1850U, 2400U))
        {
          kind = GRID_CELL_WORK;    /* 原料区 */
        }
        else if (GridNav_InRange(x, 1000U, 1400U) &&
                 GridNav_InRange(y, 0U, 300U))
        {
          kind = GRID_CELL_WORK;    /* 粗加工区 + 停位 */
        }
        else if (GridNav_InRange(x, 0U, 300U) &&
                 GridNav_InRange(y, 850U, 1550U))
        {
          kind = GRID_CELL_WORK;    /* 暂存区 + 停位 */
        }
        else if (GridNav_InRange(x, 2100U, 2400U) &&
                 GridNav_InRange(y, 1000U, 1400U))
        {
          kind = GRID_CELL_WORK;    /* 二维码板 */
        }
      }

      /* 工位实物台面：车体不得接触（见 grid_nav.h 的 GRID_CELL_PAD 说明）。
          坐标就是屏幕上那四个青色块，全部对齐到 50 mm 栅格；
          矩形表 grid_nav_pad_rects 与圆弧过弯的碰撞判定共用。 */
      if ((kind != GRID_CELL_START) && (kind != GRID_CELL_BLOCKED))
      {
        uint8_t pad = 0U;
        uint8_t r;

        for (r = 0U; r < 4U; ++r)
        {
          if (GridNav_InRange(x, (uint16_t)grid_nav_pad_rects[r][0],
                              (uint16_t)grid_nav_pad_rects[r][1]) &&
              GridNav_InRange(y, (uint16_t)grid_nav_pad_rects[r][2],
                              (uint16_t)grid_nav_pad_rects[r][3]))
          {
            pad = 1U;
            break;
          }
        }

        if (pad != 0U)
        {
          kind = GRID_CELL_PAD;
        }
      }

      grid_nav_cell_kind[cy][cx] = kind;
    }
  }
}

/* 车体膨胀：格 (cx,cy) 的格心是 (cx+0.5)*50，车 300 宽 → 占 [c-150, c+150]。
   只有这个区间覆盖到的所有格都不是禁区（且都在场地内），才算"车能过"。
   400 宽的十字通道膨胀后只剩 2 格（格心 1175/1225）可走 → 通道仍然连通；
   这 2 格的做法保证车落在 ±25 mm 内，剩余误差由执行器的车道吸附消化。 */
/* ARMCC V5 对"非 const 二维数组 → const 二维数组参数"会告警，
   这里直接用非 const 参数（函数不改内容，靠注释约束）。 */
static uint8_t GridNav_CellPassable(uint8_t cx, uint8_t cy,
                                    uint8_t kind[GRID_NAV_SIZE][GRID_NAV_SIZE],
                                    uint8_t obst[GRID_NAV_SIZE][GRID_NAV_SIZE])
{
  int16_t x0;
  int16_t x1;
  int16_t y0;
  int16_t y1;
  int16_t x;
  int16_t y;

  if ((kind[cy][cx] == GRID_CELL_BLOCKED) ||
      (kind[cy][cx] == GRID_CELL_PAD) ||
      (obst[cy][cx] != 0U))
  {
    return 0U;
  }

  x0 = ((int16_t)cx * (int16_t)GRID_NAV_CELL_MM) + (GRID_NAV_CELL_MM / 2) -
       (int16_t)(GRID_NAV_CAR_MM / 2U);
  x1 = x0 + (int16_t)GRID_NAV_CAR_MM;
  y0 = ((int16_t)cy * (int16_t)GRID_NAV_CELL_MM) + (GRID_NAV_CELL_MM / 2) -
       (int16_t)(GRID_NAV_CAR_MM / 2U);
  y1 = y0 + (int16_t)GRID_NAV_CAR_MM;

  if ((x0 < 0) || (y0 < 0) ||
      (x1 > (int16_t)GRID_NAV_FIELD_MM) || (y1 > (int16_t)GRID_NAV_FIELD_MM))
  {
    return 0U;
  }

  for (y = (y0 / (int16_t)GRID_NAV_CELL_MM);
       y <= (y1 / (int16_t)GRID_NAV_CELL_MM); ++y)
  {
    for (x = (x0 / (int16_t)GRID_NAV_CELL_MM);
         x <= (x1 / (int16_t)GRID_NAV_CELL_MM); ++x)
    {
      if ((kind[y][x] == GRID_CELL_BLOCKED) ||
          (kind[y][x] == GRID_CELL_PAD) ||
          (obst[y][x] != 0U))
      {
        return 0U;
      }
    }
  }
  return 1U;
}

static void GridNav_RebuildWalkable(void)
{
  uint8_t cx;
  uint8_t cy;

  for (cy = 0U; cy < GRID_NAV_SIZE; ++cy)
  {
    for (cx = 0U; cx < GRID_NAV_SIZE; ++cx)
    {
      grid_nav_walkable[cy][cx] =
          GridNav_CellPassable(cx, cy, grid_nav_cell_kind, grid_nav_obstacle);
    }
  }
}

uint8_t GridNav_CellType(uint8_t x, uint8_t y)
{
  if ((x >= GRID_NAV_SIZE) || (y >= GRID_NAV_SIZE))
  {
    return GRID_CELL_BLOCKED;
  }
  return grid_nav_cell_kind[y][x];
}

uint8_t GridNav_IsWalkable(uint8_t x, uint8_t y)
{
  if ((x >= GRID_NAV_SIZE) || (y >= GRID_NAV_SIZE))
  {
    return 0U;
  }
  return grid_nav_walkable[y][x];
}

uint8_t GridNav_IsObstacle(uint8_t x, uint8_t y)
{
  if ((x >= GRID_NAV_SIZE) || (y >= GRID_NAV_SIZE))
  {
    return 1U;
  }
  return grid_nav_obstacle[y][x];
}
uint8_t GridNav_HasObstacles(void)
{
  uint8_t x,y;
  for(y=0U;y<GRID_NAV_SIZE;y++)for(x=0U;x<GRID_NAV_SIZE;x++)
    if(grid_nav_obstacle[y][x])return 1U;
  return 0U;
}

uint8_t GridNav_FindWaypoint(const char *name, uint16_t *x_mm, uint16_t *y_mm)
{
  uint8_t i;

  if ((name == NULL) || (x_mm == NULL) || (y_mm == NULL))
  {
    return 0U;
  }
  for (i = 0U; i < (uint8_t)(sizeof(grid_nav_waypoints) /
                             sizeof(grid_nav_waypoints[0])); ++i)
  {
    if (strcmp(name, grid_nav_waypoints[i].name) == 0)
    {
      *x_mm = grid_nav_waypoints[i].x;
      *y_mm = grid_nav_waypoints[i].y;
      return 1U;
    }
  }
  return 0U;
}

uint8_t GridNav_AddObstacleDiscMm(uint16_t x,uint16_t y,uint16_t radius)
{
  uint8_t cx,cy,added=0U;
  if(x>=GRID_NAV_FIELD_MM || y>=GRID_NAV_FIELD_MM || radius>100U)return 0U;
  for(cy=0U;cy<GRID_NAV_SIZE;cy++)for(cx=0U;cx<GRID_NAV_SIZE;cx++) {
    int32_t left=(int32_t)cx*(int32_t)GRID_NAV_CELL_MM;
    int32_t bottom=((int32_t)GRID_NAV_SIZE-1-(int32_t)cy)*(int32_t)GRID_NAV_CELL_MM;
    int32_t right=left+(int32_t)GRID_NAV_CELL_MM,top=bottom+(int32_t)GRID_NAV_CELL_MM;
    int32_t dx=x<left?left-x:(x>right?x-right:0);
    int32_t dy=y<bottom?bottom-y:(y>top?y-top:0);
    if(dx*dx+dy*dy<=(int32_t)radius*radius && !grid_nav_obstacle[cy][cx]) {
      grid_nav_obstacle[cy][cx]=1U;added++;
    }
  }
  if(added){GridNav_RebuildWalkable();grid_nav_revision++;}
  return added;
}

void GridNav_SetObstacle(uint8_t x, uint8_t y, uint8_t blocked)
{
  if ((x < GRID_NAV_SIZE) && (y < GRID_NAV_SIZE))
  {
    uint8_t value = (blocked != 0U) ? 1U : 0U;
    if (grid_nav_obstacle[y][x] != value)
    {
      grid_nav_obstacle[y][x] = value;
      GridNav_RebuildWalkable();
      grid_nav_revision++;
    }
  }
}

uint8_t GridNav_ReplaceObstaclesMm(const uint16_t *x,const uint16_t *y,uint8_t count)
{
  uint8_t i;
  if(count>19U || (count && (!x || !y)))return 0U;
  for(i=0;i<count;i++)if(x[i]>=GRID_NAV_FIELD_MM || y[i]>=GRID_NAV_FIELD_MM)return 0U;
  memset(grid_nav_obstacle,0,sizeof(grid_nav_obstacle));
  for(i=0;i<count;i++)grid_nav_obstacle[GridNav_MmToCellY(y[i])][GridNav_MmToCellX(x[i])]=1U;
  GridNav_RebuildWalkable();grid_nav_has_plan=0U;
  grid_nav_last_path_length=grid_nav_last_segment_count=0U;grid_nav_revision++;
  return 1U;
}

void GridNav_SetPose(uint8_t x, uint8_t y)
{
  if ((x < GRID_NAV_SIZE) && (y < GRID_NAV_SIZE))
  {
    if ((grid_nav_pose_x != x) || (grid_nav_pose_y != y))
    {
      grid_nav_pose_x = x;
      grid_nav_pose_y = y;
      grid_nav_revision++;
    }
  }
}

void GridNav_GetPose(uint8_t *x, uint8_t *y)
{
  if (x != NULL) { *x = grid_nav_pose_x; }
  if (y != NULL) { *y = grid_nav_pose_y; }
}

/* 对外位姿用 mm（规则图坐标系，左下原点、y 向上）。 */
void GridNav_SetPoseMm(uint16_t x_mm, uint16_t y_mm)
{
  if ((grid_nav_pose_mm_x == x_mm) && (grid_nav_pose_mm_y == y_mm))
  {
    return;
  }
  grid_nav_pose_mm_x = x_mm;
  grid_nav_pose_mm_y = y_mm;
  GridNav_SetPose(GridNav_MmToCellX(x_mm), GridNav_MmToCellY(y_mm));
  /* 屏幕和日志用的都是 **mm** 位姿，但 GridNav_SetPose() 只在"格"变化时才 bump 版本号，
     于是"同一格内挪动"不会触发重画 —— 屏上的黄点会停住不动（最多滞后一格 = 50 mm）。
     这里补一次 bump，让 mm 级的变化也能刷新。 */
  grid_nav_revision++;
}

void GridNav_GetPoseMm(uint16_t *x_mm, uint16_t *y_mm)
{
  if (x_mm != NULL) { *x_mm = grid_nav_pose_mm_x; }
  if (y_mm != NULL) { *y_mm = grid_nav_pose_mm_y; }
}

/* ---------- 圆弧过弯的几何闸门：**可旋转车体**的精确碰撞判定 ----------
 *
 * 为什么不能直接用轴对齐车体膨胀的 grid_nav_walkable：膨胀的前提是"车体轴对齐"，
 * 而圆弧过弯时车头正好在转 90°，车体扫过的是半径约198 mm（= 对角线/2）的外接圆，
 * 比140 mm半边长更大。所以这里按**真实姿态**做矩形-矩形精确判定（分离轴），
 * 而不是查膨胀表。
 *
 * 判定内容（与建图用的是同一套几何）：
 *   ① 车体四角都在场地 [0, FIELD_MM]² 内；
 *   ② 不压四块 450×450 黄区；③ 不压四个工位实物台面；
 *   ④ 不压运行期障碍格（`&obs`）。
 * 单位 mm、规则图坐标系；yaw 用地图角度（0 = +x，逆时针为正），与
 * ZDT_Nav_SegmentHeadingDeg() 同一口径。 */

/* 旋转正方形（边长 GRID_NAV_CAR_MM、长轴单位向量 (c,s)）与轴对齐矩形的重叠判定。
   rect 用 {x0, x1, y0, y1}。返回 1 = 有重叠。 */
static uint8_t GridNav_BodyOverlapsRect(float cx, float cy, float half,
                                        float c, float s,
                                        const int16_t rect[4])
{
  float rx = ((float)rect[0] + (float)rect[1]) * 0.5f;
  float ry = ((float)rect[2] + (float)rect[3]) * 0.5f;
  float hx = ((float)rect[1] - (float)rect[0]) * 0.5f;
  float hy = ((float)rect[3] - (float)rect[2]) * 0.5f;
  float dx = cx - rx;
  float dy = cy - ry;
  float ra;
  float rb;

  /* 轴 x̂=(1,0)：车体投影半径 = half*(|c| + |s|)（正方形两条边都算上）。 */
  ra = hx;
  rb = half * (fabsf(c) + fabsf(s));
  if (fabsf(dx) > (ra + rb)) { return 0U; }
  /* 轴 ŷ=(0,1) */
  ra = hy;
  if (fabsf(dy) > (ra + rb)) { return 0U; }
  /* 轴 = 车体长轴 (c,s)：正方形在这条轴上的半径就是 half。 */
  ra = (hx * fabsf(c)) + (hy * fabsf(s));
  rb = half;
  if (fabsf((dx * c) + (dy * s)) > (ra + rb)) { return 0U; }
  /* 轴 = 车体短轴 (-s,c) */
  ra = (hx * fabsf(s)) + (hy * fabsf(c));
  if (fabsf((dx * (-s)) + (dy * c)) > (ra + rb)) { return 0U; }

  return 1U;   /* 四条轴都分离失败 → 确实重叠 */
}

uint8_t GridNav_RotatedBodyFreeMm(uint16_t x_mm, uint16_t y_mm, int16_t yaw_deg)
{
  return GridNav_RotatedBodyFreeMarginMm(x_mm, y_mm, yaw_deg, 0U);
}

uint8_t GridNav_RotatedBodyFreeMarginMm(uint16_t x_mm, uint16_t y_mm,
                                       int16_t yaw_deg, uint16_t margin_mm)
{
  const float half = (float)GRID_NAV_CAR_MM * 0.5f + (float)margin_mm;
  const float field = (float)GRID_NAV_FIELD_MM;
  float rad = (float)yaw_deg * 0.017453292f;
  float c = cosf(rad);
  float s = sinf(rad);
  float cx = (float)x_mm;
  float cy = (float)y_mm;
  float ex;
  float ey;
  int16_t cell_rect[4];

  /* ① 车体四角必须都在场地内（旋转后的角 = centre ± half*(长轴) ± half*(短轴)）。 */
  for (ex = -1.0f; ex <= 1.0f; ex += 2.0f)
  {
    for (ey = -1.0f; ey <= 1.0f; ey += 2.0f)
    {
      float px = cx + (half * ((ex * c) - (ey * s)));
      float py = cy + (half * ((ex * s) + (ey * c)));

      if ((px < 0.0f) || (py < 0.0f) || (px > field) || (py > field))
      {
        return 0U;
      }
    }
  }

  /* ②③ 黄区与工位台面 */
  {
    uint8_t i;

    for (i = 0U; i < 4U; ++i)
    {
      if ((GridNav_BodyOverlapsRect(cx, cy, half, c, s,
                                    grid_nav_zone_block_rects[i]) != 0U) ||
          (GridNav_BodyOverlapsRect(cx, cy, half, c, s,
                                    grid_nav_pad_rects[i]) != 0U))
      {
        return 0U;
      }
    }
  }

  /* ④ 运行期障碍格：逐个当成 50×50 的方块精确判定（只扫车体 AABB 覆盖到的格）。
     注意**两套坐标**：`grid_nav_obstacle[]` 的行号向下递增，而这里的 rect 是
     规则图 mm（y 向上），所以行号要经 GridNav_MmToCellY() 换算、格子的 mm 矩形要经
     GridNav_CellCenterMmY() 反算 —— 直接拿行号乘 50 会上下镜像。 */
  {
    float reach = half * (fabsf(c) + fabsf(s));
    int16_t icx_lo = (int16_t)GridNav_MmToCellX((uint16_t)((cx > reach) ? (cx - reach) : 0.0f));
    int16_t icx_hi = (int16_t)GridNav_MmToCellX((uint16_t)
        (((cx + reach) < field) ? (cx + reach) : field));
    int16_t icy_lo = (int16_t)GridNav_MmToCellY((uint16_t)
        (((cy + reach) < field) ? (cy + reach) : field));
    int16_t icy_hi = (int16_t)GridNav_MmToCellY((uint16_t)((cy > reach) ? (cy - reach) : 0.0f));
    int16_t icx;
    int16_t icy;

    for (icx = icx_lo; icx <= icx_hi; ++icx)
    {
      for (icy = icy_lo; icy <= icy_hi; ++icy)
      {
        uint8_t cell_x = (uint8_t)icx;
        uint8_t cell_y = (uint8_t)icy;
        int16_t base_y;

        if (GridNav_IsObstacle(cell_x, cell_y) == 0U)
        {
          continue;
        }
        base_y = (int16_t)((int16_t)GridNav_CellCenterMmY(cell_y) -
                           (int16_t)(GRID_NAV_CELL_MM / 2U));
        cell_rect[0] = (int16_t)(icx * (int16_t)GRID_NAV_CELL_MM);
        cell_rect[1] = (int16_t)(cell_rect[0] + (int16_t)GRID_NAV_CELL_MM);
        cell_rect[2] = base_y;
        cell_rect[3] = (int16_t)(base_y + (int16_t)GRID_NAV_CELL_MM);
        if (GridNav_BodyOverlapsRect(cx, cy, half, c, s, cell_rect) != 0U)
        {
          return 0U;
        }
      }
    }
  }

  return 1U;
}

void GridNav_GetTargetMm(uint16_t *x_mm, uint16_t *y_mm)
{
  if (x_mm != NULL) { *x_mm = grid_nav_last_target_mm_x; }
  if (y_mm != NULL) { *y_mm = grid_nav_last_target_mm_y; }
}

uint32_t GridNav_GetRevision(void)
{
  return grid_nav_revision;
}

/* 代价模型：执行器在**每次规划前**调用，把"转向耗时相当于走几格"填进来。
 * 为什么放在执行器算：只有它知道当前 `spd`（走一格要多久）和现场实测的转向耗时。
 * 这里只做区间防呆：转向代价至少等于 1 格（否则 A* 会用一堆掉头去凑等长路径），
 * 至多 400 格（防止极慢速下把代价算飞）。 */
void GridNav_SetCostModel(uint16_t turn90_units, uint16_t turn180_units)
{
  if (turn90_units < GRID_NAV_MOVE_COST)
  {
    turn90_units = GRID_NAV_MOVE_COST;
  }
  if (turn90_units > 400U)
  {
    turn90_units = 400U;
  }
  if (turn180_units < turn90_units)
  {
    turn180_units = (uint16_t)(turn90_units * 2U);
  }
  if (turn180_units > 800U)
  {
    turn180_units = 800U;
  }
  grid_nav_turn90_cost = turn90_units;
  grid_nav_turn180_cost = turn180_units;
}

/* 格心离最近车道中心线有几格（0 = 就在中线 ±25 mm 内）。
   横向用格心的 mm 坐标、纵向也用它 —— 车道中线在 x 和 y 上是同一组值
   （275 / 1200 / 2125），取两条里较小的偏离即可（转弯处两条都贴着中线）。 */
static uint8_t GridNav_LaneDeviationCells(uint8_t cx, uint8_t cy)
{
  uint16_t x = GridNav_CellCenterMmX(cx);
  uint16_t y = GridNav_CellCenterMmY(cy);
  uint16_t dx = (x > GridNav_LaneMm(x)) ? (uint16_t)(x - GridNav_LaneMm(x))
                                        : (uint16_t)(GridNav_LaneMm(x) - x);
  uint16_t dy = (y > GridNav_LaneMm(y)) ? (uint16_t)(y - GridNav_LaneMm(y))
                                        : (uint16_t)(GridNav_LaneMm(y) - y);
  uint16_t deviation = (dx < dy) ? dx : dy;

  return (uint8_t)(deviation / GRID_NAV_CELL_MM);
}

static void GridNav_ClearNodes(void)
{
  uint16_t i;

  memset(grid_nav_nodes, 0, sizeof(grid_nav_nodes));
  grid_nav_open_count = 0U;
  for (i = 0U; i < GRID_NAV_NODE_COUNT; ++i)
  {
    grid_nav_nodes[i].x = (uint8_t)(i % GRID_NAV_SIZE);
    grid_nav_nodes[i].y = (uint8_t)(i / GRID_NAV_SIZE);
    grid_nav_nodes[i].parent = GRID_NAV_INVALID_INDEX;
  }
}

static void GridNav_OpenNode(uint16_t index, uint16_t parent,
                             uint16_t g, uint16_t f, uint8_t in_dir)
{
  grid_nav_node_t *node = &grid_nav_nodes[index];

  if (node->opened == 0U)
  {
    grid_nav_open[grid_nav_open_count++] = index;
    node->opened = 1U;
  }
  if ((node->parent == GRID_NAV_INVALID_INDEX) || (g < node->g))
  {
    node->parent = parent;
    node->g = g;
    node->f = f;
    node->in_dir = in_dir;
  }
}

static uint16_t GridNav_FindBestOpen(void)
{
  uint16_t i;
  uint16_t best_index = GRID_NAV_INVALID_INDEX;
  uint16_t best_f = 0xFFFFU;

  for (i = 0U; i < grid_nav_open_count; ++i)
  {
    uint16_t index = grid_nav_open[i];
    grid_nav_node_t *node = &grid_nav_nodes[index];
    if ((node->closed == 0U) && (node->f < best_f))
    {
      best_f = node->f;
      best_index = index;
    }
  }
  return best_index;
}

/* 把 A* 格子路径合并成连续直线段，避免执行时每个格子都停车。 */
static void GridNav_BuildSegments(const grid_nav_point_t *path,
                                  uint16_t path_length)
{
  uint16_t i;

  grid_nav_last_segment_count = 0U;
  for (i = 1U; i < path_length; ++i)
  {
    int8_t dx = (int8_t)path[i].x - (int8_t)path[i - 1U].x;
    int8_t dy = (int8_t)path[i].y - (int8_t)path[i - 1U].y;

    if ((grid_nav_last_segment_count != 0U) &&
        (grid_nav_last_segments[grid_nav_last_segment_count - 1U].dx == dx) &&
        (grid_nav_last_segments[grid_nav_last_segment_count - 1U].dy == dy))
    {
      grid_nav_segment_t *segment =
          &grid_nav_last_segments[grid_nav_last_segment_count - 1U];
      segment->to_x = path[i].x;
      segment->to_y = path[i].y;
      segment->length++;
    }
    else
    {
      grid_nav_segment_t *segment =
          &grid_nav_last_segments[grid_nav_last_segment_count++];
      segment->from_x = path[i - 1U].x;
      segment->from_y = path[i - 1U].y;
      segment->to_x = path[i].x;
      segment->to_y = path[i].y;
      segment->dx = dx;
      segment->dy = dy;
      segment->length = 1U;
    }
  }
}

static void GridNav_StorePlan(const grid_nav_point_t *path, uint16_t length,
                              uint8_t target_x, uint8_t target_y,
                              uint16_t target_mm_x, uint16_t target_mm_y)
{
  memcpy(grid_nav_last_path, path, length * sizeof(path[0]));
  grid_nav_last_path_length = length;
  grid_nav_last_target_x = target_x;
  grid_nav_last_target_y = target_y;
  grid_nav_last_target_mm_x = target_mm_x;
  grid_nav_last_target_mm_y = target_mm_y;
  memset(grid_nav_exact_distance_mm,0,sizeof(grid_nav_exact_distance_mm));
  GridNav_BuildSegments(path, length);
  grid_nav_has_plan = 1U;
  grid_nav_revision++;
}

uint8_t GridNav_GetSegments(grid_nav_segment_t *segments,
                            uint16_t capacity, uint16_t *segment_count)
{
  if ((segments == NULL) || (segment_count == NULL) ||
      (grid_nav_has_plan == 0U) || (capacity == 0U) ||
      (grid_nav_last_segment_count > capacity))
  {
    return 0U;
  }
  memcpy(segments, grid_nav_last_segments,
         grid_nav_last_segment_count * sizeof(segments[0]));
  *segment_count = grid_nav_last_segment_count;
  return 1U;
}

uint16_t GridNav_GetSegmentDistanceMm(uint16_t segment_index)
{
  if(!grid_nav_has_plan || segment_index>=grid_nav_last_segment_count)return 0U;
  if(grid_nav_exact_distance_mm[segment_index]!=0U)
    return grid_nav_exact_distance_mm[segment_index];
  return (uint16_t)(grid_nav_last_segments[segment_index].length*GRID_NAV_CELL_MM);
}

uint8_t GridNav_GetPlan(grid_nav_point_t *path, uint16_t capacity,
                        uint16_t *path_length, uint8_t *target_x,
                        uint8_t *target_y)
{
  if ((path == NULL) || (path_length == NULL) ||
      (grid_nav_has_plan == 0U) || (capacity == 0U))
  {
    return 0U;
  }
  if (grid_nav_last_path_length > capacity)
  {
    return 0U;
  }
  memcpy(path, grid_nav_last_path, grid_nav_last_path_length * sizeof(path[0]));
  *path_length = grid_nav_last_path_length;
  if (target_x != NULL) { *target_x = grid_nav_last_target_x; }
  if (target_y != NULL) { *target_y = grid_nav_last_target_y; }
  return 1U;
}

/* 核心 A*：只走"车能过"的格（已含车体外廓膨胀）。 */
uint8_t GridNav_Plan(uint8_t start_x, uint8_t start_y,
                     uint8_t target_x, uint8_t target_y,
                     grid_nav_point_t *path, uint16_t *path_length)
{
  uint16_t start;
  uint16_t target;
  uint16_t current;

  if ((path == NULL) || (path_length == NULL) ||
      (start_x >= GRID_NAV_SIZE) || (start_y >= GRID_NAV_SIZE) ||
      (target_x >= GRID_NAV_SIZE) || (target_y >= GRID_NAV_SIZE) ||
      (grid_nav_walkable[start_y][start_x] == 0U) ||
      (grid_nav_walkable[target_y][target_x] == 0U))
  {
    return 0U;
  }

  GridNav_ClearNodes();
  start = GridNav_Index(start_x, start_y);
  target = GridNav_Index(target_x, target_y);
  grid_nav_nodes[start].parent = GRID_NAV_INVALID_INDEX;
  grid_nav_nodes[start].in_dir = 0xFFU;
  grid_nav_nodes[start].g = 0U;
  grid_nav_nodes[start].f = GridNav_Heuristic(start_x, start_y,
                                              target_x, target_y);
  GridNav_OpenNode(start, GRID_NAV_INVALID_INDEX, 0U,
                   grid_nav_nodes[start].f, 0xFFU);

  while (1)
  {
    uint8_t direction;
    uint16_t count;

    current = GridNav_FindBestOpen();
    if (current == GRID_NAV_INVALID_INDEX)
    {
      return 0U;
    }

    if (current == target)
    {
      uint16_t index = current;

      count = 0U;
      while (index != GRID_NAV_INVALID_INDEX)
      {
        grid_nav_scratch_reversed[count++] = index;
        if (index == start)
        {
          break;
        }
        index = grid_nav_nodes[index].parent;
      }
      if ((count == 0U) ||
          (grid_nav_scratch_reversed[count - 1U] != start))
      {
        return 0U;
      }
      for (uint16_t i = 0U; i < count; ++i)
      {
        uint16_t node_index = grid_nav_scratch_reversed[count - 1U - i];
        path[i].x = grid_nav_nodes[node_index].x;
        path[i].y = grid_nav_nodes[node_index].y;
      }
      *path_length = count;
      return 1U;
    }

    grid_nav_nodes[current].closed = 1U;
    for (direction = 0U; direction < 4U; ++direction)
    {
      int16_t nx = (int16_t)grid_nav_nodes[current].x + grid_nav_dx[direction];
      int16_t ny = (int16_t)grid_nav_nodes[current].y + grid_nav_dy[direction];
      uint16_t neighbor;
      uint16_t tentative_g;
      uint16_t step_cost;

      if ((nx < 0) || (ny < 0) ||
          (nx >= (int16_t)GRID_NAV_SIZE) ||
          (ny >= (int16_t)GRID_NAV_SIZE) ||
          (grid_nav_walkable[ny][nx] == 0U))
      {
        continue;
      }

      neighbor = GridNav_Index((uint8_t)nx, (uint8_t)ny);
      if (grid_nav_nodes[neighbor].closed != 0U)
      {
        continue;
      }
      /* 转向代价：与进入当前格的方向不同就加一笔（起点 0xFF 不算转向）。
         ★ 2026-09-13：**按角度区分直角与掉头**。现场发现 `temp→rough` 两条路
         曼哈顿长度完全相同（都 34 格），差别只在"90°+90° 走外圈"与"180°+90° 走中间"，
         而原来两者同价 → A* 只能随手选一条（现场看到的就是"该走外圈却掉头"）。
         现实耗时差一倍（90° 实测约 1.68 s），所以代价也必须差一倍。 */
      step_cost = GRID_NAV_MOVE_COST;
      if ((grid_nav_nodes[current].in_dir != 0xFFU) &&
          (grid_nav_nodes[current].in_dir != direction))
      {
        uint8_t diff = (uint8_t)((grid_nav_nodes[current].in_dir > direction)
                                     ? (grid_nav_nodes[current].in_dir - direction)
                                     : (direction - grid_nav_nodes[current].in_dir));

        step_cost = (uint16_t)(step_cost +
            ((diff == 2U) ? grid_nav_turn180_cost : grid_nav_turn90_cost));
      }
      /* 居中度：离最近的车道中心线越远，代价越高（软约束，不禁行）。
         现场要求"尽可能让车辆行驶在路中间"：外环带 550 宽，膨胀后可走的 6 格里
         最外那条格心贴着场边，最里那条贴着禁区 —— 不加这一笔，A* 会随手挑靠边的
         一格，车看起来就是"贴着墙走"。有了它，路径会自然收敛到 275/1200/2125
         这三条中线附近（格心都在中线 ±25，所以最多再多走 1 格）。 */
      step_cost = (uint16_t)(step_cost + (uint16_t)(
          GRID_NAV_LANE_COST_PER_CELL *
          GridNav_LaneDeviationCells((uint8_t)nx, (uint8_t)ny)));
      tentative_g = (uint16_t)(grid_nav_nodes[current].g + step_cost);
      if ((grid_nav_nodes[neighbor].opened == 0U) ||
          (tentative_g < grid_nav_nodes[neighbor].g))
      {
        GridNav_OpenNode(neighbor, current, tentative_g,
                         (uint16_t)(tentative_g +
                         GridNav_Heuristic((uint8_t)nx, (uint8_t)ny,
                                           target_x, target_y)),
                         direction);
      }
    }
  }
}

/* 把一个格吸附到最近的"车能过"格。
 * 为什么必需：规则图上的停位是"整 mm"值（例如原料区 1200/2250 —— 车头前缘正好顶到
 * y=2400），但格心在 25+50k 上，2250 落在格边界 → 它最近格心是 2275，
 * 而车心在 2275 时前缘到 2425，**超出场地 25 mm**，膨胀判定为不可通行。
 * 于是直接拿停位去规划会得到 NO PATH（离线校验实测过）。
 * 这里按半径递增找最近的可行格；执行器仍然把车开到**精确 mm 停位**
 * （最后一段的段长会按目标 mm 修正）。 */
static uint8_t GridNav_SnapToWalkable(uint8_t *cx, uint8_t *cy)
{
  uint8_t radius;

  if (grid_nav_walkable[*cy][*cx] != 0U)
  {
    return 1U;
  }
  for (radius = 1U; radius <= 8U; ++radius)
  {
    int16_t best_x = -1;
    int16_t best_y = -1;
    uint16_t best_dist = 0xFFFFU;
    int16_t dx;
    int16_t dy;

    for (dy = -(int16_t)radius; dy <= (int16_t)radius; ++dy)
    {
      for (dx = -(int16_t)radius; dx <= (int16_t)radius; ++dx)
      {
        int16_t nx = (int16_t)*cx + dx;
        int16_t ny = (int16_t)*cy + dy;
        uint16_t dist;

        if ((nx < 0) || (ny < 0) ||
            (nx >= (int16_t)GRID_NAV_SIZE) || (ny >= (int16_t)GRID_NAV_SIZE))
        {
          continue;
        }
        if (grid_nav_walkable[ny][nx] == 0U)
        {
          continue;
        }
        dist = (uint16_t)((dx * dx) + (dy * dy));
        if (dist < best_dist)
        {
          best_dist = dist;
          best_x = nx;
          best_y = ny;
        }
      }
    }
    if (best_x >= 0)
    {
      *cx = (uint8_t)best_x;
      *cy = (uint8_t)best_y;
      return 1U;
    }
  }
  return 0U;
}

/* mm 入口：把 mm 起点/终点换算成格、吸附到可行格再规划；终点 mm 原样保存给执行器用，
   这样执行器可以停到"精确 mm"（例如 1200/2250），而不是格心（1225/2275）。 */
uint8_t GridNav_PlanMm(uint16_t start_x, uint16_t start_y,
                       uint16_t target_x, uint16_t target_y,
                       grid_nav_point_t *path, uint16_t *path_length)
{
  uint8_t sx = GridNav_MmToCellX(start_x);
  uint8_t sy = GridNav_MmToCellY(start_y);
  uint8_t tx = GridNav_MmToCellX(target_x);
  uint8_t ty = GridNav_MmToCellY(target_y);
  uint8_t sx_raw = sx;
  uint8_t sy_raw = sy;
  uint8_t tx_raw = tx;
  uint8_t ty_raw = ty;

  if ((GridNav_SnapToWalkable(&sx, &sy) == 0U) ||
      (GridNav_SnapToWalkable(&tx, &ty) == 0U))
  {
    return 0U;
  }

  /* 吸附了就明确告警：像 `&goto 0 0`（场地角上、车280 mm放不下）这种目标，
     静默吸附会让现场以为"到那儿了"，其实停位差很远。 */
  if ((sx != sx_raw) || (sy != sy_raw) || (tx != tx_raw) || (ty != ty_raw))
  {
    char line[168];

    (void)snprintf(line, sizeof(line),
                   "[NAV] note: not drivable -> snap start (%u,%u)->(%u,%u) "
                   "target (%u,%u)->(%u,%u); stop at (%umm,%umm) not (%umm,%umm)\r\n",
                   (unsigned int)sx_raw, (unsigned int)sy_raw,
                   (unsigned int)sx, (unsigned int)sy,
                   (unsigned int)tx_raw, (unsigned int)ty_raw,
                   (unsigned int)tx, (unsigned int)ty,
                   (unsigned int)GridNav_CellCenterMmX(tx),
                   (unsigned int)GridNav_CellCenterMmY(ty),
                   (unsigned int)target_x, (unsigned int)target_y);
    GridNav_Print(line);
  }

  return GridNav_Plan(sx, sy, tx, ty, path, path_length);
}

static void GridNav_PrintPath(const grid_nav_point_t *path, uint16_t length)
{
  char line[160];
  int offset = 0;
  uint16_t i;

  GridNav_Print("[NAV] PATH ");
  for (i = 0U; i < length; ++i)
  {
    int written = snprintf(&line[offset], sizeof(line) - (size_t)offset,
                           "(%u,%u)%s",
                           (unsigned int)path[i].x, (unsigned int)path[i].y,
                           (i + 1U < length) ? " -> " : "\r\n");
    if ((written <= 0) || ((size_t)(offset + written) >= sizeof(line)))
    {
      line[sizeof(line) - 3U] = '\r';
      line[sizeof(line) - 2U] = '\n';
      line[sizeof(line) - 1U] = '\0';
      GridNav_Print(line);
      offset = 0;
      line[0] = '\0';
      written = snprintf(line, sizeof(line), "(%u,%u)%s",
                         (unsigned int)path[i].x, (unsigned int)path[i].y,
                         (i + 1U < length) ? " -> " : "\r\n");
    }
    offset += written;
    if ((i + 1U == length) || (offset > 120))
    {
      line[sizeof(line) - 1U] = '\0';
      GridNav_Print(line);
      offset = 0;
      line[0] = '\0';
    }
  }
}

/* 方向字母沿用旧约定（D=+y 等），并额外给出 mm 段长，便于执行器核对。 */
/* 打印用的方向字母。
 * ⚠ 必须按规则图口径：内部行号 cy 是**向下递增**（cy = 47 - y/50），
 * 所以 dy>0 = 朝地图下方 = 'D'，dy<0 = 朝上方 = 'U'。
 * （2026-09-12 这里写反过一次：把 dy>0 打成了 'U'，日志里的段方向会误导现场。） */
static char GridNav_DirectionLetter(int8_t dx, int8_t dy)
{
  if (dx > 0) { return 'R'; }
  if (dx < 0) { return 'L'; }
  if (dy > 0) { return 'D'; }
  return 'U';
}

static void GridNav_PrintSegments(void)
{
  char line[128];
  uint16_t i;

  for (i = 0U; i < grid_nav_last_segment_count; ++i)
  {
    const grid_nav_segment_t *segment = &grid_nav_last_segments[i];
    char letter = GridNav_DirectionLetter(segment->dx, segment->dy);
    uint16_t length_mm = (uint16_t)(segment->length * GRID_NAV_CELL_MM);

    (void)snprintf(line, sizeof(line),
                   "[NAV] SEG%u %c %ucell=%umm (%u,%u)->(%u,%u)\r\n",
                   (unsigned int)(i + 1U), letter,
                   (unsigned int)segment->length, (unsigned int)length_mm,
                   (unsigned int)segment->from_x,
                   (unsigned int)segment->from_y,
                   (unsigned int)segment->to_x,
                   (unsigned int)segment->to_y);
    GridNav_Print(line);
  }
}

/* 打印整张 48x48 地图（1:1）。大写 = 车能过（含车体外廓膨胀），
   小写 = 地面可走但车过不去，'#' = 运行期障碍，'.' = 黄区禁区，
   'S'/'s' = 启停区，'W'/'w' = 工位，'R'/'r' = 十字通道，'o'/'O' = 外环带。
   @ = 小车当前格，+ = 本次路径终点。 */
void GridNav_PrintMap(void)
{
  char line[GRID_NAV_SIZE + 8U];
  char head[96];
  uint8_t x;
  uint8_t y;
  uint8_t row;

  GridNav_Print("\r\n[GRID] 48x48 x 50mm  origin=left-bottom  x=right  y=UP  unit=mm\r\n");
  GridNav_Print("[GRID] legend: R/o=road S=start W=work(area) P=pad(NO-CONTACT)\r\n");
  GridNav_Print("[GRID]         lower-case = too narrow for the car body\r\n");
  GridNav_Print("[GRID]         #=obstacle .=yellow-block  @=car  +=goal\r\n");
  GridNav_Print("[GRID]      y=2400 top\r\n");

  for (row = 0U; row < GRID_NAV_SIZE; ++row)
  {
    /* 行号内部是"向下递增"，打印按规则图习惯自上而下：先打 cy=47（y_up 最大） */
    y = (uint8_t)(GRID_NAV_SIZE - 1U - row);
    int offset = snprintf(line, sizeof(line), "[GRID] y=%4u ",
                          (unsigned int)GridNav_CellCenterMmY(y));
    for (x = 0U; x < GRID_NAV_SIZE; ++x)
    {
      char symbol;
      uint8_t walkable = grid_nav_walkable[y][x];

      switch (grid_nav_cell_kind[y][x])
      {
        case GRID_CELL_START:   symbol = (walkable != 0U) ? 'S' : 's'; break;
        case GRID_CELL_WORK:    symbol = (walkable != 0U) ? 'W' : 'w'; break;
        case GRID_CELL_PAD:     symbol = (walkable != 0U) ? 'P' : 'p'; break;
        case GRID_CELL_ROAD:    symbol = (walkable != 0U) ? 'R' : 'r'; break;
        case GRID_CELL_RING:    symbol = (walkable != 0U) ? 'o' : 'O'; break;
        default:                symbol = '.'; break;
      }
      if (grid_nav_obstacle[y][x] != 0U)
      {
        symbol = '#';
      }
      if ((grid_nav_has_plan != 0U) &&
          (x == grid_nav_last_target_x) && (y == grid_nav_last_target_y))
      {
        symbol = '+';
      }
      if ((x == grid_nav_pose_x) && (y == grid_nav_pose_y))
      {
        symbol = '@';
      }
      line[offset++] = symbol;
    }
    line[offset++] = '\r';
    line[offset++] = '\n';
    line[offset] = '\0';
    GridNav_Print(line);
  }

  (void)snprintf(head, sizeof(head),
                 "[GRID] pose=(%u,%u)cell = (%umm,%umm)  target=(%umm,%umm)\r\n",
                 (unsigned int)grid_nav_pose_x, (unsigned int)grid_nav_pose_y,
                 (unsigned int)grid_nav_pose_mm_x,
                 (unsigned int)grid_nav_pose_mm_y,
                 (unsigned int)grid_nav_last_target_mm_x,
                 (unsigned int)grid_nav_last_target_mm_y);
  GridNav_Print(head);
}

/* 严格解析 0..9999 的十进制数字，兼容空格和逗号分隔。 */
static uint8_t GridNav_ParseNumber(const char **text, uint16_t *value)
{
  const char *cursor = *text;
  uint32_t parsed = 0U;

  while ((*cursor == ' ') || (*cursor == '\t') || (*cursor == ','))
  {
    cursor++;
  }
  if ((*cursor < '0') || (*cursor > '9'))
  {
    return 0U;
  }
  while ((*cursor >= '0') && (*cursor <= '9'))
  {
    parsed = (parsed * 10U) + (uint32_t)(*cursor - '0');
    if (parsed > 9999U)
    {
      return 0U;
    }
    cursor++;
  }
  *text = cursor;
  *value = (uint16_t)parsed;
  return 1U;
}

static uint8_t GridNav_ParsePair(const char *text, uint16_t *a, uint16_t *b)
{
  uint16_t first;
  uint16_t second;

  if (GridNav_ParseNumber(&text, &first) == 0U)
  {
    return 0U;
  }
  if ((*text != ' ') && (*text != '\t') && (*text != ','))
  {
    return 0U;
  }
  if (GridNav_ParseNumber(&text, &second) == 0U)
  {
    return 0U;
  }
  while ((*text == ' ') || (*text == '\t') || (*text == ','))
  {
    text++;
  }
  if (*text != '\0')
  {
    return 0U;
  }
  *a = first;
  *b = second;
  return 1U;
}

static uint8_t GridNav_ParseToken(const char **text, char *name, size_t capacity)
{
  const char *cursor = *text;
  size_t length = 0U;

  while ((*cursor == ' ') || (*cursor == '\t') || (*cursor == ','))
  {
    cursor++;
  }
  while (((*cursor >= 'a') && (*cursor <= 'z')) ||
         ((*cursor >= 'A') && (*cursor <= 'Z')) ||
         ((*cursor >= '0') && (*cursor <= '9')) || (*cursor == '_'))
  {
    if (length + 1U < capacity)
    {
      name[length++] = (char)tolower((unsigned char)*cursor);
    }
    cursor++;
  }
  name[length] = '\0';
  while ((*cursor == ' ') || (*cursor == '\t') || (*cursor == ','))
  {
    cursor++;
  }
  *text = cursor;
  return (uint8_t)(length != 0U);
}

/* 解析 "1 2"(格) / "raw"(命名) / "1200 2250"(mm，超过 47 必然是 mm) 三种写法。
   判定规则：命中命名表 → 命名；两个数都 < 48 → 格；否则按 mm。 */
static uint8_t GridNav_ResolveTarget(const char *text, uint16_t *x_mm,
                                     uint16_t *y_mm)
{
  char name[24];
  uint16_t a;
  uint16_t b;
  const char *cursor = text;

  if (GridNav_ParseToken(&cursor, name, sizeof(name)) != 0U)
  {
    if ((*cursor == '\0') && (GridNav_FindWaypoint(name, x_mm, y_mm) != 0U))
    {
      return 1U;
    }
  }
  if (GridNav_ParsePair(text, &a, &b) == 0U)
  {
    return 0U;
  }
  if ((a < GRID_NAV_SIZE) && (b < GRID_NAV_SIZE))
  {
    *x_mm = GridNav_CellCenterMmX((uint8_t)a);
    *y_mm = GridNav_CellCenterMmY((uint8_t)b);
    return 1U;
  }
  if ((a <= GRID_NAV_FIELD_MM) && (b <= GRID_NAV_FIELD_MM))
  {
    *x_mm = a;
    *y_mm = b;
    return 1U;
  }
  return 0U;
}

static uint8_t GridNav_PlanAndStore(uint16_t target_x_mm, uint16_t target_y_mm,
                                    uint8_t verbose)
{
  char line[112];
  uint16_t pose_x_mm;
  uint16_t pose_y_mm;
  uint16_t length = 0U;

  GridNav_GetPoseMm(&pose_x_mm, &pose_y_mm);

  if (GridNav_PlanMm(pose_x_mm, pose_y_mm, target_x_mm, target_y_mm,
                     grid_nav_scratch_path, &length) == 0U)
  {
    GridNav_Print("[NAV] NO PATH (out of drivable area or blocked)\r\n");
    return 0U;
  }

  GridNav_StorePlan(grid_nav_scratch_path, length,
                    GridNav_MmToCellX(target_x_mm), GridNav_MmToCellY(target_y_mm),
                    target_x_mm, target_y_mm);

  if (verbose != 0U)
  {
    GridNav_PrintPath(grid_nav_scratch_path, length);
    GridNav_PrintSegments();
    (void)snprintf(line, sizeof(line),
                   "[NAV] plan ready: goal=(%umm,%umm) cell=(%u,%u) steps=%u\r\n",
                   (unsigned int)target_x_mm, (unsigned int)target_y_mm,
                   (unsigned int)grid_nav_last_target_x,
                   (unsigned int)grid_nav_last_target_y,
                   (unsigned int)(length - 1U));
    GridNav_Print(line);
  }
  return 1U;
}

uint8_t GridNav_PlanTargetMm(uint16_t x_mm, uint16_t y_mm)
{
  return GridNav_PlanAndStore(x_mm, y_mm, 0U);
}

uint8_t GridNav_PlanStraightTargetMm(uint16_t x_mm, uint16_t y_mm)
{
  uint16_t sx=grid_nav_pose_mm_x,sy=grid_nav_pose_mm_y,d,i,n;
  int32_t dx=(int32_t)x_mm-sx,dy=(int32_t)y_mm-sy;
  int8_t step_x=(dx>0)?1:((dx<0)?-1:0);
  int8_t step_y=(dy>0)?-1:((dy<0)?1:0); /* grid rows run down */
  if(x_mm>GRID_NAV_FIELD_MM || y_mm>GRID_NAV_FIELD_MM || (dx!=0 && dy!=0))return 0U;
  d=(uint16_t)(abs(dx)+abs(dy));
  /* The A* cell centre is up to 25mm from the real route. For y=400,
   * its y=425 representative clips yellow blocks although the actual car
   * top is 540mm (<550). Check the actual line, never disable obstacles. */
  n=(uint16_t)((d+4U)/5U);
  for(i=0;i<=n;i++) {
    uint16_t x=(uint16_t)((int32_t)sx+((n!=0U)?dx*i/n:0));
    uint16_t y=(uint16_t)((int32_t)sy+((n!=0U)?dy*i/n:0));
    if(!GridNav_RotatedBodyFreeMm(x,y,0))return 0U;
  }
  /* Display path follows actual endpoint cells. Motion retains d exactly,
   * so 350->2125 travels 1775mm, without a grid-centre detour or rounding. */
  n=(uint16_t)(abs((int)GridNav_MmToCellX(x_mm)-(int)GridNav_MmToCellX(sx))+
      abs((int)GridNav_MmToCellY(y_mm)-(int)GridNav_MmToCellY(sy)));
  for(i=0;i<=n;i++) {
    grid_nav_scratch_path[i].x=(uint8_t)((int16_t)GridNav_MmToCellX(sx)+step_x*(int16_t)i);
    grid_nav_scratch_path[i].y=(uint8_t)((int16_t)GridNav_MmToCellY(sy)+step_y*(int16_t)i);
  }
  GridNav_StorePlan(grid_nav_scratch_path,(uint16_t)(n+1U),GridNav_MmToCellX(x_mm),
      GridNav_MmToCellY(y_mm),x_mm,y_mm);
  if(d!=0U) {
    grid_nav_segment_t *segment=&grid_nav_last_segments[0];
    segment->from_x=GridNav_MmToCellX(sx);segment->from_y=GridNav_MmToCellY(sy);
    segment->to_x=GridNav_MmToCellX(x_mm);segment->to_y=GridNav_MmToCellY(y_mm);
    segment->dx=step_x;segment->dy=step_y;
    segment->length=(uint16_t)((d+GRID_NAV_CELL_MM-1U)/GRID_NAV_CELL_MM);
    grid_nav_last_segment_count=1U;grid_nav_exact_distance_mm[0]=d;
  }
  return 1U;
}

static uint8_t GridNav_RecenterSweep(uint16_t x,uint16_t y,float from,float to)
{
  float delta=remainderf(to-from,360.0f);uint16_t i,n=(uint16_t)ceilf(fabsf(delta)/3.0f);
  for(i=0U;i<=n;i++) {
    float angle=from+(n?delta*i/n:0.0f);
    if(!GridNav_RotatedBodyFreeMm(x,y,(int16_t)lroundf(remainderf(angle,360.0f))))return 0U;
  }
  return 1U;
}
static uint8_t GridNav_RecenterPlan(uint16_t x_mm,uint16_t y_mm,float world_yaw,uint8_t oriented)
{
  uint16_t xs[3],ys[3],lengths[2],i,j,n,count=1U;int angle;
  int32_t dx=(int32_t)x_mm-grid_nav_pose_mm_x,dy=(int32_t)y_mm-grid_nav_pose_mm_y;
  if(x_mm>GRID_NAV_FIELD_MM || y_mm>GRID_NAV_FIELD_MM)return 0U;
  if(!dx || !dy)return GridNav_PlanStraightTargetMm(x_mm,y_mm);
  if(abs(dx)>150 && abs(dy)>150)return 0U;
  xs[0]=grid_nav_pose_mm_x;ys[0]=grid_nav_pose_mm_y;xs[2]=x_mm;ys[2]=y_mm;
  xs[1]=(abs(dx)<=abs(dy))?x_mm:xs[0];
  ys[1]=(abs(dx)<=abs(dy))?ys[0]:y_mm;
  for(j=0U;j<2U;j++) {
    int32_t lx=(int32_t)xs[j+1U]-xs[j],ly=(int32_t)ys[j+1U]-ys[j];
    lengths[j]=(uint16_t)(abs(lx)+abs(ly));n=(uint16_t)((lengths[j]+4U)/5U);
    if(oriented) {
      float target=atan2f((float)ly,(float)lx)*57.29578f;
      float gap=fabsf(remainderf(target-world_yaw,360.0f));
      if(gap>3.0f && !(j==0U && lengths[j]<=150U &&
          (fabsf(gap-90.0f)<=3.0f || fabsf(gap-180.0f)<=3.0f))) {
        if(!GridNav_RecenterSweep(xs[j],ys[j],world_yaw,target))return 0U;
        world_yaw=target;
      }
    }
    for(i=0U;i<=n;i++) {
      uint16_t x=(uint16_t)((int32_t)xs[j]+lx*i/n);
      uint16_t y=(uint16_t)((int32_t)ys[j]+ly*i/n);
      if(!GridNav_RotatedBodyFreeMm(x,y,oriented?
          (int16_t)lroundf(remainderf(world_yaw,360.0f)):0))return 0U;
    }
  }
  if(!oriented)for(angle=0;angle<=90;angle+=3)
    if(!GridNav_RotatedBodyFreeMm(xs[1],ys[1],(int16_t)angle))return 0U;
  grid_nav_scratch_path[0].x=GridNav_MmToCellX(xs[0]);
  grid_nav_scratch_path[0].y=GridNav_MmToCellY(ys[0]);
  for(j=0U;j<2U;j++) {
    int16_t cx=GridNav_MmToCellX(xs[j]),cy=GridNav_MmToCellY(ys[j]);
    int16_t tx=GridNav_MmToCellX(xs[j+1U]),ty=GridNav_MmToCellY(ys[j+1U]);
    int8_t stepx=(tx>cx)?1:((tx<cx)?-1:0),stepy=(ty>cy)?1:((ty<cy)?-1:0);
    while(cx!=tx || cy!=ty) {
      cx+=stepx;cy+=stepy;grid_nav_scratch_path[count].x=(uint8_t)cx;
      grid_nav_scratch_path[count++].y=(uint8_t)cy;
    }
  }
  GridNav_StorePlan(grid_nav_scratch_path,count,GridNav_MmToCellX(x_mm),
      GridNav_MmToCellY(y_mm),x_mm,y_mm);
  grid_nav_last_segment_count=2U;
  for(j=0U;j<2U;j++) {
    grid_nav_segment_t *seg=&grid_nav_last_segments[j];
    seg->from_x=GridNav_MmToCellX(xs[j]);seg->from_y=GridNav_MmToCellY(ys[j]);
    seg->to_x=GridNav_MmToCellX(xs[j+1U]);seg->to_y=GridNav_MmToCellY(ys[j+1U]);
    seg->dx=(xs[j+1U]>xs[j])?1:((xs[j+1U]<xs[j])?-1:0);
    seg->dy=(ys[j+1U]>ys[j])?-1:((ys[j+1U]<ys[j])?1:0);
    seg->length=(uint16_t)((lengths[j]+GRID_NAV_CELL_MM-1U)/GRID_NAV_CELL_MM);
    grid_nav_exact_distance_mm[j]=lengths[j];
  }
  return 1U;
}

uint8_t GridNav_PlanRecenterTargetMm(uint16_t x_mm,uint16_t y_mm)
{ return GridNav_RecenterPlan(x_mm,y_mm,0.0f,0U); }
uint8_t GridNav_PlanRecenterTargetMmHeading(uint16_t x_mm,uint16_t y_mm,float world_yaw)
{
  if(!isfinite(world_yaw))return 0U;
  return GridNav_RecenterPlan(x_mm,y_mm,world_yaw,1U);
}
static uint8_t GridNav_CheckedExactSegment(uint16_t index,int32_t x,int32_t y,int32_t tx,int32_t ty)
{
  grid_nav_segment_t *s;uint16_t distance=(uint16_t)(abs(tx-x)+abs(ty-y));
  if(index>=GRID_NAV_MAX_SEGMENTS || !distance || (tx!=x && ty!=y))return 0U;
  s=&grid_nav_last_segments[index];s->from_x=GridNav_MmToCellX((uint16_t)x);
  s->from_y=GridNav_MmToCellY((uint16_t)y);s->to_x=GridNav_MmToCellX((uint16_t)tx);
  s->to_y=GridNav_MmToCellY((uint16_t)ty);s->dx=tx>x?1:(tx<x?-1:0);s->dy=ty>y?-1:(ty<y?1:0);
  s->length=(uint16_t)((distance+GRID_NAV_CELL_MM-1U)/GRID_NAV_CELL_MM);
  grid_nav_exact_distance_mm[index]=distance;return 1U;
}
/* Only checked detours receive these exact distances. Ordinary A* remains
 * unchanged. Follow actual start on the first line; the final two lines
 * absorb the goal's cell-centre offsets, then validate the actual swept path. */
static uint8_t GridNav_CheckedExactEndpoints(uint16_t gx,uint16_t gy)
{
  int32_t x=grid_nav_pose_mm_x,y=grid_nav_pose_mm_y;uint16_t j,count=grid_nav_last_segment_count;
  if(!count) {
    if(x!=gx) {
      if(!GridNav_CheckedExactSegment(count++,x,y,gx,y))return 0U;x=gx;
    }
    if(y!=gy)if(!GridNav_CheckedExactSegment(count++,x,y,x,gy))return 0U;
    grid_nav_last_segment_count=count;return 1U;
  }
  for(j=0U;j<count;j++) {
    grid_nav_segment_t original=grid_nav_last_segments[j];int32_t tx=x,ty=y;
    if(original.dx)tx=(j+2U>=count)?gx:GridNav_CellCenterMmX(original.to_x);
    else ty=(j+2U>=count)?gy:GridNav_CellCenterMmY(original.to_y);
    if(!GridNav_CheckedExactSegment(j,x,y,tx,ty))return 0U;
    x=tx;y=ty;
  }
  /* A one-line cell path can still have a sub-cell transverse goal offset. */
  if(x!=gx || y!=gy) {
    if(!GridNav_CheckedExactSegment(count++,x,y,gx,gy))return 0U;
  }
  grid_nav_last_segment_count=count;return 1U;
}
uint8_t GridNav_PlanCheckedTargetMmHeading(uint16_t x_mm,uint16_t y_mm,float yaw)
{
  int32_t x=grid_nav_pose_mm_x,y=grid_nav_pose_mm_y;uint16_t j,i,n,d;uint8_t valid=1U;
  if(!isfinite(yaw) || !GridNav_RotatedBodyFreeMm(x_mm,y_mm,0) ||
      !GridNav_PlanTargetMm(x_mm,y_mm))return 0U;
  if(!GridNav_CheckedExactEndpoints(x_mm,y_mm)) {
    grid_nav_has_plan=0U;grid_nav_last_segment_count=0U;return 0U;
  }
  /* A* checks cell footprints. The executor begins at the real car centre,
   * so check that shifted line and every turn before committing the motion. */
  for(j=0U;j<grid_nav_last_segment_count && valid;j++) {
    grid_nav_segment_t *s=&grid_nav_last_segments[j];
    float target=atan2f((float)-s->dy,(float)s->dx)*57.29578f;
    float gap=fabsf(remainderf(target-yaw,360.0f));
    d=GridNav_GetSegmentDistanceMm(j);n=(uint16_t)((d+4U)/5U);
    if(!d || !n){valid=0U;break;}
    /* A short transverse step uses strafe and retains the current yaw.
     * Detours do not use post-alignment BACK; an opposite segment turns. */
    if(gap>3.0f && !(d<=150U && fabsf(gap-90.0f)<=3.0f)) {
      if(!GridNav_RecenterSweep((uint16_t)x,(uint16_t)y,yaw,target)){valid=0U;break;}
      yaw=target;
    }
    for(i=0U;i<=n;i++) {
      int32_t px=x+(int32_t)s->dx*d*i/n,py=y-(int32_t)s->dy*d*i/n;
      if(px<0 || py<0 || px>(int32_t)GRID_NAV_FIELD_MM || py>(int32_t)GRID_NAV_FIELD_MM ||
          !GridNav_RotatedBodyFreeMm((uint16_t)px,(uint16_t)py,(int16_t)lroundf(remainderf(yaw,360.0f)))) {
        valid=0U;break;
      }
    }
    x+=(int32_t)s->dx*d;y-=(int32_t)s->dy*d;
  }
  if(x!=x_mm || y!=y_mm)valid=0U;
  if(!valid){grid_nav_has_plan=0U;grid_nav_last_segment_count=0U;return 0U;}
  return 1U;
}

uint8_t GridNav_PlanWaypoint(const char *name)
{
  uint16_t x;
  uint16_t y;
  return (GridNav_FindWaypoint(name, &x, &y) != 0U) ?
           GridNav_PlanTargetMm(x, y) : 0U;
}

/* 解析 goto/path/pose/posemm/obs/free/clear/grid/map/waypoints 等命令。
   `goto` 支持三种写法：命名点位（raw/rough/temp/qr/h1/h2/center…）、
   格坐标（两个数都 <48）、mm 坐标（含 ≥48 的数，如 `goto 1200 2250`）。 */
uint8_t GridNav_ProcessCommand(const char *command)
{
  const char *cursor = command;
  char line[128];

  if (cursor == NULL)
  {
    return 0U;
  }
  while ((*cursor == ' ') || (*cursor == '\t') || (*cursor == '&'))
  {
    cursor++;
  }

  if ((strncmp(cursor, "goto", 4U) == 0) ||
      (strncmp(cursor, "gotomm", 6U) == 0))
  {
    uint16_t x_mm;
    uint16_t y_mm;
    const char *args = (cursor[4] == 'm') ? &cursor[6] : &cursor[4];

    if (GridNav_ResolveTarget(args, &x_mm, &y_mm) == 0U)
    {
      (void)snprintf(line, sizeof(line), "[NAV] bad args: '%s'\r\n", command);
      GridNav_Print(line);
      GridNav_Print("[NAV] usage: &goto raw|rough|temp|qr|h1|h2|center  "
                    "or  &goto <cell_x> <cell_y>  or  &goto <mm_x> <mm_y>\r\n");
      return 1U;
    }
    (void)GridNav_PlanAndStore(x_mm, y_mm, 1U);
    return 1U;
  }

  if (strncmp(cursor, "path", 4U) == 0)
  {
    uint16_t x_mm;
    uint16_t y_mm;

    if (GridNav_ResolveTarget(&cursor[4], &x_mm, &y_mm) == 0U)
    {
      GridNav_Print("[NAV] usage: &path <target>\r\n");
      return 1U;
    }
    (void)GridNav_PlanAndStore(x_mm, y_mm, 1U);
    return 1U;
  }

  if (strncmp(cursor, "posemm", 6U) == 0)
  {
    uint16_t x_mm;
    uint16_t y_mm;

    if (GridNav_ParsePair(&cursor[6], &x_mm, &y_mm) == 0U)
    {
      GridNav_Print("[NAV] usage: &posemm <mm_x> <mm_y>\r\n");
    }
    else
    {
      GridNav_SetPoseMm(x_mm, y_mm);
      (void)snprintf(line, sizeof(line), "[NAV] pose=(%umm,%umm) cell=(%u,%u)\r\n",
                     (unsigned int)x_mm, (unsigned int)y_mm,
                     (unsigned int)grid_nav_pose_x,
                     (unsigned int)grid_nav_pose_y);
      GridNav_Print(line);
    }
    return 1U;
  }

  if (strncmp(cursor, "pose", 4U) == 0)
  {
    uint16_t x;
    uint16_t y;

    if (GridNav_ParsePair(&cursor[4], &x, &y) == 0U)
    {
      GridNav_Print("[NAV] usage: &pose <cell_x> <cell_y> (0..47)\r\n");
    }
    else if ((x >= GRID_NAV_SIZE) || (y >= GRID_NAV_SIZE))
    {
      GridNav_Print("[NAV] out of range (0..47); use &posemm for mm\r\n");
    }
    else
    {
      GridNav_SetPoseMm(GridNav_CellCenterMmX((uint8_t)x),
                        GridNav_CellCenterMmY((uint8_t)y));
      (void)snprintf(line, sizeof(line), "[NAV] pose=(%u,%u)cell = (%umm,%umm)\r\n",
                     (unsigned int)x, (unsigned int)y,
                     (unsigned int)GridNav_CellCenterMmX((uint8_t)x),
                     (unsigned int)GridNav_CellCenterMmY((uint8_t)y));
      GridNav_Print(line);
    }
    return 1U;
  }

  if ((strncmp(cursor, "obs", 3U) == 0) ||
      (strncmp(cursor, "free", 4U) == 0))
  {
    uint8_t set_blocked = (uint8_t)(strncmp(cursor, "obs", 3U) == 0);
    uint16_t a;
    uint16_t b;
    uint16_t x;
    uint16_t y;
    uint8_t ok = 1U;

    if (GridNav_ParsePair(&cursor[(set_blocked != 0U) ? 3 : 4], &a, &b) == 0U)
    {
      ok = 0U;
    }
    else if ((a < GRID_NAV_SIZE) && (b < GRID_NAV_SIZE))
    {
      /* 两个值都 <48 → 当**格**坐标（与 &goto/&pose 同一套规则） */
      x = a;
      y = b;
    }
    else if ((a <= GRID_NAV_FIELD_MM) && (b <= GRID_NAV_FIELD_MM))
    {
      /* 否则当**规则图 mm**（原点左下、y 向上）—— 现场照图纸读数直接填 */
      x = GridNav_MmToCellX(a);
      y = GridNav_MmToCellY(b);
    }
    else
    {
      ok = 0U;
    }

    if (ok == 0U)
    {
      GridNav_Print("[NAV] usage: &obs x y / &free x y"
                    "  (both <48 = cell coords, else rule-book mm)\r\n");
    }
    else
    {
      GridNav_SetObstacle((uint8_t)x, (uint8_t)y, set_blocked);
      (void)snprintf(line, sizeof(line), "[NAV] %s=(%u,%u)cell (%umm,%umm)\r\n",
                     (set_blocked != 0U) ? "obstacle" : "free",
                     (unsigned int)x, (unsigned int)y,
                     (unsigned int)GridNav_CellCenterMmX((uint8_t)x),
                     (unsigned int)GridNav_CellCenterMmY((uint8_t)y));
      GridNav_Print(line);
      if (set_blocked != 0U)
      {
        /* 提醒一次就够：车体280×280，一个障碍格会让**周围 3 格内**的格心全部不可用。 */
        GridNav_Print("[NAV] note: car body is 280mm, so this blocks a 7x7 area of"
                      " car-centre positions\r\n");
      }
    }
    return 1U;
  }

  if ((strcmp(cursor, "clear") == 0) || (strcmp(cursor, "gridclear") == 0))
  {
    memset(grid_nav_obstacle, 0, sizeof(grid_nav_obstacle));
    GridNav_RebuildWalkable();
    grid_nav_has_plan = 0U;
    grid_nav_last_path_length = 0U;
    grid_nav_last_segment_count = 0U;
    grid_nav_revision++;
    GridNav_Print("[NAV] runtime obstacles cleared (yellow blocks stay)\r\n");
    return 1U;
  }

  if ((strcmp(cursor, "grid") == 0) || (strcmp(cursor, "map") == 0))
  {
    GridNav_PrintMap();
    return 1U;
  }

  if ((strcmp(cursor, "waypoints") == 0) || (strcmp(cursor, "wp") == 0))
  {
    uint8_t i;

    GridNav_Print("[NAV] named waypoints (mm, origin=left-bottom, y up):\r\n");
    for (i = 0U; i < (uint8_t)(sizeof(grid_nav_waypoints) /
                               sizeof(grid_nav_waypoints[0])); ++i)
    {
      (void)snprintf(line, sizeof(line), "  %-7s (%u,%u)\r\n",
                     grid_nav_waypoints[i].name,
                     (unsigned int)grid_nav_waypoints[i].x,
                     (unsigned int)grid_nav_waypoints[i].y);
      GridNav_Print(line);
    }
    return 1U;
  }

  return 0U;
}

void GridNav_Init(void)
{
  memset(grid_nav_obstacle, 0, sizeof(grid_nav_obstacle));
  memset(grid_nav_last_path, 0, sizeof(grid_nav_last_path));
  memset(grid_nav_last_segments, 0, sizeof(grid_nav_last_segments));
  GridNav_BuildFieldMap();
  GridNav_RebuildWalkable();

  grid_nav_last_path_length = 0U;
  grid_nav_last_segment_count = 0U;
  grid_nav_last_target_x = 0U;
  memset(grid_nav_exact_distance_mm,0,sizeof(grid_nav_exact_distance_mm));
  grid_nav_last_target_y = 0U;
  grid_nav_has_plan = 0U;
  /* 最底层的默认摆位（右下角区中心）。注意它**不是**最终默认值：ZDT_Motor_Begin() 紧接着
     会调 ZDT_Script_ApplyDefaultZone()，按 ZDT_START_ZONE_DEFAULT 覆盖成本队实际发车区。
     车头朝向同样由那里按区设定。 */
  GridNav_SetPoseMm(2250U, 150U);
  grid_nav_last_target_mm_x = 2250U;
  grid_nav_last_target_mm_y = 2250U;
  grid_nav_revision++;
  GridNav_ClearNodes();
}
