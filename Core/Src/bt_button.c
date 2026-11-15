#include "bt_button.h"

#include <stddef.h>
#include <string.h>

#include "stm32h7xx_hal.h"

#define BT_BUTTON_QUEUE_SIZE          (16U)
#define BT_BUTTON_TOKEN_MAX_LEN       (64U)
#define BT_BUTTON_ANGLE_WAIT_MS       (15U)
#define BT_BUTTON_SHORT_WAIT_MS       (25U)
#define BT_BUTTON_GRID_WAIT_MS        (300U)

typedef struct
{
  const char *text;
  bt_button_id_t id;
} bt_button_command_t;

static const bt_button_command_t bt_button_commands[] =
{
  { "&en",   BT_BUTTON_ENABLE   },
  { "&enable", BT_BUTTON_ENABLE },
  { "&m1",   BT_BUTTON_M1       },
  { "&m2",   BT_BUTTON_M2       },
  { "&u",    BT_BUTTON_FORWARD  },
  { "&l",    BT_BUTTON_LEFT     },
  { "&d",    BT_BUTTON_BACKWARD },
  { "&r",    BT_BUTTON_RIGHT    },
  { "&stop", BT_BUTTON_STOP     }
};

static bt_button_event_t bt_button_queue[BT_BUTTON_QUEUE_SIZE];
static volatile uint8_t bt_button_head;
static volatile uint8_t bt_button_tail;
static bt_button_stats_t bt_button_stats;

static char bt_button_token[BT_BUTTON_TOKEN_MAX_LEN];
static uint8_t bt_button_token_len;
static uint8_t bt_button_token_invalid;
static uint8_t bt_button_suppress_line_end;
static uint32_t bt_button_pending_tick;

static uint8_t BT_Button_ToLower(uint8_t c)
{
  if ((c >= (uint8_t)'A') && (c <= (uint8_t)'Z'))
  {
    return (uint8_t)(c + ((uint8_t)'a' - (uint8_t)'A'));
  }
  return c;
}

static uint8_t BT_Button_IsDigit(uint8_t c)
{
  return (uint8_t)((c >= (uint8_t)'0') && (c <= (uint8_t)'9'));
}

static uint8_t BT_Button_IsAsciiTokenChar(uint8_t c)
{
  return (uint8_t)(BT_Button_IsDigit(c) ||
                   ((c >= (uint8_t)'A') && (c <= (uint8_t)'Z')) ||
                   ((c >= (uint8_t)'a') && (c <= (uint8_t)'z')));
}

static uint8_t BT_Button_IsTerminator(uint8_t c)
{
  return (uint8_t)((c == (uint8_t)'\r') || (c == (uint8_t)'\n') ||
                   (c == (uint8_t)' ') || (c == (uint8_t)'\t') ||
                   (c == (uint8_t)';') || (c == (uint8_t)','));
}

/* 网格命令含空格/逗号，不能按普通按钮 token 在分隔符处提前结束。 */
static uint8_t BT_Button_StartsWith(const char *token, uint8_t length,
                                    const char *prefix)
{
  size_t prefix_length = strlen(prefix);

  if ((size_t)length <= prefix_length)
  {
    return (uint8_t)(memcmp(token, prefix, (size_t)length) == 0);
  }
  return (uint8_t)(memcmp(token, prefix, prefix_length) == 0);
}

/* 判断是否为 &goto/&pose/&obs 等完整网格命令。
 * ⚠ 2026-09-13：前缀后面**必须允许字母**，否则命名点位（&goto raw / &goto exit1）
 * 会被判非法 token 静默丢弃（现场症状："&goto raw 完全没反应"，而 &goto 24 3 正常）。 */
static uint8_t BT_Button_IsGridCommand(const char *token, uint8_t length)
{
  static const char *commands[] = { "&goto", "&path", "&pose", "&obs",
                                    "&free", "&clear", "&gridclear",
                                    "&grid", "&map" };
  uint8_t i;

  for (i = 0U; i < (uint8_t)(sizeof(commands) / sizeof(commands[0])); ++i)
  {
    size_t prefix_length = strlen(commands[i]);
    if ((length >= prefix_length) &&
        (memcmp(token, commands[i], prefix_length) == 0))
    {
      uint8_t j;
      for (j = (uint8_t)prefix_length; j < length; ++j)
      {
        char c = token[j];
        if (!(((c >= '0') && (c <= '9')) || ((c >= 'a') && (c <= 'z')) ||
              (c == ' ') || (c == '\t') ||
              (c == ',') || (c == '-') || (c == '+')))
        {
          return 0U;
        }
      }
      return 1U;
    }
  }
  return 0U;
}

/* 判断当前 token 是否仍可能是网格命令前缀，供逐字节输入时使用。 */
static uint8_t BT_Button_IsGridPrefix(const char *token, uint8_t length)
{
  static const char *commands[] = { "&goto", "&path", "&pose", "&obs",
                                    "&free", "&clear", "&gridclear",
                                    "&grid", "&map" };
  uint8_t i;

  for (i = 0U; i < (uint8_t)(sizeof(commands) / sizeof(commands[0])); ++i)
  {
    if (BT_Button_StartsWith(token, length, commands[i]))
    {
      return 1U;
    }
  }
  return 0U;
}

static bt_button_id_t BT_Button_FindExact(const char *token, uint8_t length)
{
  size_t i;

  for (i = 0U; i < (sizeof(bt_button_commands) / sizeof(bt_button_commands[0])); ++i)
  {
    if ((strlen(bt_button_commands[i].text) == (size_t)length) &&
        (memcmp(bt_button_commands[i].text, token, (size_t)length) == 0))
    {
      return bt_button_commands[i].id;
    }
  }
  return BT_BUTTON_NONE;
}

static uint8_t BT_Button_IsPrefix(const char *token, uint8_t length)
{
  size_t i;

  for (i = 0U; i < (sizeof(bt_button_commands) / sizeof(bt_button_commands[0])); ++i)
  {
    if ((size_t)length <= strlen(bt_button_commands[i].text))
    {
      if (memcmp(bt_button_commands[i].text, token, (size_t)length) == 0)
      {
        return 1U;
      }
    }
  }
  return 0U;
}

static uint8_t BT_Button_IsPrefixOf(const char *token, uint8_t length,
                                      const char *command)
{
  return (uint8_t)((length <= strlen(command)) &&
                   (memcmp(token, command, length) == 0));
}

static uint8_t BT_Button_IsCustomPrefix(const char *token, uint8_t length)
{
  uint8_t i;

  if (BT_Button_IsGridPrefix(token, length) ||
      BT_Button_IsPrefixOf(token, length, "&save") ||
      BT_Button_IsPrefixOf(token, length, "&read") ||
      BT_Button_IsPrefixOf(token, length, "&echo") ||
      BT_Button_IsPrefixOf(token, length, "&depart") ||
      BT_Button_IsPrefixOf(token, length, "&prepare") ||
      BT_Button_IsPrefixOf(token, length, "&ok") ||
      BT_Button_IsPrefixOf(token, length, "&match") ||
      BT_Button_IsPrefixOf(token, length, "&camraw") ||
      BT_Button_IsPrefixOf(token, length, "&camstat") ||
      BT_Button_IsPrefixOf(token, length, "&logstat") ||
      BT_Button_IsPrefixOf(token, length, "&arm") ||
      BT_Button_IsPrefixOf(token, length, "&zdt") ||
      BT_Button_IsPrefixOf(token, length, "&qrscan") ||
      BT_Button_IsPrefixOf(token, length, "&valign") ||
      BT_Button_IsPrefixOf(token, length, "&vyaw") ||
      BT_Button_IsPrefixOf(token, length, "&outerarc") ||
      BT_Button_IsPrefixOf(token, length, "&vsource") ||
      BT_Button_IsPrefixOf(token, length, "&vgain") ||
      BT_Button_IsPrefixOf(token, length, "&vpid") ||
      BT_Button_IsPrefixOf(token, length, "&vtune") ||
      BT_Button_IsPrefixOf(token, length, "&zero") ||
      BT_Button_IsPrefixOf(token, length, "&nav") ||
      BT_Button_IsPrefixOf(token, length, "&dis") ||
      BT_Button_IsPrefixOf(token, length, "&off") ||
      BT_Button_IsPrefixOf(token, length, "&spd") ||
      BT_Button_IsPrefixOf(token, length, "&dcomp") ||
      BT_Button_IsPrefixOf(token, length, "&endturn") ||
      BT_Button_IsPrefixOf(token, length, "&t1000") ||
      BT_Button_IsPrefixOf(token, length, "&t90") ||
      BT_Button_IsPrefixOf(token, length, "&t180") ||
      BT_Button_IsPrefixOf(token, length, "&tarc") ||
      BT_Button_IsPrefixOf(token, length, "&aspd") ||
      BT_Button_IsPrefixOf(token, length, "&cnt") ||
      BT_Button_IsPrefixOf(token, length, "&posref") ||
      BT_Button_IsPrefixOf(token, length, "&posd") ||
      BT_Button_IsPrefixOf(token, length, "&posq") ||
      BT_Button_IsPrefixOf(token, length, "&poscal") ||
      BT_Button_IsPrefixOf(token, length, "&posmode") ||
      BT_Button_IsPrefixOf(token, length, "&verall") ||
      BT_Button_IsPrefixOf(token, length, "&show") ||
      BT_Button_IsPrefixOf(token, length, "&flash") ||
      BT_Button_IsPrefixOf(token, length, "&startpose") ||
      BT_Button_IsPrefixOf(token, length, "&startzone") ||
      BT_Button_IsPrefixOf(token, length, "&cross") ||
      BT_Button_IsPrefixOf(token, length, "&leave") ||
      BT_Button_IsPrefixOf(token, length, "&nudge") ||
      BT_Button_IsPrefixOf(token, length, "&home") ||
      BT_Button_IsPrefixOf(token, length, "&waypoints") ||
      BT_Button_IsPrefixOf(token, length, "&wp") ||
      BT_Button_IsPrefixOf(token, length, "&posemm") ||
      BT_Button_IsPrefixOf(token, length, "&gotomm") ||
      BT_Button_IsPrefixOf(token, length, "&kp") ||
      BT_Button_IsPrefixOf(token, length, "&ki") ||
      BT_Button_IsPrefixOf(token, length, "&kd") ||
      /* V234 转向 PID 调参与波形：&pid 查询当前值、&tstat 状态、&ttrace 波形开关。
       * 三处（IsCustomPrefix / IsCustomToken / IsArgCommand）成对登记，
       * 否则 token 能活到 flush 却被静默丢弃——见下面 &dm 的同类注释。 */
      BT_Button_IsPrefixOf(token, length, "&pid") ||
      BT_Button_IsPrefixOf(token, length, "&tstat") ||
      BT_Button_IsPrefixOf(token, length, "&ttrace") ||
      /* 2026-09-14：达妙转盘（&dm ...）。两级闸门都要登记 ——
       * 只登记这一处的话，token 能活到 flush 但会被 IsCustomToken 丢掉，
       * 现场表现是"命令发出去了、什么都不回"。 */
      BT_Button_IsPrefixOf(token, length, "&dm"))
  {
    return 1U;
  }

  if ((length < 3U) ||
      ((strncmp(token, "&kp", 3U) != 0) &&
       (strncmp(token, "&ki", 3U) != 0) &&
       (strncmp(token, "&kd", 3U) != 0)))
  {
    return 0U;
  }

  for (i = 3U; i < length; ++i)
  {
    char c = token[i];
    if (!((c >= '0' && c <= '9') || (c == '.') || (c == '-') || (c == '+')))
    {
      return 0U;
    }
  }
  return 1U;
}

static uint8_t BT_Button_IsCustomToken(const char *token, uint8_t length)
{
  uint8_t i;

  if (BT_Button_IsGridCommand(token, length) ||
      (strcmp(token, "&save") == 0) || (strcmp(token, "&read") == 0) ||
      (strcmp(token, "&echo") == 0) ||
      (strcmp(token, "&depart") == 0) ||
      BT_Button_StartsWith(token, length, "&ok") ||
      BT_Button_StartsWith(token, length, "&match") ||
      BT_Button_StartsWith(token, length, "&camraw") ||
      BT_Button_StartsWith(token, length, "&camstat") ||
      (strcmp(token,"&logstat")==0) ||
      BT_Button_StartsWith(token, length, "&arm") ||
      BT_Button_StartsWith(token, length, "&zdt") ||
      BT_Button_StartsWith(token, length, "&qrscan") ||
      BT_Button_StartsWith(token, length, "&valign") ||
      BT_Button_StartsWith(token, length, "&vyaw") ||
      BT_Button_StartsWith(token, length, "&outerarc") ||
      BT_Button_StartsWith(token, length, "&motion") ||
      BT_Button_StartsWith(token, length, "&vsource") ||
      BT_Button_StartsWith(token, length, "&vgain") ||
      BT_Button_StartsWith(token, length, "&vpid") ||
      BT_Button_StartsWith(token, length, "&vtune") ||
      (strcmp(token, "&zero") == 0) ||
      BT_Button_StartsWith(token, length, "&nav") ||
      BT_Button_StartsWith(token, length, "&dis") ||
      BT_Button_StartsWith(token, length, "&off") ||
      BT_Button_StartsWith(token, length, "&spd") ||
      BT_Button_StartsWith(token, length, "&dcomp") ||
      BT_Button_StartsWith(token, length, "&endturn") ||
      BT_Button_StartsWith(token, length, "&t1000") ||
      BT_Button_StartsWith(token, length, "&t90") ||
      BT_Button_StartsWith(token, length, "&t180") ||
      BT_Button_StartsWith(token, length, "&tarc") ||
      BT_Button_StartsWith(token, length, "&aspd") ||
      BT_Button_StartsWith(token, length, "&cnt") ||
      BT_Button_StartsWith(token, length, "&posref") ||
      BT_Button_StartsWith(token, length, "&posd") ||
      BT_Button_StartsWith(token, length, "&posq") ||
      BT_Button_StartsWith(token, length, "&poscal") ||
      BT_Button_StartsWith(token, length, "&posmode") ||
      BT_Button_StartsWith(token, length, "&verall") ||
      BT_Button_StartsWith(token, length, "&show") ||
      BT_Button_StartsWith(token, length, "&flash") ||
      BT_Button_StartsWith(token, length, "&prepare") ||
      BT_Button_StartsWith(token, length, "&startpose") ||
      BT_Button_StartsWith(token, length, "&startzone") ||
      BT_Button_StartsWith(token, length, "&cross") ||
      BT_Button_StartsWith(token, length, "&leave") ||
      BT_Button_StartsWith(token, length, "&nudge") ||
      BT_Button_StartsWith(token, length, "&home") ||
      BT_Button_StartsWith(token, length, "&waypoints") ||
      BT_Button_StartsWith(token, length, "&wp") ||
      BT_Button_StartsWith(token, length, "&posemm") ||
      BT_Button_StartsWith(token, length, "&gotomm") ||
      /* V234：与 IsCustomPrefix 成对登记（&pid/&tstat/&ttrace） */
      BT_Button_StartsWith(token, length, "&pid") ||
      BT_Button_StartsWith(token, length, "&tstat") ||
      BT_Button_StartsWith(token, length, "&ttrace") ||
      /* 2026-09-14：达妙转盘（&dm ...）。与 IsCustomPrefix 必须成对登记，
       * 否则就是上面注释里说的"活着进 flush、被静默丢掉"。 */
      BT_Button_StartsWith(token, length, "&dm"))
  {
    return 1U;
  }

  if ((length <= 3U) ||
      ((strncmp(token, "&kp", 3U) != 0) &&
       (strncmp(token, "&ki", 3U) != 0) &&
       (strncmp(token, "&kd", 3U) != 0)))
  {
    return 0U;
  }

  for (i = 3U; i < length; ++i)
  {
    char c = token[i];
    if (!((c >= '0' && c <= '9') || (c == '.') || (c == '-') || (c == '+')))
    {
      return 0U;
    }
  }
  return 1U;
}

static uint8_t BT_Button_IsShortAnglePrefix(const char *token, uint8_t length)
{
  return (uint8_t)((length == 2U) && (token[0] == '&') &&
                   ((token[1] == 'l') || (token[1] == 'r')));
}

static uint8_t BT_Button_IsAngleToken(const char *token, uint8_t length,
                                      int16_t *angle_out)
{
  uint16_t angle = 0U;
  uint8_t i;

  if ((length < 3U) || (length > 5U) || (token[0] != '&') ||
      ((token[1] != 'l') && (token[1] != 'r')))
  {
    return 0U;
  }

  for (i = 2U; i < length; ++i)
  {
    if (!BT_Button_IsDigit((uint8_t)token[i]))
    {
      return 0U;
    }
    angle = (uint16_t)((angle * 10U) + (uint16_t)(token[i] - '0'));
  }

  if ((angle == 0U) || (angle > 360U))
  {
    return 0U;
  }

  if (angle_out != NULL)
  {
    *angle_out = (token[1] == 'l') ? -(int16_t)angle : (int16_t)angle;
  }
  return 1U;
}

static uint8_t BT_Button_Push(bt_button_id_t id, int16_t angle_deg)
{
  bt_button_event_t event;
  uint8_t next;

  bt_button_stats.valid_count++;
  event.id = id;
  event.angle_deg = angle_deg;
  event.text[0] = '\0';
  event.sequence = bt_button_stats.valid_count;
  event.tick_ms = HAL_GetTick();

  next = (uint8_t)((bt_button_head + 1U) % BT_BUTTON_QUEUE_SIZE);
  if (next == bt_button_tail)
  {
    bt_button_stats.dropped_count++;
    return 0U;
  }

  bt_button_queue[bt_button_head] = event;
  __DMB();
  bt_button_head = next;
  return 1U;
}

static uint8_t BT_Button_PushText(const char *text, uint32_t received_tick)
{
  bt_button_event_t event;
  uint8_t next;

  bt_button_stats.valid_count++;
  event.id = BT_BUTTON_CUSTOM;
  event.angle_deg = 0;
  (void)strncpy(event.text, text, sizeof(event.text) - 1U);
  event.text[sizeof(event.text) - 1U] = '\0';
  event.sequence = bt_button_stats.valid_count;
  event.tick_ms = received_tick;

  next = (uint8_t)((bt_button_head + 1U) % BT_BUTTON_QUEUE_SIZE);
  if (next == bt_button_tail)
  {
    bt_button_stats.dropped_count++;
    return 0U;
  }

  bt_button_queue[bt_button_head] = event;
  __DMB();
  bt_button_head = next;
  return 1U;
}

static void BT_Button_ResetToken(void)
{
  bt_button_token_len = 0U;
  bt_button_token_invalid = 0U;
  bt_button_pending_tick = 0U;
}

/* 数值型命令（&nav / &spd / &t1000 / &cnt）允许 token 里带空格，现场习惯写
 * "&t1000 5000"、"&spd 0.25"；空格保留在 token 内交给上层解析。 */
static uint8_t BT_Button_IsValuePrefix(const char *token, uint8_t length)
{
  return (uint8_t)(BT_Button_StartsWith(token, length, "&nav") ||
                   BT_Button_StartsWith(token, length, "&spd") ||
                   BT_Button_StartsWith(token, length, "&dcomp") ||
                   BT_Button_StartsWith(token, length, "&endturn") ||
                   BT_Button_StartsWith(token, length, "&t1000") ||
                   BT_Button_StartsWith(token, length, "&cnt"));
}

/* 带参数的自定义命令：token 内部**允许空格**，并且用较长空闲超时（300 ms）派发。
 * 覆盖：网格命令（&goto raw / &goto 1200 2250）、数值命令（&spd 0.5）、
 *       &startpose 1|2、&nudge u 300、&posemm 1200 2250、&gotomm、&dm。
 * 不含这些的话，现场写 "&startpose 2" 会被空格切成 "&startpose"（2 被丢掉），
 * "&nudge u 300" 同理 —— 表现都是"命令发出去了但参数没生效"。
 *
 * 2026-09-14 加 &dm：达妙转盘命令（`&dm enable` / `&dm pos 0.5`）必须在这里，
 * 否则那个空格会把 token 截断成 `&dm`，参数被丢掉。
 *
 * ⚠ 长度上限：bluetooth 事件文本缓冲 bt_button_event_t.text[] 只有 16 字节
 * （见 bt_button.h），PushText 又剥掉前导 '&'，所以**经蓝牙发送的载荷最多 15 字符**。
 * `dm status`(9) / `dm disable`(10) / `dm pos 0.5`(10) 都装得下；
 * `dm mit 0 0 100 1 0`(18)、`dm zero confirm`(15，刚好) 要留意，
 * 超长的（如 dm mit）请走有线控制台，或以后改成结构化短命令。 */
static uint8_t BT_Button_IsArgCommand(const char *token, uint8_t length)
{
  return (uint8_t)(BT_Button_IsGridPrefix(token, length) ||
                   BT_Button_StartsWith(token, length, "&depart") ||
                   BT_Button_StartsWith(token, length, "&ok") ||
                   BT_Button_StartsWith(token, length, "&match") ||
                   BT_Button_StartsWith(token, length, "&camraw") ||
                   BT_Button_StartsWith(token, length, "&camstat") ||
      BT_Button_StartsWith(token, length, "&arm") ||
                   BT_Button_StartsWith(token, length, "&zdt") ||
                   BT_Button_StartsWith(token, length, "&qrscan") ||
      BT_Button_StartsWith(token, length, "&valign") ||
      BT_Button_StartsWith(token, length, "&vyaw") ||
      BT_Button_StartsWith(token, length, "&outerarc") ||
      BT_Button_StartsWith(token, length, "&motion") ||
                   BT_Button_StartsWith(token, length, "&vsource") ||
                   BT_Button_StartsWith(token, length, "&vgain") ||
                   BT_Button_StartsWith(token, length, "&vpid") ||
                   BT_Button_StartsWith(token, length, "&vtune") ||
                   BT_Button_IsValuePrefix(token, length) ||
                   BT_Button_StartsWith(token, length, "&prepare") ||
      BT_Button_StartsWith(token, length, "&startpose") ||
                   BT_Button_StartsWith(token, length, "&startzone") ||
                   BT_Button_StartsWith(token, length, "&nudge") ||
                   BT_Button_StartsWith(token, length, "&posemm") ||
                   BT_Button_StartsWith(token, length, "&gotomm") ||
                   /* V234：&ttrace 1|0 带空格，必须在这里登记，否则空格会把参数切掉 */
                   BT_Button_StartsWith(token, length, "&ttrace") ||
                   BT_Button_StartsWith(token, length, "&dm"));
}

static void BT_Button_FlushToken(void)
{
  bt_button_id_t id;
  int16_t angle = 0;

  if (bt_button_token_len == 0U)
  {
    return;
  }

  /* Exact buttons win over partial custom prefixes: &en is also a prefix
   * of &endturn, but a completed &en must enable, not dispatch text "en". */
  id = BT_Button_FindExact(bt_button_token, bt_button_token_len);
  if (!bt_button_token_invalid && id != BT_BUTTON_NONE)
  {
    (void)BT_Button_Push(id, 0);
  }
  else if (!bt_button_token_invalid &&
      BT_Button_IsCustomToken(bt_button_token, bt_button_token_len))
  {
    (void)BT_Button_PushText(&bt_button_token[1], bt_button_pending_tick);
  }
  else if (!bt_button_token_invalid &&
      BT_Button_IsAngleToken(bt_button_token, bt_button_token_len, &angle))
  {
    id = (angle < 0) ? BT_BUTTON_ROTATE_LEFT : BT_BUTTON_ROTATE_RIGHT;
    (void)BT_Button_Push(id, angle);
  }
  else if (!bt_button_token_invalid &&
           BT_Button_IsShortAnglePrefix(bt_button_token, bt_button_token_len))
  {
    id = (bt_button_token[1] == 'l') ? BT_BUTTON_LEFT : BT_BUTTON_RIGHT;
    (void)BT_Button_Push(id, 0);
  }
  else
  {
    id = BT_Button_FindExact(bt_button_token, bt_button_token_len);
    if (id != BT_BUTTON_NONE)
    {
      (void)BT_Button_Push(id, 0);
    }
    else
    {
      bt_button_stats.invalid_count++;
    }
  }

  BT_Button_ResetToken();
  bt_button_suppress_line_end = 2U;
}

void BT_Button_Init(void)
{
  memset(bt_button_queue, 0, sizeof(bt_button_queue));
  memset(&bt_button_stats, 0, sizeof(bt_button_stats));
  bt_button_head = 0U;
  bt_button_tail = 0U;
  BT_Button_ResetToken();
  bt_button_suppress_line_end = 0U;
}

/* 这条 token 需要等多久才判定"命令发完了"，返回 0 表示不靠超时派发。
 *
 * 现场蓝牙 App 按一次"发送"往往**只发内容、不带 \r\n**，所以任何"必须等到换行
 * 才解析"的写法都会把命令憋在缓冲区里，直到下一次发别的内容时先收到 '&' 才补发
 * 上一条 —— 表现就是"要发两次、而且收到的总是上一条"。因此这里统一用空闲超时兜底：
 *   - 网格命令（&goto/&pose/&obs/&path/...）和数值命令（&nav/&spd/&t1000/&cnt）
 *     内部允许空格，不能用短超时切碎，用 BT_BUTTON_GRID_WAIT_MS = 300 ms
 *     （远大于一次连续发送内的字节间隔，远小于两次人工按下之间的间隔）；
 *   - 角度命令 &l90/&r90 用 15 ms，短前缀 &l/&r 用 25 ms（原行为）；
 *   - 普通按钮命令（&en/&u/&stop/...）在最后一个字节精确匹配时就立即派发，
 *     不在这里处理。 */
static uint32_t BT_Button_FlushDelayMs(const char *token, uint8_t length)
{
  if (BT_Button_IsArgCommand(token, length))
  {
    return BT_BUTTON_GRID_WAIT_MS;
  }
  if (BT_Button_IsShortAnglePrefix(token, length))
  {
    return BT_BUTTON_SHORT_WAIT_MS;
  }
  if (BT_Button_IsCustomToken(token, length) ||
      BT_Button_IsCustomPrefix(token, length) ||
      BT_Button_IsAngleToken(token, length, NULL) ||
      (bt_button_token_invalid != 0U))
  {
    return BT_BUTTON_ANGLE_WAIT_MS;
  }
  return 0U;
}

void BT_Button_Poll(void)
{
  uint32_t delay_ms;

  if (bt_button_token_len == 0U)
  {
    return;
  }

  delay_ms = BT_Button_FlushDelayMs(bt_button_token, bt_button_token_len);
  if ((delay_ms != 0U) &&
      ((HAL_GetTick() - bt_button_pending_tick) >= delay_ms))
  {
    BT_Button_FlushToken();
  }
}

uint8_t BT_Button_ProcessCommand(const char *command)
{
  char token[BT_BUTTON_TOKEN_MAX_LEN];
  const char *cursor = command;
  uint8_t length = 0U;
  bt_button_id_t id;
  int16_t angle = 0;

  if (command == NULL)
  {
    return 0U;
  }
  while ((*cursor == ' ') || (*cursor == '\t'))
  {
    cursor++;
  }
  if (*cursor != '&')
  {
    return 0U;
  }
  token[length++] = '&';
  cursor++;

  while ((*cursor != '\0') && (length < (BT_BUTTON_TOKEN_MAX_LEN - 1U)))
  {
    uint8_t c = (uint8_t)*cursor++;
    if (BT_Button_IsTerminator(c) && !((c == ' ') || (c == '\t')))
    {
      break;
    }
    if (!BT_Button_IsAsciiTokenChar(c) &&
        !((c == ' ') || (c == '\t') || (c == '.') || (c == ',') ||
          (c == '-') || (c == '+')))
    {
      bt_button_stats.invalid_count++;
      return 0U;
    }
    token[length++] = (char)BT_Button_ToLower(c);
  }
  token[length] = '\0';
  while ((length > 0U) && ((token[length - 1U] == ' ') ||
                           (token[length - 1U] == '\t')))
  {
    token[--length] = '\0';
  }

  id = BT_Button_FindExact(token, length);
  if (id != BT_BUTTON_NONE)
  {
    (void)BT_Button_Push(id, 0);
    return 1U;
  }
  if (BT_Button_IsCustomToken(token, length))
  {
    (void)BT_Button_PushText(&token[1], HAL_GetTick());
    return 1U;
  }

  if (BT_Button_IsAngleToken(token, length, &angle))
  {
    id = (angle < 0) ? BT_BUTTON_ROTATE_LEFT : BT_BUTTON_ROTATE_RIGHT;
    (void)BT_Button_Push(id, angle);
    return 1U;
  }

  id = BT_Button_FindExact(token, length);
  if (id == BT_BUTTON_NONE)
  {
    bt_button_stats.invalid_count++;
    return 0U;
  }
  (void)BT_Button_Push(id, 0);
  return 1U;
}

uint8_t BT_Button_ProcessByte(uint8_t byte)
{
  uint8_t lower;
  bt_button_id_t id;

  if (bt_button_token_len == 0U)
  {
    if (bt_button_suppress_line_end != 0U)
    {
      if ((byte == (uint8_t)'\r') || (byte == (uint8_t)'\n'))
      {
        bt_button_suppress_line_end--;
        return 1U;
      }
      if ((byte == (uint8_t)' ') || (byte == (uint8_t)'\t'))
      {
        return 1U;
      }
      bt_button_suppress_line_end = 0U;
    }
    if (byte != (uint8_t)'&')
    {
      return 0U;
    }
    bt_button_token[0] = '&';
    bt_button_token_len = 1U;
    bt_button_token_invalid = 0U;
    bt_button_pending_tick = HAL_GetTick();
    return 1U;
  }

  if (byte == (uint8_t)'&')
  {
    BT_Button_FlushToken();
    bt_button_token[0] = '&';
    bt_button_token_len = 1U;
    bt_button_token_invalid = 0U;
    bt_button_pending_tick = HAL_GetTick();
    return 1U;
  }

  if (BT_Button_IsTerminator(byte))
  {
    /* 带参数的命令里的空格和逗号保留在 token 内，等待换行或空闲超时后一次解析。 */
    if (((byte == (uint8_t)' ') || (byte == (uint8_t)'\t') ||
         (byte == (uint8_t)',')) &&
        (BT_Button_IsArgCommand(bt_button_token, bt_button_token_len) != 0U))
    {
      if (bt_button_token_len < (BT_BUTTON_TOKEN_MAX_LEN - 1U))
      {
        bt_button_token[bt_button_token_len++] =
            (byte == (uint8_t)',') ? ',' : ' ';
        bt_button_token[bt_button_token_len] = '\0';
        bt_button_pending_tick = HAL_GetTick();
      }
      return 1U;
    }
    BT_Button_FlushToken();
    bt_button_suppress_line_end = 1U;
    return 1U;
  }

  lower = BT_Button_ToLower(byte);
  {
    /* 允许的字符：token 字符本体、小数点/逗号/正负号；
       另外"带参数命令"内部的空格也允许（不能在这里判成非法）。 */
    uint8_t allowed = BT_Button_IsAsciiTokenChar(lower);
    uint8_t space_ok = 0U;

    if ((lower == '.') || (lower == ',') || (lower == '-') ||
        (lower == '+'))
    {
      allowed = 1U;
    }
    else if (lower == ' ')
    {
      space_ok = BT_Button_IsArgCommand(bt_button_token,
                                        bt_button_token_len);
      allowed = space_ok;
    }
    else
    {
      /* allowed 已由上一步给出 */
    }

    if ((allowed == 0U) ||
        (bt_button_token_len >= (BT_BUTTON_TOKEN_MAX_LEN - 1U)))
    {
      bt_button_token_invalid = 1U;
      bt_button_pending_tick = HAL_GetTick();
      return 1U;
    }
  }

  bt_button_token[bt_button_token_len++] = (char)lower;
  bt_button_token[bt_button_token_len] = '\0';
  bt_button_pending_tick = HAL_GetTick();

  if (BT_Button_IsShortAnglePrefix(bt_button_token, bt_button_token_len) ||
      BT_Button_IsArgCommand(bt_button_token, bt_button_token_len) ||
      BT_Button_IsCustomPrefix(bt_button_token, bt_button_token_len) ||
      BT_Button_IsCustomToken(bt_button_token, bt_button_token_len))
  {
    return 1U;
  }

  if ((bt_button_token_len >= 3U) &&
      ((bt_button_token[1] == 'l') || (bt_button_token[1] == 'r')) &&
      (bt_button_token_len <= 5U) &&
      BT_Button_IsDigit((uint8_t)bt_button_token[bt_button_token_len - 1U]))
  {
    return 1U;
  }

  id = BT_Button_FindExact(bt_button_token, bt_button_token_len);
  if (id != BT_BUTTON_NONE)
  {
    (void)BT_Button_Push(id, 0);
    BT_Button_ResetToken();
    bt_button_suppress_line_end = 2U;
    return 1U;
  }

  if (!BT_Button_IsPrefix(bt_button_token, bt_button_token_len))
  {
    bt_button_token_invalid = 1U;
  }
  return 1U;
}

uint8_t BT_Button_Pop(bt_button_event_t *event)
{
  if ((event == NULL) || (bt_button_tail == bt_button_head))
  {
    return 0U;
  }
  __DMB();
  *event = bt_button_queue[bt_button_tail];
  bt_button_tail = (uint8_t)((bt_button_tail + 1U) % BT_BUTTON_QUEUE_SIZE);
  __DMB();
  return 1U;
}

const bt_button_stats_t *BT_Button_GetStats(void)
{
  return &bt_button_stats;
}

const char *BT_Button_GetName(bt_button_id_t id)
{
  switch (id)
  {
    case BT_BUTTON_ENABLE:       return "EN";
    case BT_BUTTON_M1:           return "M1";
    case BT_BUTTON_M2:           return "M2";
    case BT_BUTTON_FORWARD:      return "U";
    case BT_BUTTON_LEFT:         return "L";
    case BT_BUTTON_BACKWARD:     return "D";
    case BT_BUTTON_RIGHT:        return "R";
    case BT_BUTTON_STOP:         return "STOP";
    case BT_BUTTON_ROTATE_LEFT:  return "L<angle>";
    case BT_BUTTON_ROTATE_RIGHT: return "R<angle>";
    case BT_BUTTON_CUSTOM:       return "CUSTOM";
    default:                     return "NONE";
  }
}
