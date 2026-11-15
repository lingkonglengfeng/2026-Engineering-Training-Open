import { writeFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, resolve } from 'node:path';

// Conceptual wiring sheet. Net labels describe relationships; this is not a CAD netlist.
const here = dirname(fileURLToPath(import.meta.url));
const destination = resolve(here, '../docs/hardware/双板模块接口关系图_v0.3.svg');
const out = [];
const esc = value => String(value).replaceAll('&', '&amp;').replaceAll('<', '&lt;').replaceAll('>', '&gt;').replaceAll('"', '&quot;');
const put = s => out.push(s);
const text = (x, y, value, cls = 'body', extra = '') => put(`<text x="${x}" y="${y}" class="${cls}" ${extra}>${esc(value)}</text>`);
const rect = (x, y, w, h, cls = 'box', rx = 8) => put(`<rect x="${x}" y="${y}" width="${w}" height="${h}" rx="${rx}" class="${cls}"/>`);
const path = (d, cls = 'wire', extra = '') => put(`<path d="${d}" class="${cls}" ${extra}/>`);
const circ = (x, y, r = 5, cls = 'pin') => put(`<circle cx="${x}" cy="${y}" r="${r}" class="${cls}"/>`);

function section(x, y, w, h, label) {
  rect(x, y, w, h, 'section', 3);
  rect(x + 13, y - 9, Math.max(150, label.length * 17 + 28), 28, 'sectionLabelBg', 3);
  text(x + 24, y + 11, label, 'sectionTitle');
}
function label(x, y, value, kind = 'logic') {
  const w = Math.max(70, value.length * 9 + 20);
  rect(x, y - 20, w, 27, `tag tag-${kind}`, 3);
  text(x + 9, y, value, 'tagText');
}
function connector(x, y, w, name, pins, note = '', kind = 'signal') {
  const h = 46 + pins.length * 28 + (note ? 29 : 8);
  rect(x, y, w, h, `connector connector-${kind}`, 4);
  text(x + 15, y + 27, name, 'moduleTitle');
  path(`M${x + 10} ${y + 38}H${x + w - 10}`, 'thin');
  pins.forEach((pin, i) => {
    const yy = y + 62 + i * 28;
    circ(x + 19, yy - 5, 4, 'pin');
    text(x + 35, yy, `${i + 1}`, 'pinNo');
    text(x + 56, yy, pin, 'pinName');
  });
  if (note) text(x + 15, y + h - 11, note, 'note');
  return h;
}
function converter(x, y, w, name, input, output, note, kind = 'power') {
  rect(x, y, w, 135, `module module-${kind}`, 5);
  text(x + 18, y + 31, name, 'moduleTitle');
  path(`M${x + 12} ${y + 42}H${x + w - 12}`, 'thin');
  text(x + 18, y + 70, `IN   ${input}`, 'pinName');
  text(x + 18, y + 97, `OUT  ${output}`, 'pinName');
  text(x + 18, y + 122, note, 'note');
}
function transceiver(x, y, name, tx, rx, bus, rate) {
  rect(x, y, 325, 187, 'module module-signal', 5);
  text(x + 15, y + 30, name, 'moduleTitle');
  path(`M${x + 10} ${y + 41}H${x + 315}`, 'thin');
  text(x + 17, y + 69, `1 D   ← ${tx}`, 'pinName');
  text(x + 17, y + 96, `4 R   → ${rx}`, 'pinName');
  text(x + 17, y + 123, `3 VCC  3V3_CAN`, 'pinName');
  text(x + 17, y + 150, `7/6   ${bus} H/L`, 'pinName');
  text(x + 17, y + 175, `8 RS 单独配置 · ${rate}`, 'note');
}
function idc(x, y, name, nets) {
  const w = 191, h = 390;
  rect(x, y, w, h, 'connector connector-signal', 4);
  text(x + 13, y + 27, name, 'moduleTitle');
  text(x + 13, y + 48, '奇数=信号 偶数=GND', 'note');
  path(`M${x + 9} ${y + 58}H${x + w - 9}`, 'thin');
  nets.forEach((net, i) => {
    const yy = y + 85 + i * 22;
    circ(x + 17, yy - 5, 3.5, 'pin');
    text(x + 29, yy, `${i * 2 + 1}/${i * 2 + 2}`, 'pinNo');
    text(x + 79, yy, net, 'idcNet');
  });
  text(x + 13, y + h - 11, '红边标识 1 脚', 'note');
}
function quiet(x, y, w, name, lines) {
  const h = 142;
  rect(x, y, w, h, 'module module-quiet', 5);
  text(x + 14, y + 29, name, 'moduleTitle');
  path(`M${x + 10} ${y + 40}H${x + w - 10}`, 'thin');
  lines.forEach((line, i) => text(x + 14, y + 68 + i * 27, line, 'pinName'));
}

put(`<?xml version="1.0" encoding="UTF-8"?>`);
put(`<svg xmlns="http://www.w3.org/2000/svg" width="2600" height="1800" viewBox="0 0 2600 1800" role="img" aria-labelledby="title desc">`);
put(`<title id="title">智能搬运车双板模块与接口关系图 v0.3</title>`);
put(`<desc id="desc">仿原理图样式的两板连接示意。左侧为电源/信号板，含XT60电池、5V和舵机电源、两颗CAN收发器、六个ZDT XH2.54四针接口、达妙XT30二加二接口、两路舵机和两路视觉XH2.54四针接口。右侧为主控拓展板，含核心板排座及传感器接口。两板通过26芯牛角排线传信号、XH二针线束传5V。</desc>`);
put(`<style>
  text{font-family:'Microsoft YaHei','Noto Sans CJK SC',Arial,sans-serif;fill:#183349}
  .background{fill:#fff}.frame{fill:none;stroke:#a53c33;stroke-width:2.4}.board{fill:#fff;stroke:#293f52;stroke-width:2}.section{fill:#fff;stroke:#a8b5bf;stroke-width:1.5}.sectionLabelBg{fill:#fff;stroke:none}
  .sectionTitle{font-size:22px;font-weight:700;fill:#bd4b33}.title{font-size:33px;font-weight:700}.subtitle{font-size:17px;fill:#5a6f7e}.boardTitle{font-size:29px;font-weight:700;fill:#bd4b33}.moduleTitle{font-size:19px;font-weight:700}.body{font-size:17px}.pinName{font-size:16px}.pinNo{font-size:14px;fill:#af4333;font-weight:700}.note{font-size:13px;fill:#586c7c}.idcNet{font-size:14px;font-weight:700}.footer{font-size:15px}
  .module,.connector{fill:#fff;stroke:#bd4b33;stroke-width:1.7}.module-power,.connector-power{fill:#fff8f5}.module-servo,.connector-servo{fill:#fffaf0}.module-signal,.connector-signal{fill:#f5fbf8}.module-quiet,.connector-quiet{fill:#f7faff}.pin{fill:#fff;stroke:#bd4b33;stroke-width:1.5}.thin{stroke:#c4d0d7;stroke-width:1.3;fill:none}.wire{stroke:#526b80;stroke-width:2.2;fill:none}.wireBat{stroke:#d66a46;stroke-width:4;fill:none}.wireLogic{stroke:#327dcc;stroke-width:4;fill:none}.wireServo{stroke:#ba8c24;stroke-width:3.6;fill:none}.wireSig{stroke:#3e9870;stroke-width:3.2;fill:none}.ground{stroke:#546779;stroke-width:2.3;fill:none}.tag{stroke-width:1.3}.tag-power{fill:#fff0e9;stroke:#d88e6f}.tag-logic{fill:#eaf4ff;stroke:#81b3e3}.tag-servo{fill:#fff5de;stroke:#dec17a}.tag-signal{fill:#eaf8f0;stroke:#89c5a4}.tagText{font-size:14px;font-weight:700}.warning{fill:#fff3e6;stroke:#e7a16c;stroke-width:1.5}
</style>`);
rect(0, 0, 2600, 1800, 'background', 0);
rect(28, 28, 2544, 1744, 'frame', 0);
text(69, 76, '智能搬运车 · 双板模块与接口关系图', 'title');
text(71, 103, 'v0.3 概念示意｜依照原理图分区表达，网络名表示电气关系；连接器物理 1 脚须按实物核对', 'subtitle');

// Board boundaries and functional zones.
rect(61, 137, 1485, 1505, 'board', 0);
rect(1568, 137, 971, 1505, 'board', 0);
text(86, 181, '板 1：电源 / 信号板 PWR-IO', 'boardTitle');
text(1593, 181, '板 2：主控拓展板 CTRL', 'boardTitle');
section(85, 218, 1437, 312, '01  电池入口 / 电源转换 / 板间 5V');
section(85, 558, 1437, 650, '02  CAN 收发器 / 电机接口');
section(85, 1223, 1437, 375, '03  舵机 / 视觉 / 补光');
section(1592, 218, 924, 758, '04  主控 / 牛角排线');
section(1592, 1024, 924, 574, '05  本地主控与传感器接口');

// Battery input and conversion on PWR-IO.
connector(108, 278, 202, 'J1  XT60 电池', ['BAT− / GND', 'BAT+ / 6S'], '唯一电池入口', 'power');
path('M310 337H355', 'wireBat');
rect(355, 319, 96, 36, 'module module-power', 2); text(370, 344, 'F1 保险', 'pinName');
path('M451 337H488', 'wireBat');
rect(488, 319, 90, 36, 'module module-power', 2); text(501, 344, 'SW 主开关', 'pinName');
path('M578 337H760', 'wireBat');
path('M578 337V253H1020V337', 'wireBat');
circ(675, 337, 5, 'pin');
path('M675 337V386', 'wireBat');
text(686, 382, 'TVS / 反接保护', 'note');
path('M663 396H687M667 403H683M672 410H678', 'ground');
label(608, 302, 'VBAT_MOTOR 18–25.2V', 'power');
converter(760, 274, 240, 'U5  5V DC/DC', 'VBAT', '5V_SYS', '≥3A 档待负载实测', 'power');
converter(1020, 274, 240, 'U6  舵机 DC/DC', 'VBAT', 'SERVO_V', '电压按舵机铭牌冻结', 'servo');
converter(1280, 274, 218, 'U7  CAN LDO', '5V_SYS', '3V3_CAN', '仅给两颗收发器', 'signal');
path('M1000 371V435H1270V371H1280', 'wireLogic');
label(820, 460, '5V_CTRL / GND', 'logic');
label(1080, 460, 'SERVO_V / PGND', 'servo');

// Inter-board logic power connector, intentionally separate from IDC.
connector(1317, 412, 181, 'J2  XH2.54-2P', ['5V_CTRL', 'GND_CTRL'], '', 'logic');
connector(1614, 412, 181, 'J2  XH2.54-2P', ['5V_CTRL', 'GND_CTRL'], '', 'logic');
path('M1498 474H1614', 'wireLogic');
text(1514, 464, '5V', 'note');

// CAN transceivers and matching motor connector symbols.
transceiver(107, 655, 'U1  SN65HVD230 · ZDT', 'PB13', 'PB12', 'ZDT', '500 kbit/s');
transceiver(107, 883, 'U2  SN65HVD230 · 达妙', 'PA12', 'PA11', 'DM', '1 Mbit/s');
label(107, 1116, 'CANH/L：TVS → 接口；120Ω 跳线可选', 'signal');
label(107, 1150, 'U1/U2 各自 RS、100nF、TX/RX/H/L 测试点', 'signal');

const zdtPins = ['GND_MOTOR', 'VBAT_MOTOR', 'ZDT_CANH', 'ZDT_CANL'];
const zdtXs = [498, 772, 1046];
let n = 1;
for (const yy of [604, 806]) {
  for (const xx of zdtXs) connector(xx, yy, 243, `J${n + 2}  ZDT #${n} · XH4`, zdtPins, '四针同座：动力 + CAN', 'power'), n++;
}
connector(498, 1000, 342, 'J9  达妙 · XT30(2+2)', ['DM_CANL（信号）', 'DM_CANH（信号）', 'GND_MOTOR（动力）', 'VBAT_MOTOR（动力）'], '符号顺序依旧图；插合面再核对', 'power');
rect(866, 1038, 424, 97, 'warning', 4);
text(884, 1069, 'ZDT 六个 4P 口共用 FDCAN2 总线', 'body');
text(884, 1097, '仅在总线物理两端启用 120Ω 终端', 'body');
text(884, 1122, 'XH 动力触点电流和线规需实测确认', 'note');

// IDC ribbon pair: each signal is adjacent to a reference ground.
const idcNets = ['DM_TX','DM_RX','ZDT_TX','ZDT_RX','SERVO1','SERVO2','SERVO_EN','LIGHT1','LIGHT2','CAM1_TX','CAM1_RX','CAM2_TX','CAM2_RX'];
idc(1326, 603, 'J10  IDC 2×13', idcNets);
idc(1612, 603, 'J1  IDC 2×13', idcNets);
for (let i = 0; i < 6; i++) path(`M1517 ${719 + i * 48}H1612`, 'wireSig');
text(1516, 593, '26P 排线', 'note');
text(1533, 1108, '只传信号', 'note');

// Servo, vision and light output modules.
connector(108, 1260, 250, 'J11  抓取舵机 3P', ['GND_SERVO', 'SERVO_V', 'PD12 PWM'], '功率回流直返电源区', 'servo');
connector(380, 1260, 250, 'J12  转盘舵机 3P', ['GND_SERVO', 'SERVO_V', 'PD13 PWM'], 'SERVO_V 待型号确认', 'servo');
connector(650, 1260, 256, 'J13  视觉1 · XH4', ['GND', '5V_CAM', 'PB15 MCU_RX', 'PB14 MCU_TX'], '线束随新版座重做', 'logic');
connector(925, 1260, 256, 'J14  视觉2 · XH4', ['GND', '5V_CAM', 'PA3 MCU_RX', 'PA2 MCU_TX'], '相机 TX 电平需核对', 'logic');
connector(1200, 1260, 284, 'J15/16  补光 XH3', ['GND', '灯电源', 'PD1/PD2 PWM'], '电流超 GPIO 能力加驱动', 'signal');
label(108, 1518, '舵机动力不走 IDC / 主控排针', 'servo');
label(654, 1518, '视觉 5V 在本板分支滤波', 'logic');

// CTRL board: local supply and core header.
converter(1614, 280, 181, 'U3  3.3V', '5V_CTRL', '3V3_MCU', '与核心板 LDO 二选一', 'logic');
rect(1845, 275, 633, 648, 'module module-quiet', 5);
text(1870, 314, 'U_CORE  STM32H723ZGT6 核心板排座 · 2×30P', 'moduleTitle');
path('M1859 328H2465', 'thin');
const coreRows = [
  ['PA12 / PA11','FDCAN1_TX / RX → 达妙'],
  ['PB13 / PB12','FDCAN2_TX / RX → ZDT'],
  ['PD12 / PD13','TIM4_CH1 / CH2 → 舵机'],
  ['PD0','SERVO_EN（拟分配）'],
  ['PD1 / PD2','LIGHT_PWM1 / PWM2'],
  ['PB14 / PB15','USART1_TX / RX → 视觉1'],
  ['PA2 / PA3','USART2_TX / RX → 视觉2'],
];
coreRows.forEach((r, i) => {
  const yy = 372 + i * 75;
  circ(1865, yy - 5, 4, 'pin');
  text(1882, yy, r[0], 'pinName');
  text(2071, yy, r[1], 'pinName');
  if (i < coreRows.length - 1) path(`M1865 ${yy + 19}H2457`, 'thin');
});
text(1870, 892, '3V3_MCU 与 PWR-IO 的 3V3_CAN 不并联；两板 GND 互通', 'note');

// Quiet expansion-board connectors and local devices.
quiet(1614, 1060, 280, 'J3  HWT101 · XH4', ['PE3 USART10_TX','PE2 USART10_RX','GND / 模块电源待核']);
quiet(1910, 1060, 280, 'J4  I2C 传感器 · XH4', ['PB6 SCL / PB7 SDA','3V3_MCU / GND','板侧上拉到 3.3V']);
quiet(2206, 1060, 280, 'J5  串口屏 · XH4', ['PB10 USART3_TX','PB11 USART3_RX','5V / GND']);
quiet(1614, 1230, 280, 'J6  SWD', ['PA13 SWDIO / PA14 SWCLK','NRST / 3V3 / GND','靠板边']);
quiet(1910, 1230, 280, 'U4/U5  本地器件', ['IPS200：SPI2 + CS/DC/RST','W25Q64：SPI1 + PA4 CS','本板就近去耦']);
quiet(2206, 1230, 280, '按键 / 调试', ['PA0 START / PA1 STOP','PC10/11 UART4','STOP 动力切断另核']);
rect(1614, 1400, 872, 192, 'warning', 4);
text(1635, 1435, '设计校核提示', 'moduleTitle');
text(1635, 1468, '① 牛角座两端 1 脚与排线红边，按实物逐针测通断；不能凭图像镜像。', 'body');
text(1635, 1500, '② CAN 收发器位于电源/信号板的安静信号区，远离开关节点和电机功率回路。', 'body');
text(1635, 1532, '③ 5V 模块、电机 XH 电流、舵机电压与堵转电流均待实测冻结。', 'body');
text(1635, 1564, '④ 本图为功能关系示意；正式原理图必须完成 ERC、PCB 完成 DRC。', 'body');

// Footer/title block, inspired by the existing EasyEDA sheets.
rect(61, 1662, 2478, 81, 'frame', 0);
path('M1880 1662V1743M2190 1662V1743M2370 1662V1743', 'frame');
text(82, 1692, '工训智能搬运 · 双板电气关系', 'moduleTitle');
text(82, 1723, '源：2026-08-29 原理图、现行 .ioc、2026-09 接口讨论。所有连接器针序以正式 CAD + 线束通断复核为准。', 'footer');
text(1902, 1692, '图名：模块和接口关系图', 'footer');
text(1902, 1723, '性质：概念图 / 不可直接制板', 'footer');
text(2210, 1692, '版本', 'footer'); text(2210, 1723, 'v0.3', 'footer');
text(2390, 1692, '日期', 'footer'); text(2390, 1723, '2026-09-23', 'footer');
put('</svg>');
writeFileSync(destination, out.join('\n') + '\n', 'utf8');
process.stdout.write(destination + '\n');
