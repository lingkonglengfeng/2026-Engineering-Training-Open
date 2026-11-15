"""生成离线单页控制台，并用当前固件路由检查指令覆盖。"""
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
rows=[]


def add(group, label, command, args='', note='', console=False, blocked=False):
    rows.append(dict(id=len(rows),group=group,label=label,command=command,args=args,note=note,console=console,blocked=blocked))


def build():
    rows.clear()
    for label,command in [('使能','en'),('失能','dis'),('失能别名','off'),('停止','stop'),('转向模式','m1'),('平移模式','m2'),('前进','u'),('后退','d'),('左移/左转','l'),('右移/右转','r'),('相对左转90°','l90'),('相对右转90°','r90')]:
        add('底盘',label,command,note='方向取决于M1/M2；按住方向键松开自动STOP。角度可在自定义发送中修改。')
    for label,command in [('链路自检','echo'),('航向归零','zero'),('四轮位置','posq'),('记录轮位置基准','posref'),('位置差值','posd'),('轮位置自标定','poscal'),('所有驱动版本','verall')]:
        add('底盘',label,command,note='zero后保持静止约3秒；poscal会驱动车轮。')
    for command,args,note in [('startpose','1','1/2起始区，车摆好后静止约3秒'),('startzone','1','startpose别名'),('depart','','开始当前流程阶段'),('ok','','仅WAIT_OK确认继续'),('match status','','查看流程状态'),('match reset','','中止后重置流程及相机状态'),('leave','','驶出当前起始区'),('home','','返回当前起始区'),('cross','2','目标起始区1/2'),('endturn','','当前固件已禁用运动中转向')]:
        add('流程',note.split('，')[0],command,args,note)
    for command,args,note in [('map','','地图与位姿'),('grid','','地图别名'),('wp','','命名工位'),('waypoints','','工位别名'),('goto','raw','目标raw/rough1..3/temp1..3/qr/h1/h2/exit1/exit2/center或x y；会驱动车辆'),('gotomm','1200 2250','目标毫米x y；当前解析仍共用目标解析器，建议坐标至少一项>=48'),('path','raw','规划目标路线'),('pose','24 3','设置格坐标0..47'),('posemm','1200 2250','设置毫米坐标'),('obs','24 20','添加障碍，两项<48按格，否则毫米'),('free','24 20','移除运行时障碍'),('clear','','清除运行时障碍'),('gridclear','','地图重置/清理'),('nudge','u 50','地图方向u/d/l/r，距离1..1000mm')]:
        add('导航',note.split('，')[0],command,args,note)
    for command,args,note in [('nav','','查看导航参数'),('spd','0.35','速度比例0.02..1.00'),('dcomp','0','距离补偿-0.30..0.30'),('t1000','1930','每1000mm基准耗时50..20000ms'),('cnt','65536','每圈计数100..1000000'),('t90','1500','90度代价200..10000ms'),('t180','3000','180度代价200..10000ms'),('tarc','0','0关闭；否则200..10000ms'),('aspd','0.1','角速度参数；固件校验范围'),('navreset','','恢复导航默认参数'),('kp','0.1','航向PID P'),('ki','0','航向PID I'),('kd','0','航向PID D'),('save','','保存参数到Flash'),('read','','读取Flash参数'),('show','','查看PID参数'),('flash','','Flash信息'),('delete','','擦除PID存储，仅有线裸指令')]:
        add('参数',note,command,args,note,console=command=='delete')
    for command,options in [('valign',['on','off','status','smooth on','smooth off','search image-right','search off','trace on','trace off','trace status','trace dump','trace clear','trace cancel']),('qrscan',['on','off','start','status']),('camraw',['1','2','all','off'])]:
        for option in options:
            add('视觉',command+' '+option,command+' '+option,note='主控指令；状态/参数检查请停车执行。camraw会增加日志量。')
    add('视觉','相机统计','camstat')
    for command,args,note in [('vsource','2 1 ring 2 3','工位1..3 相机1|2 类型plate|ring ID 轴1..3'),('vgain','2 0 -0.001 0.001 0','工位 Fdx Fdy Rdx Rdy；单位/正负必须实车标定'),('vpid','2 1 0 0','工位 P倍率 I(1/s) D(s)'),('vtune','2 5 0.03 100 100','工位 容差px 最大速度 pulse_ms settle_ms')]:
        add('视觉',command+'设置',command,args,note)
    for option in ['help','status','stop','camera','camera init','camera status','camera mode position','camera mode encoder','start','start go','start save','start status','zero','zero dm','zero dm save','zero dm confirm','zero 5','zero 6','top 5','guardoff']:
        add('机械臂','arm '+option,'arm '+option,note='相机展开/起始姿态会执行动作；save按当前固件存储。')
    for verb,args,note in [('enable','5','轴5升降、6伸缩、dm旋转'),('disable','5','轴5/6/dm'),('clear','5','清除5/6标定'),('pct','5 50','5/6绝对行程百分比0..100；不自动使能'),('percent','5 50','pct别名'),('mark','5 min','5/6 min|max记录端点'),('seek','5 min','5/6 min|max寻找端点'),('pulse','5 100','5/6 有符号脉冲'),('mm','5 10','5/6 有符号毫米'),('span','5 -219505','5/6 有符号反馈全行程'),('travel','5 100','5/6 物理全行程mm'),('scale','5 640','5/6 每mm脉冲'),('speed','5 30','5/6 线性轴RPM'),('dir','6 0','5/6 0|1 RAM方向'),('feedback','5 -2000','升降反馈counts/mm'),('dropcal','10','实测下降mm'),('rotate','0','绝对旋转deg'),('by','10','相对旋转deg'),('limit','-90 90','旋转最小/最大deg'),('guard','10 -90 90','高度mm 最小/最大deg'),('guardtop','10 -90 90','下降量mm 最小/最大deg')]:
        add('机械臂','arm '+verb,'arm '+verb,args,note)
    for parent,options in [('grip',['enable','disable','open','close','save open','save close']),('table',['enable','disable'])]:
        for option in options:
            add('机械臂','arm '+parent+' '+option,'arm '+parent+' '+option)
    for command,args,note in [('arm grip pulse','1500','舵机脉宽us'),('arm table go','1','放料盘位置1/2/3'),('arm table save','1','记录位置1/2/3'),('arm table angle','0','角度deg')]:
        add('机械臂',command,command,args,note)
    for verb in ['help','status','target','on','off','enable','disable','query','angle','home','zero','zerohere','zero save','zero confirm','zero w25','zero w25 save','zero w25 status','zero w25 clear','diag','diag status','probe','clear','recover','loopback','loopback ext']:
        add('达妙','dm '+verb,'dm '+verb,note='zero/zerohere为RAM软零；zero save/confirm写电机Flash；w25 save写W25；HOME会运动。')
    for verb,args,note,blocked in [('pos','0 1','电机坐标rad [rad/s]，不是机械臂deg',False),('br','4','本地CAN码0..9，4=1Mbps',False),('brscan','','波特率扫描',False),('txlog','0','0/1控制帧日志',False),('resend','0','0/1目标重发',False),('setid','0x11 0x01','反馈ID/命令ID，RAM',False),('speed','0','原始速度接口被机械臂安全入口阻止',True),('mit','0 0 100 1 0','原始MIT接口被机械臂安全入口阻止',True),('raw','0x01 00 00 00 00 00 00 00 00','原始CAN接口被机械臂安全入口阻止',True)]:
        add('达妙','dm '+verb,'dm '+verb,args,note,blocked=blocked)
    for command,args,note in [('help','','有线控制台帮助'),('?','','help别名'),('version','','驱动版本'),('ver','','版本别名'),('yaw','','航向与零漂'),('status','','底盘状态'),('verbose','','开启CAN日志'),('quiet','','关闭CAN日志'),('uecho','0','0/1逐字回显'),('run','0','四轮转速RPM -3000..3000'),('test','1 0','地址1..255 RPM -3000..3000'),('zpos','1 100 30','地址 脉冲 RPM')]:
        add('诊断',note,command,args,note,console=True)
    add('诊断','ZDT查询','zdt query','1','电机地址；地址5/6也可查询')
    checked=subprocess.run([sys.executable,str(ROOT/'tools/check_bt_whitelist.py')],capture_output=True,text=True,check=True)
    required={re.sub(r'\s+',' ',s.strip()) for s in re.findall(r'^  &(.*?)\s{2,}(?:ok|console-only)',checked.stdout,re.M)}
    # 原检查器把子参数比较也当根verb，下面三项不是独立指令；?为help别名。
    exclusions={'dm 0','dm 1','dm confirm','dm ?'}
    missing=[name for name in required-exclusions if not any((row['command']+' '+row['args']).startswith(name) for row in rows)]
    if missing:raise ValueError('Missing firmware command coverage: '+repr(missing))
    audit=dict(date='2026-10-07',commands=len(rows),firmware_routes=len(required),uncovered=missing,
        checker_exclusions=sorted(exclusions),source_hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [ROOT/'Core/Src/zdt_motor.c',ROOT/'Core/Src/arm_calibration.c',ROOT/'Core/Src/dm_motor.c',ROOT/'Core/Src/vision.c',ROOT/'Core/Src/bt_button.c']})
    template=(HERE/'template.html').read_text(encoding='utf-8')
    app=(HERE/'app.js').read_text(encoding='utf-8')
    core=(HERE/'serial.js').read_text(encoding='utf-8')
    html=template.replace('/* CATALOG */','const CATALOG='+json.dumps(rows,ensure_ascii=False).replace('</','<\\/')+';').replace('/* SERIAL_CORE */',core).replace('/* APPLICATION */',app)
    out=ROOT/'output/serial_console'
    out.mkdir(parents=True,exist_ok=True)
    (out/'工程串口控制台.html').write_text(html,encoding='utf-8')
    (out/'command_catalog.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2),encoding='utf-8')
    (out/'audit.json').write_text(json.dumps(audit,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(audit,ensure_ascii=False))


if __name__=='__main__':build()
