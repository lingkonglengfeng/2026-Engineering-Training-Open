"""Create a portable source bundle without local IP, calibration or logs."""
from pathlib import Path
import zipfile

ROOT = Path(__file__).resolve().parents[1]
SOURCES = [
    "docs/electrical/2026-10-29_窗口55x80_10ms补发与圆盘75_V218.md",
    "docs/electrical/2026-10-29_取消圆盘纠偏后靠近30mm_V215.md",
    "docs/electrical/2026-10-29_抓取框再下移15px_V214.md",
    "docs/electrical/2026-10-29_圆盘伸出最多50次_V213.md",
    "docs/electrical/2026-10-29_低位等待与伸出确认_V212.md",
    "docs/electrical/2026-10-29_伸出确认补发与进框等待23s.md",
    "tools/update_pc_red_samples.py",
    "vision/pc_vision/test_data/red_material_calibration.json",
    "vision/pc_vision/test_data/red_20261028_164555_raw.png",
    "vision/pc_vision/test_data/red_20261028_164644_raw.png",
    "vision/pc_vision/test_data/red_20261029_012640_raw.png",
    "docs/electrical/2026-10-29_红色重新取样与STOP截图核对.md",
    "docs/electrical/2026-10-29_抓取高度等待与方框下移10px.md",
    "视觉上位机.cmd", "一键图传检查.cmd", "一键电脑视觉.cmd", "检查电脑视觉环境.cmd",
    "vision/protocol8.py", "vision/maixcam_service/service.py",
    "vision/maixcam_material/main.py", "vision/maixcam_material/plate_tangent.py",
    "vision/maixcam_ring/middle_ring.py", "vision/maixcam_bridge/main.py",
    "vision/maixcam_bridge/bridge.py", "tools/build_pc_vision_bridge.py",
    "tools/package_pc_vision_bridge.py", "vision/maixcam_qr/service.py",
    "vision/maixcam_qr_bridge/main.py", "vision/maixcam_qr_bridge/app.yaml", "vision/maixcam_qr_bridge/app.png", "vision/maixcam_qr_bridge/README.md",
    "vision/maixcam_bridge/app.yaml", "vision/maixcam_bridge/app.png",
    "vision/maixcam_bridge/dist/maix-cam1_pc_bridge-v1.3.0.zip",
    "vision/maixcam_qr_bridge/dist/maix-cam2_pc_bridge-v1.4.0.zip",
    "vision/pc_vision/assets/camera.png", "vision/pc_vision/assets/camera.ico",
    "vision/pc_vision/assets/LICENSE-fluentui-emoji.txt", "vision/pc_vision/assets/SOURCE.md",
    "vision/pc_vision/test_data/black_cyan_raw.png", "vision/pc_vision/test_data/cyan_only_raw.png",
    "vision/pc_vision/test_data/rough_occupied_raw.png", "vision/pc_vision/test_data/temp_occupied_a_raw.png",
    "vision/pc_vision/test_data/temp_occupied_b_raw.png",
    "vision/pc_vision/test_data/plate_soft_edge_a.png",
    "vision/pc_vision/test_data/plate_soft_edge_b.png",
    "vision/pc_vision/test_data/plate_soft_edge_c.png",
    "docs/electrical/2026-10-28_圆盘盘沿低识别与320宽修复_PC2.5.6.md",
    "docs/electrical/2026-10-28_按轮日志与三次漏下降核对_PC2.5.5.md",
    "tools/extract_latest_match_run.py",
    "tools/analyze_pc_blue_pair.py", "vision/pc_vision/test_data/blue_lightblue_raw.png",
    "vision/pc_vision/test_data/blue_lightblue_calibration.json",
    "docs/electrical/2026-10-17_PC蓝浅蓝判色与顶面修正.md",
    "docs/electrical/2026-10-27_CAM2障碍联动与绕行_V192.md",
    "docs/electrical/2026-10-27_CAM2障碍融入比赛流程_V194.md",
    "docs/electrical/2026-10-28_机械臂可视化控制与W25参数_V196.md",
    "docs/electrical/2026-10-28_载物盘19_109_199与圆环死区2px_V197.md",
    "docs/electrical/2026-10-28_W25启动优先与共用放置纠偏_V198.md",
    "tools/analyze_pc_cylinder_tops.py", "vision/pc_vision/test_data/cylinder_top_cases.json",
    "vision/pc_vision/test_data/cylinder_20261026_162045_28994_raw.png",
    "vision/pc_vision/test_data/cylinder_20261026_162047_29028_raw.png",
    "vision/pc_vision/test_data/cylinder_20261026_162049_29073_raw.png",
    "vision/pc_vision/test_data/cylinder_20261026_162053_29172_raw.png",
]


def build():
    from package_pc_vision_bridge import package
    if not (ROOT / 'vision/maixcam_bridge/dist/maix-cam1_pc_bridge-v1.3.0.zip').exists() or not (ROOT / 'vision/maixcam_qr_bridge/dist/maix-cam2_pc_bridge-v1.4.0.zip').exists():
        package()
    destination = ROOT / "output/PC视觉一键启动_2026-10-29_2.5.6_Plate75.zip"
    destination.parent.mkdir(parents=True, exist_ok=True)
    desktop = ROOT / "vision/pc_vision"
    sources = SOURCES + [str(path.relative_to(ROOT)) for path in desktop.iterdir()
                        if path.is_file() and (path.suffix in (".py", ".ps1", ".md")
                                              or path.name in ("requirements.txt", "config.example.json", "config.cam2.example.json", ".gitignore"))]
    with zipfile.ZipFile(destination, "w", zipfile.ZIP_DEFLATED) as bundle:
        for relative in sources:
            bundle.write(ROOT / relative, "PC视觉一键启动/" + Path(relative).as_posix())
        bundle.write(desktop / "README.md", "PC视觉一键启动/README.md")
        rollback = ROOT / "vision/maixcam_service/dist/maix-v1-v1.1.17.zip"
        bundle.write(rollback, "PC视觉一键启动/rollback/" + rollback.name)
        rollback=ROOT/'vision/maixcam_qr/dist/maix-cam2_qr-v1.0.3.zip'
        bundle.write(rollback,'PC视觉一键启动/rollback/'+rollback.name)
    return destination


if __name__ == "__main__":
    print(build())
