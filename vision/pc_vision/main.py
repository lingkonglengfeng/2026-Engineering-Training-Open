"""Desktop entry point. Preview is the default and never writes to the vehicle."""
import argparse
import csv
import json
import logging
from pathlib import Path
import sys
import time

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT / "vision/maixcam_service"))

from settings import DEFAULT, read_config


def run_connected(config, args, log_folder):
    import cv2
    from backend import PcBackend
    from screen_preview import ScreenPreviewClient
    from qr_backend import PcQrBackend,qr_service
    from transport import ControlClient, VideoClient, StreamContextChanged
    import service
    service.PREVIEW_ENABLED = False
    service.DIAGNOSTIC_LOG_PATH = str(log_folder / "service_errors.log")
    cv2.setNumThreads(1)
    control = video = backend = None
    try:
        control = ControlClient(config["host"], config["control_port"])
        role=config.get('camera_role','material')
        if control.hello.get('role','material')!=role:raise ValueError('Bridge role does not match configured camera')
        logging.info("Connected bridge: %s", control.hello.get("build"))
        video = VideoClient(config["host"], config["video_port"],mode=control.hello.get("video_mode","pull-v1"),control=control,
                            expected_size=(640,480) if role=='qr' else (480,360))
        logging.info("Video mode=%s%s",video.mode,"; upload new camera main.py for continuous capture" if video.mode=="pull-v1" else "")
        calibration = Path(config["calibration_file"])
        if not calibration.is_absolute():
            calibration = args.config.parent / calibration
        backend = (PcQrBackend if role=='qr' else PcBackend)(video, control, calibration, show=not args.headless)
        backend.screen_sender = ScreenPreviewClient(config['host'],control.hello)
        backend.screen_sender.enabled = config.get('screen_preview',True)
        if role!='qr':backend.configure_motion(config.get('motion'))
        backend.preview_only = args.mode == "preview"
        engine = (qr_service.QrService if role=='qr' else service.VisionService)(control, backend,clock=time.perf_counter)
        with (log_folder / ("frames_"+time.strftime("%H%M%S")+".csv")).open("a",newline="",encoding="utf-8") as stream:
            rows = csv.writer(stream)
            rows.writerow(["pc_monotonic_s","epoch","frame_id","camera_read_finished_ns","camera_read_us",
                           "request_to_jpeg_ms","decode_ms","detect_ms","mode","status",
                           "video_mode","camera_encode_us","previous_send_us","pc_receive_age_ms"])
            last_frame, next_log = 0, 0.
            while True:
                control.check()
                if args.mode == "preview":
                    for _ in range(32):
                        if not control.read(256):
                            break
                    if backend.config[0] == 0:
                        try:backend.fetch()
                        except StreamContextChanged:continue
                        backend.status = "Raw JPEG decoded OK"
                    else:
                        if role=='qr':
                            result=backend.observe(time.perf_counter(),scan=backend.task_display is None)
                            backend.preview(backend.config,result,result.get('task') or backend.task_display)
                        else:backend.observe(time.perf_counter())
                    backend.status = "PREVIEW ONLY / " + backend.status
                else:
                    worked = engine.tick()
                    if not worked:
                        backend.idle_preview()
                        time.sleep(.002)
                    if role=='qr':backend.preview(engine.config,None,engine.locked_task or engine.saved_display)
                if video.error is not None:
                    raise ConnectionError(str(video.error))
                backend.show()
                backend.screen_sender.submit(backend.display_image,backend.last_meta)
                now = time.perf_counter()
                if backend.last_meta is not None and backend.last_meta[1] != last_frame:
                    meta = backend.last_meta
                    rows.writerow([round(now,6),meta[0],meta[1],meta[2],meta[5],
                                   round(backend.last_transfer_ms,3),round(backend.last_decode_ms,3),
                                   round(backend.last_detect_ms,3),backend.config[0],backend.status,video.mode,
                                   meta[6] if len(meta)>6 else "",meta[7] if len(meta)>7 else "",
                                   round(video.last_receive_age_ms,3)])
                    last_frame = meta[1]
                if now >= next_log:
                    meta = backend.last_meta
                    logging.info("decode_ok=%d fail=%d video=%s wait=%.2fms camera_read=%.2fms encode=%.2fms previous_send=%.2fms detect=%.2fms mode=%d %s",
                                 backend.decode_ok,backend.decode_fail,video.mode,backend.last_transfer_ms,
                                 meta[5]/1000 if meta else 0,meta[6]/1000 if meta and len(meta)>6 else 0,
                                 meta[7]/1000 if meta and len(meta)>7 else 0,
                                 backend.last_detect_ms,backend.config[0],backend.status)
                    stream.flush()
                    next_log = now+2
                if args.max_frames and backend.frame_count >= args.max_frames:
                    return
    finally:
        if backend is not None and getattr(backend,'screen_sender',None): backend.screen_sender.close()
        if backend is not None: backend.close()
        if video is not None: video.close()
        if control is not None: control.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=HERE/"config.json")
    parser.add_argument("--host")
    parser.add_argument("--mode", choices=("preview","run"), default="preview")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--check", action="store_true", help="Check dependencies/config; no network or vehicle IO")
    args = parser.parse_args(argv)
    config = read_config(args.config)
    import cv2
    import numpy
    from backend import PcBackend
    from transport import ControlClient
    import service
    if args.check:
        import tkinter
        import serial
        print("Dependencies OK: Python %s, OpenCV %s, NumPy %s" % (sys.version.split()[0],cv2.__version__,numpy.__version__))
        print("Desktop UI OK: Tk",tkinter.TkVersion)
        print('Serial logging OK: pySerial',serial.__version__)
        print("Config OK:",args.config)
        print("Preview default; camera bridge ports %d/%d. No device commands sent." % (config["video_port"],config["control_port"]))
        return 0
    if args.host:
        config["host"] = args.host.strip()
    if not config["host"]:
        config["host"] = input("请输入 MaixCAM 的 IP 地址（相机网络设置中查看）: ").strip()
        if not config["host"]:
            raise ValueError("MaixCAM IP is required")
        args.config.parent.mkdir(parents=True,exist_ok=True)
        args.config.write_text(json.dumps(config,ensure_ascii=False,indent=2),encoding="utf-8")
        print("已保存 IP；以后双击即可连接。")
    log_folder = HERE/"logs"/time.strftime("%Y-%m-%d")
    log_folder.mkdir(parents=True,exist_ok=True)
    logging.basicConfig(level=logging.INFO,format="%(asctime)s %(levelname)s %(message)s",handlers=[
        logging.StreamHandler(),logging.FileHandler(log_folder/("session_"+time.strftime("%H%M%S")+".log"),encoding="utf-8")])
    logging.info("Mode=%s host=%s | Q/ESC in image window or Ctrl+C in console to exit",args.mode,config["host"])
    try:
        while True:
            try:
                run_connected(config,args,log_folder)
                return 0
            except (OSError,ConnectionError,ValueError,RuntimeError) as exc:
                logging.exception("连接/图像错误: %s。确认两端同一热点，已上传并运行相机桥 main.py。",exc)
                if args.max_frames:
                    return 1
                time.sleep(config["reconnect_seconds"])
    except KeyboardInterrupt:
        logging.info("Exited by user")
        return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception:
        logging.exception("启动失败；查看上方错误。")
        raise SystemExit(1)
