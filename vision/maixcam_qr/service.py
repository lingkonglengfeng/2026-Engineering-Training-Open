"""CAM2前视扫码：固定8字节CRC8，四组任务码，独立单文件入口由工具生成。"""
import argparse
import math
from pathlib import Path
import sys
import time
import traceback

sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
from protocol8 import crc8, make_frame, PROTOCOL_COLOR_IDS

BUILD='2026-10-09-cam2-qr-launcher-v3'
SET,ACK,HEARTBEAT,QR_GROUP=0x80,0x81,0x82,0x07
STOP,QR_MODE,QR_STATION=0,6,4
PARTIAL_TIMEOUT,RECOVERY_GAP=.050,.055
CAPTURE_WIDTH,CAPTURE_HEIGHT,CAPTURE_FPS=640,480,30
CAMERA_ROTATION_DEG=180  # CAM2倒装；先旋转，再解码/缩放预览，UI文字不旋转。
PREVIEW_WIDTH,PREVIEW_HEIGHT,PREVIEW_HZ=320,240,5
CONFIRM_FRAMES=3
UART_PORT,UART_BAUDRATE='/dev/ttyS0',115200


def parse_task(payload):
    """规则编号原码：1/3颜色组各3种，2/4放置位置各为123排列。"""
    if isinstance(payload,bytes):
        try:payload=payload.decode('ascii')
        except UnicodeDecodeError:return None
    if not isinstance(payload,str):return None
    text=payload.strip(' \r\n\t')
    if len(text)!=15 or any(text[i]!='+' for i in (3,7,11)):return None
    groups=[]
    for index,start in enumerate((0,4,8,12)):
        part=text[start:start+3]
        if len(part)!=3 or any(c not in '123456' for c in part):return None
        values=tuple(ord(c)-48 for c in part)
        if index in (0,2):
            if len(set(values))!=3:return None
        elif set(values)!={1,2,3}:return None
        groups.append(values)
    return text,tuple(groups)


def encode_task(text,request_sequence):
    task=parse_task(text)
    if task is None:raise ValueError('Invalid four-group task code')
    packets=[]
    for index,values in enumerate(task[1]):
        if index in (0,2):values=tuple(PROTOCOL_COLOR_IDS[c] for c in values)
        packets.append(make_frame(QR_GROUP,request_sequence,index+1,True,bytes((*values,4))))
    return b''.join(packets)


class TaskTracker:
    def __init__(self):self.reset()
    def reset(self):self.pending=None;self.count=0;self.status='SCANNING'
    def update(self,payloads):
        tasks={task[0] for value in payloads for task in [parse_task(value)] if task is not None}
        if len(tasks)!=1:
            self.pending,self.count=None,0
            self.status='AMBIGUOUS' if len(tasks)>1 else 'BAD_TASK' if payloads else 'SCANNING'
            return None
        text=next(iter(tasks))
        self.count=self.count+1 if self.pending==text else 1
        self.pending=text
        self.status='CONFIRM %d/%d'%(self.count,CONFIRM_FRAMES)
        return text if self.count>=CONFIRM_FRAMES else None


class FrameParser:
    def __init__(self):self.buffer=bytearray();self.started=0.;self.errors=0
    def expire(self,now):
        if self.buffer and now-self.started>=PARTIAL_TIMEOUT:
            self.buffer.clear();self.errors+=1
    def feed(self,data,now):
        self.expire(now);frames=[]
        for byte in data:
            if not self.buffer:
                if byte!=0x6B:continue
                self.started=now
            self.buffer.append(byte)
            if len(self.buffer)<8:continue
            if crc8(self.buffer[:7])==self.buffer[7]:
                frames.append(bytes(self.buffer));self.buffer.clear()
            else:
                self.errors+=1;del self.buffer[0]
                while self.buffer and self.buffer[0]!=0x6B:del self.buffer[0]
                self.started=now
        return frames


class FrameWriter:
    """最多一个32字节业务组；ACK优先，组内不插心跳，旧尾在新ACK前丢弃。"""
    def __init__(self,serial):
        self.serial=serial;self.controls=[];self.pending=b'';self.original=None
        self.started=self.resume_at=0.;self.sent=self.errors=0
    def abandon(self,now,retry=True):
        if self.original and self.original[1]!='qr' and retry:
            self.controls=self.controls[:7];self.controls.insert(0,self.original)
        self.pending,self.original=b'',None;self.resume_at=now+RECOVERY_GAP
    def discard_result(self,now):
        if self.original and self.original[1]=='qr':self.abandon(now,False)
    def control(self,packet,kind='ack'):
        if kind=='heartbeat':self.controls=[p for p in self.controls if p[1]!=kind]
        if len(self.controls)>=8:return False
        self.controls.append((packet,kind));return True
    def ready(self,now):return not self.pending and not self.controls and now>=self.resume_at
    def pump(self,now):
        if now<self.resume_at:return False
        if not self.pending:
            if not self.controls:return False
            self.original=self.controls.pop(0);self.pending=self.original[0];self.started=now
        try:
            if now-self.started>=PARTIAL_TIMEOUT:raise OSError('partial group expired')
            written=self.serial.write(self.pending)
            if not isinstance(written,int) or not 0<=written<=len(self.pending):raise OSError('invalid write count')
            self.pending=self.pending[written:]
            if self.pending:return False
            self.original=None;self.sent+=1;return True
        except Exception as exc:
            self.errors+=1;self.abandon(now)
            if self.errors==1:print('[CAM2 TX]',exc,flush=True)
            return False
    def result(self,packet,now):
        if not isinstance(packet,bytes) or len(packet)!=32:raise ValueError('QR result must be 32 bytes')
        if not self.ready(now):return False
        self.original=(packet,'qr');self.pending,self.started=packet,now
        return self.pump(now)


def encode_set(mode,station=4,hz=20,sequence=0):
    return make_frame(SET,sequence,0,False,bytes((mode,station,hz,0)))


def validate_set(packet):
    mode,station,hz,flags=packet[3:7]
    if packet[2]&0x80:return 6
    if mode not in (STOP,QR_MODE):return 1
    if packet[2]&7:return 2
    if station>6 or (mode==QR_MODE and station!=QR_STATION):return 5
    if flags or (hz!=0 if mode==STOP else not 1<=hz<=50):return 6
    return 0


class QrService:
    def __init__(self,serial,backend,clock=time.monotonic):
        self.serial,self.backend,self.clock=serial,backend,clock
        self.parser,self.writer=FrameParser(),FrameWriter(serial)
        self.config=(STOP,0,0,0,0);self.set_sequence=0;self.epoch=0
        self.last_request=self.last_ack=None;self.last_request_time=-math.inf
        self.locked_task=self.saved_display=None;self.packet=None
        self.next_heartbeat=self.next_result=self.next_preview=self.next_error_log=0.
    def request(self,packet,now):
        if packet[1]!=SET:return
        if packet==self.last_request and now-self.last_request_time<=2:
            self.writer.control(self.last_ack);return
        if len(self.writer.controls)>=7:return
        result=validate_set(packet)
        desired=(packet[3],packet[4],packet[2]&7,packet[5],packet[6])
        if result==0:
            self.writer.controls=[p for p in self.writer.controls if p[1]!='heartbeat']
            self.writer.discard_result(now)
            try:self.backend.configure(desired)
            except Exception as exc:
                result=7;print('[CAM2 CONFIG]',exc,flush=True)
                try:self.backend.configure(self.config)
                except Exception:pass
            else:
                if desired[0]==STOP:self.saved_display=self.locked_task or self.saved_display
                else:self.saved_display=None
                self.config=desired;self.set_sequence=(packet[2]>>3)&15
                self.locked_task=self.packet=None;self.epoch+=1
                self.next_result=self.next_preview=0.
                print('[CAM2 SET] seq=%d config=%s'%(self.set_sequence,desired),flush=True)
        reply=make_frame(ACK,(packet[2]>>3)&15,packet[2]&7,result==0,
            bytes((SET,result,self.config[0],self.config[1])))
        self.writer.control(reply);self.last_request,self.last_ack=packet,reply
        self.last_request_time=self.clock()
    def poll_rx(self,now):
        try:data=self.serial.read(256,timeout=0)
        except Exception as exc:
            if now>=self.next_error_log:
                print('[CAM2 RX]',exc,flush=True);self.next_error_log=now+2
            return
        for packet in self.parser.feed(data or b'',now):self.request(packet,self.clock())
    def tick(self):
        now=self.clock();self.poll_rx(now);now=self.clock()
        self.parser.expire(now);self.writer.pump(now)
        if now>=self.next_heartbeat:
            self.writer.control(make_frame(HEARTBEAT,0,0,True,bytes((1,*self.config[:2],0))),'heartbeat')
            self.next_heartbeat=now+.5
        self.writer.pump(self.clock())
        if not self.writer.ready(self.clock()):return False
        observation=None;epoch=self.epoch
        if self.config[0]==QR_MODE:
            try:observation=self.backend.observe(self.clock(),scan=self.locked_task is None)
            except Exception as exc:
                self.backend.invalidate()
                if self.clock()>=self.next_error_log:
                    print('[CAM2 OBSERVE]',exc,'\n'+traceback.format_exc(),flush=True)
                    self.next_error_log=self.clock()+2
            self.poll_rx(self.clock())
            if epoch!=self.epoch or not self.writer.ready(self.clock()):return False
            text=observation.get('task') if observation else None
            if self.locked_task is None and text is not None:
                task=parse_task(text)
                if task is not None:
                    self.locked_task=task[0];self.packet=encode_task(task[0],self.set_sequence)
                    print('[CAM2 QR] rule=%s wire_groups=%s'%(task[0],[tuple(p[3:6]) for p in [self.packet[i:i+8] for i in range(0,32,8)]]),flush=True)
            now=self.clock()
            if self.packet is not None and now>=self.next_result:
                self.writer.result(self.packet,now);self.next_result=now+1/self.config[3]
        now=self.clock()
        if self.writer.ready(now) and now>=self.next_preview:
            try:self.backend.preview(self.config,observation,self.locked_task or self.saved_display)
            except Exception as exc:
                if now>=self.next_error_log:
                    print('[CAM2 PREVIEW]',exc,flush=True);self.next_error_log=now+2
            self.next_preview=self.clock()+1/PREVIEW_HZ
        return observation is not None


class MaixQrBackend:
    def __init__(self,camera,image,display):
        self.camera,self.image,self.display=camera,image,display
        self.cam=self.frame=self.displayed=None;self.config=(STOP,0,0,0,0)
        self.tracker=TaskTracker();self.status='WAIT SET6';self.frames=0
        self.decoder=None;self.allow_decoder_keyword=True
        enum=getattr(image,'QRCodeDecoderType',None)
        if enum is not None:self.decoder=getattr(enum,'QRCODE_DECODER_TYPE_ZBAR',None)
        self.next_diagnostic=0.;self.camera_ms=self.rotate_ms=self.detect_ms=self.preview_ms=0.
    def configure(self,config):
        if config[0]==QR_MODE and self.cam is None:
            self.cam=self.camera.Camera(CAPTURE_WIDTH,CAPTURE_HEIGHT,self.image.Format.FMT_RGB888,fps=CAPTURE_FPS)
        elif config[0]==QR_MODE:
            clear=getattr(self.cam,'clear_buff',None)
            if clear is not None:clear()
        self.frame=None;self.tracker.reset();self.config=config
        self.status='SCANNING' if config[0]==QR_MODE else 'STOP'
    def observe(self,now,scan=True):
        start=time.monotonic();frame=self.cam.read(block=True,block_ms=250)
        self.camera_ms=1000*(time.monotonic()-start)
        if frame is None:raise RuntimeError('camera empty frame')
        if (frame.width(),frame.height())!=(CAPTURE_WIDTH,CAPTURE_HEIGHT):raise RuntimeError('camera geometry mismatch')
        start=time.monotonic()
        if CAMERA_ROTATION_DEG not in (0,180):raise ValueError('CAM2 rotation must be 0 or 180')
        if CAMERA_ROTATION_DEG:
            frame=frame.rotate(CAMERA_ROTATION_DEG)
            if frame is None or (frame.width(),frame.height())!=(CAPTURE_WIDTH,CAPTURE_HEIGHT):
                raise RuntimeError('CAM2 rotation failed or changed geometry')
        self.rotate_ms=1000*(time.monotonic()-start)
        self.frame=frame;self.frames+=1;task=None
        start=time.monotonic()
        if scan:
            if self.decoder is not None and self.allow_decoder_keyword:
                try:codes=frame.find_qrcodes(decoder_type=self.decoder)
                except TypeError:
                    self.allow_decoder_keyword=False;codes=frame.find_qrcodes()
            else:codes=frame.find_qrcodes()
            if len(codes)>16:
                self.tracker.reset();self.status='TOO MANY QR';return {'task':None}
            payloads=[code.payload() for code in codes]
            task=self.tracker.update(payloads);self.status=self.tracker.status
        else:self.status='LATCHED'
        self.detect_ms=1000*(time.monotonic()-start)
        if now>=self.next_diagnostic:
            print('[CAM2 PERF] frames=%d C=%.1f R=%.1f D=%.1f V=%.1f ms status=%s'%(
                self.frames,self.camera_ms,self.rotate_ms,self.detect_ms,self.preview_ms,self.status),flush=True)
            self.next_diagnostic=now+2
        return {'task':task}
    def invalidate(self):self.frame=None;self.tracker.reset();self.status='CAMERA/DECODER ERROR'
    def preview(self,config,observation,text):
        start=time.monotonic()
        if self.frame is not None and config[0]==QR_MODE:shown=self.frame.resize(PREVIEW_WIDTH,PREVIEW_HEIGHT)
        else:shown=self.image.Image(PREVIEW_WIDTH,PREVIEW_HEIGHT,self.image.Format.FMT_RGB888)
        shown.draw_rect(0,0,PREVIEW_WIDTH,57,self.image.COLOR_BLACK,thickness=-1)
        label='STOP / LAST TASK' if text and config[0]==STOP else 'LATCHED' if text else self.status
        shown.draw_string(4,4,'CAM2 QR '+label,self.image.COLOR_WHITE)
        shown.draw_string(4,24,text or 'Wait for STM32 SET6',self.image.COLOR_WHITE)
        shown.draw_string(4,42,'RULE IDs: 5 BLACK / 6 LIGHTBLUE',self.image.COLOR_WHITE)
        result=self.display.show(shown,fit=self.image.Fit.FIT_CONTAIN)
        if result is not None and int(result)!=0:raise RuntimeError('display error')
        self.displayed=shown;self.preview_ms=1000*(time.monotonic()-start)
    def close(self):
        self.frame=self.displayed=None
        if self.cam is not None:self.cam.close();self.cam=None


def run_device():
    from maix import app,camera,display,image,err
    from maix.peripheral import uart,pinmap
    serial=disp=backend=None
    try:
        for pin,function in (('A16','UART0_TX'),('A17','UART0_RX')):
            err.check_raise(pinmap.set_pin_function(pin,function),'CAM2 UART mapping failed')
        serial=uart.UART(UART_PORT,UART_BAUDRATE,databits=uart.BITS.BITS_8,
            parity=uart.PARITY.PARITY_NONE,stopbits=uart.STOP.STOP_1)
        disp=display.Display()
        try:display.set_trans_image_quality(75)
        except (AttributeError,RuntimeError):pass
        backend=MaixQrBackend(camera,image,disp);service=QrService(serial,backend)
        print('[CAM2] build=%s; rotation=%d; STOP0 QR6; ttyS0 115200; QR_GROUP32; SET seq binding'%(BUILD,CAMERA_ROTATION_DEG),flush=True)
        while not app.need_exit():
            observed=service.tick()
            if not observed:serial.available(timeout=10)
    finally:
        if backend is not None:
            try:backend.close()
            except Exception as exc:print('[CAM2 CAMERA CLOSE]',exc)
        for device in (serial,disp):
            if device is not None:
                try:device.close()
                except Exception as exc:print('[CAM2 CLOSE]',exc)


def check_photo(path):
    import cv2
    import numpy as np
    image=cv2.imdecode(np.fromfile(str(path),np.uint8),cv2.IMREAD_COLOR)
    if image is None:raise ValueError('Cannot read QR photo')
    text,_,_=cv2.QRCodeDetector().detectAndDecode(image)
    task=parse_task(text)
    print('decoded:',text,'\nrule_task:',task[0] if task else 'INVALID')
    if task:print('wire SETseq0:',encode_task(task[0],0).hex(' '))
    return task


def main(argv=None):
    # Installed MaixCAM apps run under /maixapp. Launcher parameters belong to
    # the runtime, not the optional desktop photo checker. Never argparse them.
    if Path('/maixapp').is_dir():
        print('[CAM2 ENTRY] device mode; desktop CLI parser disabled',flush=True)
        return run_device()
    parser=argparse.ArgumentParser();parser.add_argument('--image');args=parser.parse_args(argv)
    if args.image:check_photo(args.image)
    else:run_device()


if __name__=='__main__':
    main()
