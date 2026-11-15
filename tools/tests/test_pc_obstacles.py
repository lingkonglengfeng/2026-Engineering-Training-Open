"""Saved CAM2 images and fake UART only; never opens real cameras or COM ports."""
import json
from pathlib import Path
import queue
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / 'vision/pc_vision'), str(ROOT / 'vision/maixcam_service')]
import cv2
import numpy as np
from obstacle_detection import ObstacleDetector, CALIBRATION
from obstacle_link import ObstacleLink, parse_state
from serial_monitor import SerialLog

PHOTO_NAMES = ('20261027_044251_803660_108209', '20261027_044259_623267_108353',
               '20261027_044311_520135_108574', '20261027_050152_277828_129343',
               '20261027_050326_481063_131087')


def photo(index):
    path = ROOT / 'vision/pc_vision/test_data/obstacles' / (PHOTO_NAMES[index] + '_raw.png')
    if not path.exists():
        raise FileNotFoundError('Repository obstacle fixture missing: ' + str(path))
    return cv2.imdecode(np.fromfile(str(path), np.uint8), cv2.IMREAD_COLOR)


def cylinder(x=310, width=80, bottom=456):
    image = np.full((480, 640, 3), 170, np.uint8)
    top = bottom - round(width * 2.1)
    cv2.rectangle(image, (x-width//2, top+5), (x+width//2-1, bottom-5), (25,25,25), -1)
    cv2.ellipse(image, (x,top+5), (width//2,5), 0, 0, 360, (25,25,25), -1)
    cv2.ellipse(image, (x,bottom-5), (width//2,5), 0, 0, 360, (25,25,25), -1)
    return image


def state(session=10, armed=1, eligible=1):
    return '[VOBS STATE] v=1 session=%d armed=%d eligible=%d x=1000 y=500 yaw=0' % (session,armed,eligible)


def ready_link(now=1.):
    link = ObstacleLink();link.sequence=0
    link.receive(state(10),now)
    assert link.phase == 'ACTIVE'
    return link


def snapshot(now=1.05, frame=4, mode='run'):
    return {'operation':mode,'camera_role':'qr',
            'obstacle_report':{'reportable':True,'frame_key':[0,frame,frame*1000],
               'frame_pc_perf_s':now,'selected':{'front_gap_mm':250.,'body_right_mm':-100.}}}


class DetectorTests(unittest.TestCase):
    def test_five_saved_images_and_distance_anchors(self):
        widths=[]
        for i in range(5):
            candidates=ObstacleDetector.candidates(photo(i))
            self.assertEqual(len(candidates),1,PHOTO_NAMES[i]);widths.append(candidates[0]['width_px'])
        for measured,expected in zip(widths,[72.,101.,70.,78.,122.]):
            self.assertLessEqual(abs(measured-expected),1.)
        self.assertLess(abs(23900/widths[3]-300),10)
        self.assertLess(abs(23900/widths[4]-200),10)

    def test_three_distinct_frames_and_no_stale_saved_target(self):
        detector=ObstacleDetector();im=photo(3)
        for i in range(1,4):
            report=detector.update(im,(1,i,i*1000000),10+i*.05)
            self.assertEqual(report['confirmed'],i>=3)
        self.assertTrue(report['reportable'])
        same=detector.update(im,(1,3,3000000),10.2)
        self.assertEqual(same['confirmed_frames'],3)
        missing=detector.update(np.full_like(im,170),(1,4,4000000),10.25)
        self.assertIsNone(missing['selected']);self.assertFalse(missing['reportable'])

    def test_repeating_one_frame_never_confirms(self):
        detector=ObstacleDetector()
        for _ in range(9):report=detector.update(photo(3),(1,1,10),1.)
        self.assertEqual(report['confirmed_frames'],1);self.assertFalse(report['reportable'])

    def test_distinct_frame_ids_can_share_a_coarse_camera_timestamp(self):
        detector=ObstacleDetector();image=photo(3)
        for i in range(1,4):report=detector.update(image,(1,i,100),1+i*.03)
        self.assertTrue(report['confirmed']);self.assertTrue(report['reportable'])
        self.assertEqual(report['confirmed_frames'],3)
        duplicate=detector.update(image,(1,3,101),1.12)
        self.assertFalse(duplicate['reportable'])
        self.assertLessEqual(duplicate['confirmed_frames'],3)
        # A genuinely reversed camera clock remains an invalid observation.
        reversed_report=detector.update(image,(1,4,99),1.15)
        self.assertFalse(reversed_report['reportable'])

    def test_epoch_non_monotonic_or_frame_gap_reacquires(self):
        for key,stamp in [((2,4,40),1.2),((1,2,20),1.2),((1,4,40),2.)]:
            detector=ObstacleDetector();im=photo(3)
            for i in range(1,4):detector.update(im,(1,i,i*10),1+i*.03)
            report=detector.update(im,key,stamp)
            self.assertFalse(report['confirmed'])

    def test_real_far_photos_are_reported_as_extrapolated(self):
        for index in (0,2):
            detector=ObstacleDetector()
            for i in range(3):r=detector.update(photo(index),(1,i,i+1),i*.05)
            self.assertTrue(r['confirmed']);self.assertTrue(r['reportable'])
            self.assertTrue(r['selected']['extrapolated'])

    def test_fifty_pixel_real_cutout_and_noise_boundary(self):
        source=photo(3)
        cutout=cv2.resize(source[270:456,270:352],(52,118),interpolation=cv2.INTER_AREA)
        foreground=cv2.cvtColor(cutout,cv2.COLOR_BGR2GRAY)<110
        for bottom in (300,320,340,360,380,400,420):
            image=source.copy();image[280:,270:352]=170
            roi=image[bottom-118:bottom,285:337];roi[foreground]=cutout[foreground]
            detector=ObstacleDetector()
            for i in range(3):r=detector.update(image,(1,i,i+1),i*.05)
            self.assertTrue(r['reportable'],bottom)
            self.assertLessEqual(abs(r['selected']['width_px']-50),1)
            self.assertTrue(r['selected']['extrapolated'])
            self.assertAlmostEqual(r['selected']['front_gap_mm'],478,delta=11)
        # Just-below-threshold cylinder and a small dark chair leg do not report.
        for width in (49,15):
            detector=ObstacleDetector()
            for i in range(3):r=detector.update(cylinder(310,width,405),(1,i,i+1),i*.05)
            self.assertFalse(r['reportable'])

    def test_fifty_pixel_cutout_across_real_background_and_horizon(self):
        source=photo(3);mask=np.zeros(source.shape[:2],np.uint8);mask[265:460,265:355]=255
        background=cv2.inpaint(source,mask,15,cv2.INPAINT_TELEA)
        cutout=cv2.resize(source[273:457,271:350],(50,116),interpolation=cv2.INTER_AREA)
        self.assertFalse(ObstacleDetector.candidates(background))
        for bottom in (300,320,340,360,380):
            for center in range(240,601,40):
                image=background.copy();image[bottom-116:bottom,center-25:center+25]=cutout
                detector=ObstacleDetector()
                for i in range(3):r=detector.update(image,(1,i,i+1),i*.05)
                self.assertTrue(r['reportable'],(center,bottom,r))
                self.assertLessEqual(abs(r['selected']['center_u_px']-center),6)
                self.assertTrue(45<=r['selected']['width_px']<=55)
                self.assertTrue(r['selected']['extrapolated'])

    def test_connected_background_retains_cylinder_body_width(self):
        source=photo(3);mask=np.zeros(source.shape[:2],np.uint8);mask[265:460,265:355]=255
        background=cv2.inpaint(source,mask,15,cv2.INPAINT_TELEA)
        for width in (50,60,78,100):
            height=round(184*width/79)
            cutout=cv2.resize(source[273:457,271:350],(width,height),interpolation=cv2.INTER_AREA)
            for bottom in (300,320,340,360,380):
                image=background.copy();image[bottom-height:bottom,280-width//2:280-width//2+width]=cutout
                candidates=ObstacleDetector.candidates(image)
                self.assertEqual(len(candidates),1,(width,bottom,candidates))
                self.assertLessEqual(abs(candidates[0]['width_px']-width),2,(width,bottom,candidates))
                self.assertLessEqual(abs(candidates[0]['center_u_px']-280),1)

    def test_floor_horizon_tall_rectangles_and_legs_not_reported(self):
        for bottom in (300,320,340,360,380):
            for width in (50,15):
                image=np.full((480,640,3),170,np.uint8)
                cv2.rectangle(image,(310-width//2,bottom-110),(310+width//2-1,bottom),(25,25,25),-1)
                detector=ObstacleDetector()
                for i in range(3):r=detector.update(image,(1,i,i+1),i*.05)
                self.assertFalse(r['reportable'],(bottom,width))

    def test_near_extrapolation_reported_with_label(self):
        detector=ObstacleDetector()
        for i in range(3):r=detector.update(cylinder(310,140,580),(1,i,i+1),i*.05)
        self.assertTrue(r['selected']['extrapolated']);self.assertTrue(r['reportable'])
        self.assertLess(r['selected']['front_gap_mm'],200)

    def test_bottom_crop_keeps_width_lateral_and_reference(self):
        candidate=ObstacleDetector.candidates(photo(4))[0]
        self.assertTrue(candidate['bottom_clipped']);self.assertFalse(candidate['side_clipped'])
        self.assertAlmostEqual(candidate['body_right_mm'],-129.1,1)
        self.assertEqual(CALIBRATION['distance_reference'],'front_edge_to_near_wall')
        self.assertEqual(CALIBRATION['empirical_K_mm_px'],23900)

    def test_side_crop_is_never_reportable(self):
        detector=ObstacleDetector()
        for i in range(3):r=detector.update(cylinder(620,100),(1,i,i+1),i*.05)
        self.assertFalse(r['reportable'])

    def test_empty_floor_thin_legs_and_wide_panel_rejected(self):
        for im in [np.full((480,640,3),170,np.uint8),cylinder(310,15),cylinder(310,250)]:
            self.assertFalse(ObstacleDetector.candidates(im))

    def test_flat_black_rectangles_are_not_cylinders(self):
        for bottom in (455,479):
            image=np.full((480,640,3),170,np.uint8)
            cv2.rectangle(image,(270,285),(350,bottom),(25,25,25),-1)
            self.assertFalse(ObstacleDetector.candidates(image))

    def test_background_chair_bodies_above_floor_roi_rejected(self):
        image=photo(3);image[300:]=170
        self.assertFalse(ObstacleDetector.candidates(image))

    def test_five_backgrounds_with_target_removed_are_empty(self):
        for index in range(5):
            image=photo(index)
            target=ObstacleDetector.candidates(image)[0]
            x,y,w,h=target['bbox']
            mask=np.zeros(image.shape[:2],np.uint8)
            mask[275:min(480,y+h+2),max(0,x-3):min(640,x+w+3)]=255
            background=cv2.inpaint(image,mask,3,cv2.INPAINT_TELEA)
            self.assertFalse(ObstacleDetector.candidates(background),index)

    def test_nearest_candidate_selected(self):
        image=np.minimum(cylinder(180,80),cylinder(430,110,485))
        candidates=ObstacleDetector.candidates(image)
        self.assertEqual(len(candidates),2)
        self.assertGreater(candidates[0]['width_px'],candidates[1]['width_px'])

    def test_nearer_outside_corridor_does_not_hide_full_target(self):
        image=np.minimum(cylinder(310,80),cylinder(75,110,485))
        detector=ObstacleDetector()
        for i in range(3):report=detector.update(image,(1,i,i+1),i*.05)
        self.assertEqual(len(report['candidates']),2)
        self.assertLess(report['candidates'][0]['body_right_mm'],-190)
        self.assertAlmostEqual(report['selected']['center_u_px'],309.5)
        self.assertTrue(report['reportable'])

    def test_nearer_clipped_geometry_does_not_hide_complete_target(self):
        full=ObstacleDetector.candidates(photo(3))[0]
        clipped=dict(full,side_clipped=True,width_px=120.,front_gap_mm=199.,center_u_px=635.)
        detector=ObstacleDetector()
        with patch.object(ObstacleDetector,'candidates',return_value=[clipped,full]):
            for i in range(3):report=detector.update(photo(3),(1,i,i+1),i*.05)
        self.assertIs(report['selected'],full);self.assertTrue(report['reportable'])

    def test_input_pixels_untouched(self):
        detector=ObstacleDetector();image=photo(3);copy=image.copy()
        detector.update(image,(1,1,1),1.)
        self.assertTrue(np.array_equal(image,copy))


class LinkTests(unittest.TestCase):
    def test_versioned_state_required_without_arm_commands(self):
        link=ObstacleLink();link.submit(snapshot())
        self.assertIsNone(link.observation_command(1.06))
        for text in [state().replace('v=1','v=2'),state().replace('yaw=0','yaw=nan'),
                     state().replace('session=10','session=4294967296'),'[VOBS STATE] armed=1']:
            self.assertIsNone(parse_state(text,1.))
        self.assertEqual(link.phase,'WAIT_STATE')

    def test_actual_write_age_and_manual_queue_independence(self):
        link=ready_link();link.submit(snapshot(1.05))
        self.assertEqual(link.observation_command(1.14),'vobs 10 1 250 -100 90')
        self.assertIsNone(link.observation_command(1.15))

    def test_old_frame_or_old_state_is_not_sent(self):
        link=ready_link();link.submit(snapshot(1.05))
        self.assertIsNone(link.observation_command(1.30));self.assertIsNone(link.pending)
        link.submit(snapshot(2.))
        self.assertIsNone(link.observation_command(2.01))

    def test_new_competition_session_is_followed_automatically(self):
        link=ObstacleLink();link.receive(state(9,0,0),1.0)
        self.assertEqual(link.phase,'WAIT_ARMED')
        link.receive(state(10,1,0),1.1);self.assertEqual(link.phase,'WAIT_ELIGIBLE')
        link.receive(state(10,1,1),1.2);self.assertEqual(link.phase,'ACTIVE')
        link.submit(snapshot(1.21));self.assertIsNotNone(link.observation_command(1.22))

    def test_stop_blocks_late_old_session_and_new_session_recovers(self):
        link=ready_link();link.submit(snapshot());link.cancel('STOP')
        for old in (state(10,1,1),state(10,0,0),state(10,1,1)):
            link.receive(old,1.1);link.submit(snapshot(1.11))
            self.assertIsNone(link.observation_command(1.12))
        link.receive(state(11,0,0),1.2);self.assertEqual(link.phase,'WAIT_ARMED')
        link.receive(state(12,1,1),1.3);link.submit(snapshot(1.31,8))
        self.assertIsNotNone(link.observation_command(1.32))

    def test_stop_before_first_state_waits_for_unarmed_then_new_session(self):
        link=ObstacleLink();link.cancel('prepare')
        link.receive(state(20,1,1),1.0);self.assertEqual(link.phase,'WAIT_SESSION')
        link.receive(state(20,0,0),1.1)
        link.receive(state(20,1,1),1.2);self.assertEqual(link.phase,'WAIT_SESSION')
        link.receive(state(21,1,1),1.3);self.assertEqual(link.phase,'ACTIVE')

    def test_eligible_transition_requires_a_new_frame(self):
        link=ready_link();link.receive(state(10,1,0),1.1)
        link.receive(state(10,1,1),1.2);link.submit(snapshot(1.15))
        self.assertIsNone(link.observation_command(1.21))
        link.submit(snapshot(1.22,5));self.assertIsNotNone(link.observation_command(1.23))

    def test_state_after_long_gap_requires_new_frame(self):
        link=ready_link();link.submit(snapshot(2.01,4))
        self.assertIsNone(link.observation_command(2.02))
        link.receive(state(10),2.03);link.submit(snapshot(2.01,4))
        self.assertIsNone(link.observation_command(2.04))
        link.submit(snapshot(2.05,5));self.assertIsNotNone(link.observation_command(2.06))

    def test_same_state_does_not_clear_new_pending(self):
        link=ready_link();link.submit(snapshot(1.05));link.receive(state(),1.06)
        self.assertIsNotNone(link.observation_command(1.07))

    def test_reconnect_keeps_sequence_and_follows_existing_session(self):
        link=ready_link();link.submit(snapshot());first=link.observation_command(1.1)
        link.disconnected();link.connected();link.receive(state(10),2.0)
        link.submit(snapshot(2.01,8));second=link.observation_command(2.02)
        self.assertEqual(int(second.split()[2]),int(first.split()[2])+1)

    def test_new_pc_process_seeds_sequence_ahead_of_prior_session(self):
        with patch('obstacle_link.time.time',return_value=1000.):
            first=ObstacleLink()
        first.receive(state(10),1.0);first.submit(snapshot(1.05))
        first_seq=int(first.observation_command(1.06).split()[2])
        with patch('obstacle_link.time.time',return_value=1001.):
            second=ObstacleLink()
        second.receive(state(10),2.0);second.submit(snapshot(2.05))
        second_seq=int(second.observation_command(2.06).split()[2])
        self.assertGreater((second_seq-first_seq)&0xffffffff,0)
        self.assertLess((second_seq-first_seq)&0xffffffff,0x80000000)

    def test_preview_missing_or_ineligible_clears_pending(self):
        link=ready_link();link.submit(snapshot());link.submit(snapshot(mode='preview'))
        self.assertIsNone(link.pending)
        link.submit(snapshot());link.receive(state(10,1,0),1.06)
        self.assertIsNone(link.observation_command(1.07));self.assertEqual(link.phase,'WAIT_ELIGIBLE')
        link.submit(None);self.assertIsNone(link.pending)

    def test_u32_session_sequence_fits_console_packet(self):
        link=ready_link();link.expected_session=0xffffffff;link.state['session']=0xffffffff
        link.sequence=0xfffffffe;link.submit(snapshot())
        command=link.observation_command(1.1)
        self.assertLessEqual(len(command),63);self.assertIn('4294967295 4294967295',command)


class FakeUart:
    def __init__(self):
        self.incoming=queue.Queue();self.writes=[];self.opened=False;self.armed=False;self.session=20
    def open(self):self.opened=True
    def close(self):self.opened=False
    @property
    def in_waiting(self):return 1
    def write(self,data):
        text=data.decode('ascii').strip();self.writes.append((time.perf_counter(),text))
        return len(data)
    def publish(self,session=20,armed=1,eligible=1):
        self.incoming.put((state(session,armed,eligible)+'\r\n').encode())
    def read(self,size):
        try:return self.incoming.get(timeout=.01)
        except queue.Empty:return b''


class SerialIntegrationTests(unittest.TestCase):
    def wait(self,condition,seconds=2):
        until=time.perf_counter()+seconds
        while not condition() and time.perf_counter()<until:time.sleep(.005)
        self.assertTrue(condition())

    def test_fake_uart_automatic_follow_report_and_stop_command(self):
        with tempfile.TemporaryDirectory() as folder:
            device=FakeUart();logger=SerialLog(folder,serial_factory=lambda:device)
            logger.start('FAKE_ONLY',115200)
            try:
                self.wait(lambda:logger.connected)
                self.assertFalse(device.writes)
                device.publish(21)
                self.wait(lambda:logger.obstacle_status()['phase']=='ACTIVE')
                logger.update_obstacle(snapshot(time.perf_counter()))
                self.wait(lambda:any(text.startswith('vobs 21 ') for _,text in device.writes))
                commands=[text for _,text in device.writes]
                self.assertFalse(any(text in ('vobs status','vobs on') for text in commands))
                report=next(text for text in commands if text.startswith('vobs 21 '))
                self.assertLessEqual(int(report.split()[-1]),200)
                logger.send('&STOP')
                self.assertEqual(logger.obstacle_status()['phase'],'WAIT_SESSION')
                self.wait(lambda:any(text=='&STOP' for _,text in device.writes))
                self.assertFalse(any(text=='vobs retry' for _,text in device.writes))
            finally:logger.stop();logger.thread.join(2)

    def test_stop_reset_prepare_block_old_session(self):
        with tempfile.TemporaryDirectory() as folder:
            for command in ('&STOP','&dis','&off','&disable','&match reset','&prepare 1','&prepare2','vobs off'):
                logger=SerialLog(folder);logger.connected=True
                logger.obstacles.receive(state(),1)
                logger.send(command)
                self.assertEqual(logger.obstacle_status()['phase'],'WAIT_SESSION')
                logger.obstacles.receive(state(),1.1)
                self.assertIsNone(logger.obstacles.observation_command(1.2))
                queued,_=logger.outbound.get_nowait();self.assertEqual(queued,command)

    def test_manually_requested_vobs_on_still_accepts_first_state(self):
        with tempfile.TemporaryDirectory() as folder:
            logger=SerialLog(folder);logger.connected=True
            logger.send('vobs on');logger.obstacles.receive(state(20),1.0)
            self.assertEqual(logger.obstacle_status()['phase'],'ACTIVE')
            self.assertEqual(logger.outbound.get_nowait()[0],'vobs on')

    def test_late_old_state_after_manual_command_is_ignored(self):
        for command in ('&STOP','&match reset','&prepare 1'):
            with tempfile.TemporaryDirectory() as folder:
                device=FakeUart()
                logger=SerialLog(folder,serial_factory=lambda:device);logger.start('FAKE_ONLY',115200)
                try:
                    self.wait(lambda:logger.connected);device.publish(20)
                    self.wait(lambda:logger.obstacle_status()['phase']=='ACTIVE')
                    logger.send(command)
                    self.wait(lambda:any(text==command for _,text in device.writes))
                    before=len(device.writes)
                    device.publish(20)
                    time.sleep(.05)
                    logger.update_obstacle(snapshot(time.perf_counter(),9))
                    self.assertEqual(logger.obstacle_status()['phase'],'WAIT_SESSION')
                    self.assertFalse(any(text=='vobs on' or text.startswith('vobs 20 ') for _,text in device.writes[before:]))
                    device.publish(21,0,0);device.publish(22,1,1)
                    self.wait(lambda:logger.obstacle_status()['phase']=='ACTIVE')
                    logger.update_obstacle(snapshot(time.perf_counter(),10))
                    self.wait(lambda:any(text.startswith('vobs 22 ') for _,text in device.writes))
                finally:logger.stop();logger.thread.join(2)

    def test_serial_before_prepare_and_external_stop(self):
        with tempfile.TemporaryDirectory() as folder:
            device=FakeUart();logger=SerialLog(folder,serial_factory=lambda:device)
            logger.start('FAKE_ONLY',115200)
            try:
                self.wait(lambda:logger.connected)
                logger.send('&prepare 1')
                self.wait(lambda:any(text=='&prepare 1' for _,text in device.writes))
                device.publish(20,1,1);time.sleep(.03)
                self.assertEqual(logger.obstacle_status()['phase'],'WAIT_SESSION')
                device.publish(21,0,0);device.publish(22,1,1)
                self.wait(lambda:logger.obstacle_status()['phase']=='ACTIVE')
                logger.update_obstacle(snapshot(time.perf_counter()))
                self.wait(lambda:any(text.startswith('vobs 22 ') for _,text in device.writes))
                # External STOP is reported only by MCU state; no local STOP is sent.
                device.publish(23,0,0)
                self.wait(lambda:logger.obstacle_status()['phase']=='WAIT_ARMED')
                before=len(device.writes);logger.update_obstacle(snapshot(time.perf_counter(),8));time.sleep(.06)
                self.assertEqual(len(device.writes),before)
                self.assertFalse(any(text in ('vobs on','vobs status','&OK','&STOP') for _,text in device.writes))
            finally:logger.stop();logger.thread.join(2)

    def test_reconnect_to_armed_competition_keeps_sequence(self):
        with tempfile.TemporaryDirectory() as folder:
            device=FakeUart();logger=SerialLog(folder,serial_factory=lambda:device)
            logger.start('FAKE_ONLY',115200)
            try:
                self.wait(lambda:logger.connected);device.publish(21)
                self.wait(lambda:logger.obstacle_status()['phase']=='ACTIVE')
                logger.update_obstacle(snapshot(time.perf_counter(),1))
                self.wait(lambda:any(text.startswith('vobs 21 ') for _,text in device.writes))
                first=next(text for _,text in device.writes if text.startswith('vobs 21 '))
                logger.stop();logger.thread.join(2)
                logger.start('FAKE_ONLY',115200);self.wait(lambda:logger.connected)
                device.publish(21);self.wait(lambda:logger.obstacle_status()['phase']=='ACTIVE')
                logger.update_obstacle(snapshot(time.perf_counter(),2))
                self.wait(lambda:sum(text.startswith('vobs 21 ') for _,text in device.writes)==2)
                second=[text for _,text in device.writes if text.startswith('vobs 21 ')][-1]
                self.assertEqual((int(second.split()[2])-int(first.split()[2]))&0xffffffff,1)
                self.assertFalse(any(text in ('vobs on','vobs status','&OK','&STOP') for _,text in device.writes))
            finally:logger.stop();logger.thread.join(2)


class BackendAndGuiTests(unittest.TestCase):
    wait=SerialIntegrationTests.wait

    def backend(self):
        from qr_backend import PcQrBackend
        class Video:
            frames=0
            def get(self,epoch):
                self.frames+=1
                data=cv2.imencode('.jpg',photo(3),[cv2.IMWRITE_JPEG_QUALITY,95])[1].tobytes()
                return (epoch,self.frames,self.frames*1000000,640,480,100,100,100),data,0.
        return PcQrBackend(Video(),SimpleNamespace(hello={'rotation':0},epoch=0,frame=None,observation_status=lambda:{}),
                           ROOT/'vision/pc_vision/test_data/nonexistent_obstacle_samples.json',show=False)

    def test_stop_idle_fetch_confirms_and_never_annotates_raw(self):
        obj=self.backend()
        for _ in range(3):obj.idle_preview()
        self.assertTrue(obj.obstacle_report['confirmed'])
        self.assertEqual(obj.config[0],0)
        self.assertFalse(np.array_equal(obj.display_image,obj.raw_image))
        self.assertFalse(np.array_equal(obj.display_image,obj.image))
        obj.configure((0,4,0,0,0))
        self.assertIsNone(obj.obstacle_report)
        obj.idle_preview();self.assertEqual(obj.obstacle_report['confirmed_frames'],1)

    def test_qr_decoder_receives_clean_full_frame(self):
        obj=self.backend();seen=[]
        obj.configure((6,4,0,20,0))
        obj.decoder=SimpleNamespace(detectAndDecodeMulti=lambda gray:(seen.append(gray.copy()) or False,(),None,None))
        obj.observe(time.perf_counter())
        self.assertEqual(seen[0].shape,(480,640))
        self.assertTrue(np.array_equal(seen[0],cv2.cvtColor(obj.raw_image,cv2.COLOR_BGR2GRAY)))
        self.assertIsNotNone(obj.obstacle_report)
        obj.invalidate();self.assertIsNone(obj.obstacle_report)

    def test_worker_reports_without_any_tk_refresh_and_stores_no_image_in_uart_slot(self):
        import gui
        with tempfile.TemporaryDirectory() as folder:
            device=FakeUart();logger=SerialLog(folder,serial_factory=lambda:device)
            logger.start('FAKE_ONLY',115200)
            worker=gui.VisionWorker({'camera_role':'qr'},Path(folder)/'cam2.json','run')
            worker.obstacle_sink=logger.update_obstacle
            try:
                self.wait(lambda:logger.connected);device.publish(21)
                self.wait(lambda:logger.obstacle_status()['phase']=='ACTIVE')
                obj=self.backend();obj.video.mode='push-v2';obj.video.skipped=0
                for _ in range(3):obj.idle_preview();worker.publish(obj,obj.video)
                self.wait(lambda:any(text.startswith('vobs 21 ') for _,text in device.writes))
                with logger.lock:pending=dict(logger.obstacles.pending)
                self.assertNotIn('image',pending);self.assertNotIn('raw_image',pending)
                self.assertFalse(any(isinstance(value,np.ndarray) for value in pending.values()))
                worker.stop();self.assertIsNone(logger.obstacles.pending)
                self.assertFalse(any(text in ('&STOP','vobs retry') for _,text in device.writes))
            finally:worker.stop();logger.stop();logger.thread.join(2)

    def test_wireless_capability_sends_fresh_crc_frame_without_serial_duplicate(self):
        import gui,struct
        class WifiControl:
            hello={'role':'qr','rotation':0,'obstacle_uart_type':134,'obstacle_max_age_ms':200}
            epoch=0;frame=None;new_context=False
            def __init__(self):self.writes=[]
            def write(self,packet):self.writes.append((self.frame,packet));return len(packet)
            def has_new_context(self,epoch):return self.new_context
            def observation_status(self):return {}
        with tempfile.TemporaryDirectory() as folder:
            device=FakeUart();logger=SerialLog(folder,serial_factory=lambda:device)
            logger.start('FAKE_ONLY',115200)
            worker=gui.VisionWorker({'camera_role':'qr'},Path(folder)/'cam2.json','run')
            worker.obstacle_sink=logger.update_obstacle
            try:
                self.wait(lambda:logger.connected);device.publish(21)
                self.wait(lambda:logger.obstacle_status()['phase']=='ACTIVE')
                backend=self.backend();backend.control=WifiControl()
                backend.video.mode='push-v2';backend.video.skipped=0
                for _ in range(3):backend.idle_preview();worker.publish(backend,backend.video)
                self.assertEqual(len(backend.control.writes),1)
                key,packet=backend.control.writes[0]
                self.assertEqual(packet[:3],b'\x6b\x86\x00')
                self.assertEqual(struct.unpack('<Hh',packet[3:7]),(306,-106))
                self.assertEqual(packet[7],gui.qr_service.crc8(packet[:7]))
                self.assertEqual(key,backend.last_meta[:3])
                self.assertIsNone(logger.obstacles.pending)
                time.sleep(.05)
                self.assertFalse(any(text.startswith('vobs ') for _,text in device.writes))
                worker.publish(backend,backend.video)
                self.assertEqual(len(backend.control.writes),1)
                self.assertEqual(worker.snapshot()[0]['obstacle_link']['channel'],'cam2_wifi')
                self.assertEqual(worker.snapshot()[0]['obstacle_link']['sent'],1)
            finally:worker.stop();logger.stop();logger.thread.join(2)

    def test_wireless_preview_old_frame_and_context_do_not_send(self):
        import gui
        class WifiControl:
            hello={'role':'qr','rotation':0,'obstacle_uart_type':134,'obstacle_max_age_ms':200}
            epoch=0;frame=None;new_context=False
            def __init__(self):self.writes=[]
            def write(self,packet):self.writes.append(packet);return len(packet)
            def has_new_context(self,epoch):return self.new_context
            def observation_status(self):return {}
        backend=self.backend();backend.control=WifiControl()
        backend.video.mode='push-v2';backend.video.skipped=0
        preview=gui.VisionWorker({'camera_role':'qr'},Path('unused'),'preview')
        for _ in range(3):backend.idle_preview();preview.publish(backend,backend.video)
        self.assertEqual(backend.control.writes,[])
        run=gui.VisionWorker({'camera_role':'qr'},Path('unused'),'run')
        backend.obstacle_report['frame_pc_perf_s']=time.perf_counter()-.3
        run.publish(backend,backend.video);self.assertEqual(backend.control.writes,[])
        backend.idle_preview();backend.control.epoch=1
        run.publish(backend,backend.video);self.assertEqual(backend.control.writes,[])
        backend.control.epoch=0
        backend.idle_preview();backend.control.new_context=True
        run.publish(backend,backend.video);self.assertEqual(backend.control.writes,[])
        backend.control.new_context=False;backend.idle_preview()
        run.publish(backend,backend.video);self.assertEqual(len(backend.control.writes),1)
        run.stop();backend.idle_preview();run.publish(backend,backend.video)
        self.assertEqual(len(backend.control.writes),1)

    def test_real_loopback_wifi_envelope_and_bridge_uart_age(self):
        import gui,struct
        import test_pc_vision as existing
        from transport import ControlClient,VideoClient
        from qr_backend import PcQrBackend,qr_service
        # All sockets and the UART here are local fakes; no camera or COM port.
        loop=existing.Loopback(cv2.rotate(photo(3),cv2.ROTATE_180),role='qr')
        control=video=backend=None
        try:
            control=ControlClient('127.0.0.1',loop.control_port)
            self.assertEqual(control.hello.get('obstacle_uart_type'),0x86)
            video=VideoClient('127.0.0.1',loop.video_port,mode='push-v2',control=control,expected_size=(640,480))
            backend=PcQrBackend(video,control,ROOT/'vision/pc_vision/test_data/nonexistent_obstacle_samples.json',show=False)
            worker=gui.VisionWorker({'camera_role':'qr'},Path('unused'),'run')
            # Allow warm-up / skipped stream slots, but still require three
            # genuinely distinct, timely observations before any UART frame.
            for _ in range(12):
                backend.idle_preview();worker.publish(backend,video)
                if worker.wireless_sent:break
            self.assertEqual(worker.wireless_sent,1,backend.obstacle_report)
            connection,payload=loop.state.results.get(timeout=2)
            epoch,image_id,stamp=existing.wire.RESULT_META.unpack_from(payload)
            self.assertEqual((epoch,image_id,stamp),tuple(backend.last_meta[:3]))
            self.assertEqual(payload[existing.wire.RESULT_META.size:][:3],b'\x6b\x86\x00')
            uart=SimpleNamespace(write=lambda packet:len(packet))
            framed,written=existing.bridge.write_obstacle_first(loop.state,uart,connection,payload,stamp+60_000_000)
            self.assertEqual(written,8);self.assertEqual(framed[:3],b'\x6b\x86\x06')
            self.assertEqual(struct.unpack('<Hh',framed[3:7]),(306,-106))
            self.assertEqual(framed[7],qr_service.crc8(framed[:7]))
            self.assertEqual(existing.bridge.write_obstacle_first(loop.state,uart,connection,payload,stamp+200_000_001),(None,0))
            raw=bytes.fromhex('6B 86 00 2C 01 96 FF')
            golden_payload=existing.wire.RESULT_META.pack(epoch,image_id,stamp)+raw+bytes((qr_service.crc8(raw),))
            golden,written=existing.bridge.write_obstacle_first(loop.state,uart,connection,golden_payload,stamp+50_000_000)
            self.assertEqual(written,8);self.assertEqual(golden,bytes.fromhex('6B 86 05 2C 01 96 FF BE'))
            # QR packets remain an atomic four-frame group in QR mode.
            loop.state.accept_request(qr_service.encode_set(6,4,20,5),time.perf_counter())
            loop.state.remember_frame(loop.state.epoch,999,stamp)
            task=qr_service.encode_task('516+132+651+231',5)
            qr_payload=existing.wire.RESULT_META.pack(loop.state.epoch,999,stamp)+task
            self.assertEqual(loop.state.result_current(connection,qr_payload,stamp+10_000_000),task)
        finally:
            if backend:backend.close()
            if video:video.close()
            if control:control.close()
            loop.close()

    def test_gui_automatic_screenshot_metadata_and_no_implicit_stop(self):
        import tkinter as tk
        import gui
        root=tk.Tk();root.withdraw();app=None
        with tempfile.TemporaryDirectory() as folder,patch.object(gui,'HERE',Path(folder)):
            try:
                app=gui.DualApp(root,Path(folder)/'c1.json',Path(folder)/'c2.json',autoconnect=False)
                panel=next(item for item in app.panels if item.camera_role=='qr')
                self.assertFalse(hasattr(panel,'obstacle_enabled'))
                self.assertIsNone(app.serial_log.thread)
                panel.operation.set('主控联动');panel.worker=SimpleNamespace()
                app.serial_log.connected=True
                app.serial_log._record('RX',state(21),{'perf_s':time.perf_counter(),'wall_time':'test'})
                self.assertEqual(app.serial_log.obstacle_status()['phase'],'ACTIVE')
                self.assertTrue(app.serial_log.outbound.empty())
                detector=ObstacleDetector();im=photo(3)
                for i in range(3):report=detector.update(im,(1,i,i+1),time.perf_counter())
                panel.latest={'image':im.copy(),'raw_image':im.copy(),'frame':3,'obstacle_report':report,
                              'camera_role':'qr','pc_frame_perf_s':time.perf_counter(),'pc_frame_wall_time':'test'}
                panel.save_screenshot()
                data=json.loads(next((Path(folder)/'screenshots/cam2').glob('*.json')).read_text(encoding='utf-8'))
                self.assertTrue(data['obstacle_report']['confirmed'])
                self.assertEqual(data['obstacle_report']['calibration']['camera_body_right_mm'],-100)
                self.assertEqual(data['obstacle_link']['phase'],'ACTIVE')
                self.assertTrue(app.serial_log.outbound.empty())
                before=set((Path(folder)/'screenshots/cam2').glob('*.json'))
                panel.latest['obstacle_link']={'channel':'cam2_wifi','sent':7,'reason':'sent'}
                panel.save_screenshot()
                added=set((Path(folder)/'screenshots/cam2').glob('*.json'))-before
                saved=json.loads(added.pop().read_text(encoding='utf-8'))
                self.assertEqual((saved['obstacle_link']['channel'],saved['obstacle_link']['sent']),('cam2_wifi',7))
            finally:
                if app:
                    for panel in app.panels:panel.worker=None
                    app.close()
                else:root.destroy()

    def test_frame_ages_during_uart_block_and_is_discarded(self):
        class Delayed(FakeUart):
            def write(self,data):
                if data.startswith(b'echo'):time.sleep(.24)
                return super().write(data)
        with tempfile.TemporaryDirectory() as folder:
            device=Delayed();logger=SerialLog(folder,serial_factory=lambda:device)
            logger.start('FAKE_ONLY',115200)
            try:
                self.wait(lambda:logger.connected);device.publish(21)
                self.wait(lambda:logger.obstacle_status()['phase']=='ACTIVE')
                with logger.lock:
                    logger.send('echo');logger.update_obstacle(snapshot(time.perf_counter()))
                self.wait(lambda:any(text=='echo' for _,text in device.writes))
                time.sleep(.04)
                self.assertFalse(any(text.startswith('vobs 21 ') for _,text in device.writes))
            finally:logger.stop();logger.thread.join(2)


if __name__=='__main__':unittest.main()
