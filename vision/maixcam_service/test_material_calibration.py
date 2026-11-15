"""Manual LAB UI, real calibration-file roundtrip and fail-closed serial mode."""
import json
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np
import service
from test_service import NativeImage,Clock,Serial,Backend,frames


class Touch:
    def __init__(self):self.events=[];self.closed=False
    def available(self):return bool(self.events)
    def read0(self):return self.events.pop(0)
    def close(self):self.closed=True


class Frame(NativeImage):
    def __init__(self,array=None):
        self.array=np.zeros((240,320,3),np.uint8) if array is None else array
        self.lab=[60,35,-50];self.strings=[];self.rois=[]
    def width(self):return self.array.shape[1]
    def height(self):return self.array.shape[0]
    def draw_rect(self,*args,**kwargs):pass
    def draw_string(self,x,y,text,*args):self.strings.append(text)
    def get_statistics(self,roi):
        self.rois.append(roi)
        return SimpleNamespace(l_median=lambda:self.lab[0],a_median=lambda:self.lab[1],b_median=lambda:self.lab[2])
    def copy(self):return Frame(self.array.copy())
    def binary(self,thresholds,copy=False):
        import cv2
        lab=cv2.cvtColor(self.array,cv2.COLOR_RGB2LAB).astype(float)
        lab[:,:,0]*=100/255;lab[:,:,1:]-=128
        loL,hiL,loA,hiA,loB,hiB=thresholds[0]
        mask=((lab[:,:,0]>=loL)&(lab[:,:,0]<=hiL)&(lab[:,:,1]>=loA)&(lab[:,:,1]<=hiA)&(lab[:,:,2]>=loB)&(lab[:,:,2]<=hiB))
        self.array=np.repeat((mask*255).astype(np.uint8)[:,:,None],3,axis=2)


class MaterialCalibrationTests(unittest.TestCase):
    def module(self):return service.detector_module('material')
    def three_points(self,ui,frame):
        ui.action('learn',frame)
        for point in ((130,110),(155,130),(180,145)):ui.action('surface',frame,point)
    def backend(self,touch):
        camera=SimpleNamespace(read=lambda **kw:Frame(),clear_buff=lambda:None,close=lambda:None)
        api=SimpleNamespace(Format=SimpleNamespace(FMT_RGB888='rgb'),Fit=SimpleNamespace(FIT_CONTAIN='contain'),
            COLOR_WHITE=1,COLOR_BLACK=0,Image=lambda *args:Frame())
        display=SimpleNamespace(width=lambda:368,height=lambda:552,show=lambda *args,**kwargs:0)
        return service.MaixBackend(SimpleNamespace(Camera=lambda *args,**kwargs:camera),api,display)

    def test_calibration_flag_context_and_legacy_ring_unchanged(self):
        for target in range(7):
            self.assertEqual(service.validate_set(service.encode_set(2,1,target,20,7,4)),0)
        for mode,station,target,flag in ((2,2,1,4),(2,1,1,5),(2,1,1,6),(2,1,1,8),(3,2,2,4),(0,0,0,4),(8,1,0,4)):
            self.assertEqual(service.validate_set(service.encode_set(mode,station,target,20 if mode else 0,7,flag)),6)
        for flags in range(4):self.assertEqual(service.validate_set(service.encode_set(3,2,2,20,7,flags)),0)

    def test_real_file_only_changes_on_save_and_reload_restores_other_colors(self):
        module=self.module();frame=Frame()
        with tempfile.TemporaryDirectory() as directory:
            path=str(Path(directory)/'maixcam_material_colors.json')
            original={'1':[[45,40,20]],'4':[[55,-40,20]]};module.save_samples(original,path)
            ui=service.MaterialCalibrationUI(module,module.load_samples(path),Touch())
            before=Path(path).read_bytes()
            with patch.object(module,'save_samples',side_effect=lambda samples:save(samples,path)):
                self.three_points(ui,frame)
                self.assertEqual(Path(path).read_bytes(),before)
                self.assertEqual(len(ui.samples['1']),3);self.assertTrue(ui.dirty)
                ui.action('save',frame)
                self.assertFalse(ui.dirty)
            reloaded=module.load_samples(path)
            self.assertEqual(reloaded['1'],[[60,35,-50]]*3)
            self.assertEqual(reloaded['4'],original['4'])
            self.assertEqual(json.loads(Path(path).read_text())['version'],1)
            self.assertFalse(Path(path+'.tmp').exists())

    def test_blue_pair_separation_glare_rejection_and_three_point_completion(self):
        module=self.module();ui=service.MaterialCalibrationUI(module,{},Touch(),3);frame=Frame()
        ui.action('learn',frame)
        frame.lab=[99,35,-50];ui.action('surface',frame,(130,110));self.assertFalse(ui.dirty)
        self.assertEqual(ui.pending_values,[])
        frame.lab=[60,35,-50]
        for point in ((130,110),(155,130),(180,145)):ui.action('surface',frame,point)
        self.assertEqual(len(ui.samples['3']),3);self.assertFalse(ui.learning)
        ui.action('surface',frame,(190,150));self.assertEqual(len(ui.samples['3']),3)
        for _ in range(3):ui.action('color',frame)
        self.assertEqual(ui.color_id,6);frame.lab=[60,-25,-35]
        self.three_points(ui,frame)
        separator,status=module.blue_separator(ui.samples)
        self.assertIsNotNone(separator);self.assertNotEqual(status,'OVERLAP')
        ui.action('color',frame);self.assertEqual(ui.color_id,1)

    def test_failed_save_keeps_dirty_ui_and_exit_does_not_claim_done(self):
        module=self.module();ui=service.MaterialCalibrationUI(module,{},Touch());frame=Frame()
        self.three_points(ui,frame)
        with patch.object(module,'save_samples',side_effect=OSError('read-only filesystem')):
            ui.action('save',frame);self.assertTrue(ui.dirty)
            ui.action('exit',frame);self.assertFalse(ui.done);self.assertIn('SAVE FAIL',ui.message)
        with patch.object(module,'save_samples') as saved:
            ui.action('exit',frame);saved.assert_called_once()
        self.assertTrue(ui.done);self.assertFalse(ui.dirty)
        count=len(ui.samples['1']);ui.action('surface',frame,(160,120));self.assertEqual(len(ui.samples['1']),count)

    def test_real_touch_contain_mapping_held_press_and_outside_frame(self):
        module=self.module();touch=Touch();ui=service.MaterialCalibrationUI(module,{},touch);frame=Frame()
        ui.action('learn',frame)
        geometry=module.fit_geometry(320,240,368,552)
        x,y=module.screen_point(180,130,320,240,geometry)
        touch.events=[(x,y,True)];ui.update(frame,368,552,0)
        self.assertEqual(frame.rois[-1],[174,124,12,12])
        touch.events=[(x,y,True)];ui.update(frame,368,552,.2)
        self.assertEqual(len(ui.pending_values),1);self.assertNotIn('1',ui.samples)
        touch.events=[(x,y,False)];ui.update(frame,368,552,.3)
        touch.events=[(x,y,True)];ui.update(frame,368,552,.5)
        self.assertEqual(len(ui.pending_values),2);self.assertNotIn('1',ui.samples)
        touch.events=[(0,0,False),(0,0,True)];ui.update(frame,368,552,.7)
        self.assertEqual(len(ui.pending_values),2)

    def test_incomplete_three_points_do_not_change_old_threshold_or_save(self):
        module=self.module();old={'1':[[45,40,20]],'4':[[55,-40,20]]}
        ui=service.MaterialCalibrationUI(module,dict(old),Touch());frame=Frame()
        before=module.calibrated_threshold(module.COLORS[0],ui.samples)
        ui.action('learn',frame)
        for point in ((130,110),(155,130)):ui.action('surface',frame,point)
        self.assertTrue(ui.learning);self.assertFalse(ui.dirty)
        self.assertEqual(ui.samples,old)
        self.assertEqual(module.calibrated_threshold(module.COLORS[0],ui.samples),before)
        with patch.object(module,'save_samples') as save_file:
            ui.action('save',frame);ui.action('exit',frame);save_file.assert_not_called()
        self.assertFalse(ui.done);self.assertIn('Finish 3',ui.message)
        ui.action('view',frame);self.assertTrue(ui.raw_view)
        ui.action('learn',frame)
        self.assertFalse(ui.learning);self.assertEqual(ui.pending_values,[]);self.assertEqual(ui.samples,old)

    def test_no_new_samples_save_never_overwrites_existing_file(self):
        module=self.module();ui=service.MaterialCalibrationUI(module,{'1':[[45,40,20]]},Touch())
        with patch.object(module,'save_samples') as save_file:
            ui.action('save',Frame());save_file.assert_not_called()
        self.assertIn('No new samples',ui.message)

    def test_three_separate_touch_presses_progress_and_no_point_distance_rule(self):
        ui=service.MaterialCalibrationUI(self.module(),{},Touch());frame=Frame()
        ui.action('learn',frame)
        image=SimpleNamespace(COLOR_WHITE=1,COLOR_BLACK=0)
        for index in range(3):
            ui.touch.events=[(160,120,True)];ui.update(frame,320,240,index*.4)
            frame.strings=[];ui.draw(frame,image)
            self.assertTrue(any('%d/3'%(index+1) in text for text in frame.strings))
            ui.touch.events=[(160,120,False)];ui.update(frame,320,240,index*.4+.2)
        self.assertFalse(ui.learning);self.assertTrue(ui.dirty)
        self.assertEqual(len(ui.samples['1']),3)
        self.assertEqual(ui.pending_points,[(160,120)]*3)
        self.assertEqual(set(ui.buttons(320,240)),{'color','view','learn','save','exit'})

    def test_view_native_style_binary_copy_never_changes_rgb_sampling_source(self):
        import cv2
        pixels=np.full((240,320,3),180,np.uint8)
        cv2.circle(pixels,(150,125),30,(20,70,180),-1)
        frame=Frame(pixels.copy())
        ui=service.MaterialCalibrationUI(self.module(),{'3':[[60,35,-50]]*3,'6':[[60,-25,-35]]*3},Touch(),3)
        image=SimpleNamespace(COLOR_WHITE=1,COLOR_BLACK=0,Format=SimpleNamespace(FMT_RGB888='rgb'))
        raw=ui.render(frame,image)
        np.testing.assert_array_equal(raw.array,pixels)
        ui.action('view',frame);masked=ui.render(frame,image)
        self.assertFalse(ui.raw_view)
        self.assertEqual(tuple(masked.array[125,150]),(255,255,255))
        self.assertEqual(tuple(masked.array[125,50]),(0,0,0))
        np.testing.assert_array_equal(frame.array,pixels)
        ui.action('learn',frame);self.assertTrue(ui.raw_view)
        shown=ui.render(frame,image)
        np.testing.assert_array_equal(shown.array,pixels)
        np.testing.assert_array_equal(frame.array,pixels)

    def test_color_change_discards_only_partial_new_group_keeps_completed_colors(self):
        ui=service.MaterialCalibrationUI(self.module(),{'4':[[50,-45,20]]},Touch());frame=Frame()
        self.three_points(ui,frame);red=list(ui.samples['1'])
        ui.action('color',frame);ui.action('learn',frame)
        ui.action('surface',frame,(130,110));ui.action('color',frame)
        self.assertEqual(ui.pending_values,[]);self.assertFalse(ui.learning)
        self.assertEqual(ui.samples['1'],red);self.assertNotIn('2',ui.samples)
        self.assertEqual(ui.samples['4'],[[50,-45,20]])

    def test_backend_touch_before_success_ack_and_never_runs_detector(self):
        module=self.module();touch=Touch();backend=self.backend(touch)
        serial,clock=Serial(),Clock();server=service.VisionService(serial,backend,clock)
        with patch.object(service,'open_calibration_touch',return_value=touch), \
             patch.object(module,'load_samples',return_value={}), \
             patch.object(module,'draw_pick_window',side_effect=AssertionError('no grab overlay in calibration')), \
             patch.object(module,'detect_material_center',side_effect=AssertionError('calibration must skip detection')):
            serial.rx.extend(service.encode_set(2,1,6,20,7,4));server.tick()
            self.assertIsNotNone(backend.material_calibration)
            self.assertEqual(backend.material_calibration.color_id,5)  # wire6 BLACK -> original rule5
            for now in (.1,.2,.3):clock.value=now;server.tick()
            packets=frames(serial.output);self.assertEqual(packets[0][1],service.ACK)
            self.assertTrue(packets[0][2]&0x80)
            pixels=[p for p in packets if p[1]==2]
            self.assertTrue(pixels);self.assertTrue(all(not p[2]&0x80 and p[3:7]==bytes(4) for p in pixels))
            backend.material_calibration.action('exit',Frame())
            self.assertEqual(server.config,(2,1,6,20,4))
            clock.value=.4;server.tick();self.assertTrue(backend.material_calibration.done)
            serial.rx.extend(service.encode_set(0,0,0,0,8));clock.value=.5;server.tick()
            self.assertTrue(touch.closed);self.assertIsNone(backend.material_calibration)
        backend.close()

    def test_service_layer_suppresses_valid_pixels_even_from_buggy_backend(self):
        serial,clock,backend=Serial(),Clock(),Backend()
        server=service.VisionService(serial,backend,clock)
        serial.rx.extend(service.encode_set(2,1,1,20,7,4));server.tick()
        pixels=[p for p in frames(serial.output) if p[1]==2]
        self.assertTrue(pixels);self.assertTrue(all(not p[2]&0x80 and p[3:7]==bytes(4) for p in pixels))
        serial.rx.extend(service.encode_set(2,1,1,20,8,0));clock.value=.1;server.tick()
        pixels=[p for p in frames(serial.output) if p[1]==2]
        self.assertTrue(pixels[-1][2]&0x80)

    def test_touch_failure_is_failed_ack_and_stop_does_not_save_pending_samples(self):
        module=self.module();backend=self.backend(Touch());serial=Serial();server=service.VisionService(serial,backend,Clock())
        with patch.object(service,'open_calibration_touch',side_effect=RuntimeError('touch unavailable')), \
             patch.object(module,'load_samples',return_value={}),patch.object(service,'report_fault'):
            server.request(service.encode_set(2,1,1,20,7,4),0);server.writer.pump(0)
        packet=frames(serial.output)[0]
        self.assertFalse(packet[2]&0x80);self.assertEqual(packet[4],7)
        touch=Touch()
        with patch.object(service,'open_calibration_touch',return_value=touch),patch.object(module,'load_samples',return_value={}):
            backend.configure((2,1,1,20,4));self.three_points(backend.material_calibration,Frame())
            with patch.object(module,'save_samples') as saved:
                backend.configure((0,0,0,0,0));saved.assert_not_called()
            self.assertTrue(touch.closed)
        backend.close()


# Capture real function outside per-test monkeypatches for atomic file roundtrip.
save=service.detector_module('material').save_samples


if __name__=='__main__':unittest.main()
