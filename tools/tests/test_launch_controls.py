"""PC launch controls use fake UART only."""
from pathlib import Path
import sys,tempfile,unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'vision/pc_vision'))
from serial_monitor import SerialLog,SerialPanel,COMMON_COMMANDS,command_packet

class LaunchControls(unittest.TestCase):
    def test_commands_and_buttons(self):
        import tkinter as tk
        root=tk.Tk();root.withdraw()
        try:
            with tempfile.TemporaryDirectory() as folder:
                log=SerialLog(folder);log.connected=True;log.session='fake'
                panel=SerialPanel(root,log,Path(folder)/'serial.json',refresh=False)
                self.assertTrue(log.outbound.empty());self.assertIsNone(log.thread)
                panel.poll()
                for button,expected in zip(panel.launch_buttons,('&prepare','&OK 1','&OK 2')):
                    button.invoke();self.assertEqual(log.outbound.get_nowait()[0],expected)
                self.assertTrue(log.outbound.empty());self.assertIsNone(log.thread)
                self.assertEqual(COMMON_COMMANDS[0][1],'&prepare')
                for _,command in COMMON_COMMANDS:command_packet(command)
                log.connected=False;panel.poll()
                for button in panel.launch_buttons:self.assertIn('disabled',button.state())
        finally:root.destroy()

if __name__=='__main__':unittest.main()
