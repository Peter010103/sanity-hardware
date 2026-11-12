import os
import json
import time
from pathlib import Path
import signal
import tempfile
import unittest
from unittest.mock import patch

from sanity_zenoh import experiment

try:
    from rclpy.context import Context
except ImportError:
    Context = None

ROOT = Path(__file__).resolve().parents[6]


@unittest.skipIf(Context is None, 'ROS2 environment is required')
class ExperimentTests(unittest.TestCase):
    def test_physical_launcher_arguments_parse_in_ros_for_mpc_profiles(self):
        class CommandCaptured(Exception):
            pass

        cases = [('mpc', '1.5', '1', []),
            ('mpc', '6', '8', ['--ramp-time','4','--max-reference-acceleration','5',
                '--max-reference-tilt','30','--fallback-speed-limit','7.5']),
            ('mpc', '5', '2', ['--config',str(ROOT/'Sanity/src/ros2_sanity_ws/src/sanity_zenoh/config/sanity09_mpc_5ms.yaml'),
                '--mpc-config',str(ROOT/'Sanity/src/ros2_sanity_ws/src/sanity_zenoh/config/sanity09_mpc_high_bank.json')])]
        for controller, speed, radius, extra in cases:
            with self.subTest(controller=controller,speed=speed), tempfile.TemporaryDirectory() as directory:
                output = Path(directory)/'recording'
                commands = []
                recorder = unittest.mock.Mock()
                recorder.returncode = 0
                recorder.poll.return_value = 0
                recorder.wait.return_value = 0

                def capture(command, **kwargs):
                    if command[:3] == ['ros2', 'bag', 'record']:
                        (output/'mocap').mkdir()
                        (output/'mocap'/'test.mcap').touch()
                        return recorder
                    commands.append(command)
                    raise CommandCaptured()

                handlers = {signum:signal.getsignal(signum) for signum in (signal.SIGINT, signal.SIGTERM)}
                try:
                    clock_receipt={'passed':True,'server':'10.0.0.251','samples':[{'conservative_absolute_offset_ms':.2}]*3,'verified_monotonic_ns':time.monotonic_ns()}
                    with patch.dict(os.environ,{'SANITY_CLOCK_PREFLIGHT_JSON':json.dumps(clock_receipt)}),patch.object(experiment.subprocess, 'Popen', side_effect=capture):
                        with self.assertRaises(CommandCaptured):
                            experiment.main(['--workspace', str(ROOT), '--output', str(output),
                                '--controller', controller, '--fly', '--frames-confirmed',
                                '--mission', 'circle', '--speed', speed, '--radius', radius, '--laps', '5', *extra])
                finally:
                    for signum, handler in handlers.items():
                        signal.signal(signum, handler)
                self.assertEqual(len(commands), 1)
                command = commands[0]
                args = command[command.index('--ros-args'):]
                context = Context()
                try:
                    with patch.dict(os.environ, {'ROS_LOCALHOST_ONLY':'1', 'ROS_DOMAIN_ID':'179'}):
                        context.init(args=args)
                    self.assertTrue(context.ok())
                finally:
                    if context.ok():
                        context.shutdown()
                    context.destroy()
                self.assertIn('enable_output:=true', args)
                if controller == 'policy':
                    self.assertTrue(any(value.startswith('checkpoint:=') for value in args))
                else:
                    self.assertFalse(any(value.startswith('checkpoint:=') for value in args))


if __name__ == '__main__':
    unittest.main()
