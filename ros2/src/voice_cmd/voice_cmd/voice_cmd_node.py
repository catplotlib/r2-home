import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import vosk
import sounddevice as sd
import json

COMMANDS = {
    "forward":      (0.5,  0.0),
    "go":           (0.5,  0.0),
    "backward":     (-0.5, 0.0),
    "back":         (-0.5, 0.0),
    "reverse":      (-0.5, 0.0),
    "left":         (0.0,  0.8),
    "turn left":    (0.0,  0.8),
    "right":        (0.0, -0.8),
    "turn right":   (0.0, -0.8),
    "stop":         (0.0,  0.0),
    "halt":         (0.0,  0.0),
}

class VoiceCmdNode(Node):
    def __init__(self):
        super().__init__('voice_cmd_node')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.model = vosk.Model("/home/puja/vosk_model/model")
        self.recognizer = vosk.KaldiRecognizer(self.model, 48000)

        # find USB mic by name automatically
        device_index = None
        for i, dev in enumerate(sd.query_devices()):
            if 'USB' in dev['name'] and dev['max_input_channels'] > 0:
                device_index = i
                break

        if device_index is None:
            self.get_logger().error('USB mic not found!')
            raise RuntimeError('USB mic not found')

        self.get_logger().info(f'Using mic: {sd.query_devices(device_index)["name"]} (device {device_index})')
        self.get_logger().info('Listening for voice commands...')

        self.stream = sd.RawInputStream(
            samplerate=48000, blocksize=8000,
            dtype='int16', channels=1,
            callback=self.audio_callback,
            device=device_index)
        self.stream.start()

    def audio_callback(self, indata, frames, time, status):
        if self.recognizer.AcceptWaveform(bytes(indata)):
            result = json.loads(self.recognizer.Result())
            text = result.get("text", "").lower().strip()
            if text:
                self.get_logger().info(f'Heard: {text}')
                self.handle_command(text)

    def handle_command(self, text):
        for phrase, (linear, angular) in COMMANDS.items():
            if phrase in text:
                msg = Twist()
                msg.linear.x  = linear
                msg.angular.z = angular
                self.pub.publish(msg)
                self.get_logger().info(
                    f'Command: "{phrase}" → linear={linear} angular={angular}')
                return
        self.get_logger().info(f'No command matched for: "{text}"')

    def destroy_node(self):
        self.stream.stop()
        self.stream.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VoiceCmdNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
