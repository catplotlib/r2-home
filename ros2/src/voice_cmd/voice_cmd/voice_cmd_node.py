import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import sounddevice as sd
import numpy as np
import tempfile
import wave
import subprocess
import time as time_module
import os
import threading
import webrtcvad
from openai import OpenAI
from scipy.signal import resample
import json
import struct

RESPONSES = {
    "forward":   "on my way",
    "backward":  "going back",
    "left":      "turning left",
    "right":     "turning right",
}

SOUND_MAP = {
    "on my way":     "/home/puja/robot_sounds/forward.wav",
    "going back":    "/home/puja/robot_sounds/backward.wav",
    "turning left":  "/home/puja/robot_sounds/left.wav",
    "turning right": "/home/puja/robot_sounds/right.wav",
    "yes":           "/home/puja/robot_sounds/awake.wav",
}

SOUNDS_TO_GENERATE = {
    "on my way":     "on my way",
    "going back":    "going back",
    "turning left":  "turning left",
    "turning right": "turning right",
    "yes":           "yes?",
}

class VoiceCmdNode(Node):
    def __init__(self):
        super().__init__('voice_cmd_node')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.awake_pub = self.create_publisher(Bool, '/robot_awake', 10)
        self.openai_client = OpenAI(api_key=os.environ.get("OPENAI_API_KEY"))

        self.last_cmd_time = 0
        self.cooldown = 2.0
        self.awake = False
        self.awake_time = 0
        self.awake_timeout = 30.0

        self.samplerate = 48000
        self.vad_samplerate = 16000
        self.frame_duration = 30
        self.frame_size = int(self.samplerate * self.frame_duration / 1000)
        self.vad_frame_size = int(self.vad_samplerate * self.frame_duration / 1000)

        self.vad = webrtcvad.Vad(2)

        self.generate_sounds()

        # find USB mic
        self.device_index = None
        for i, dev in enumerate(sd.query_devices()):
            if 'USB' in dev['name'] and dev['max_input_channels'] > 0:
                self.device_index = i
                break

        if self.device_index is None:
            self.get_logger().error('USB mic not found!')
            raise RuntimeError('USB mic not found')

        self.get_logger().info(f'Using mic: {sd.query_devices(self.device_index)["name"]}')
        self.get_logger().info('Sleeping... say "robot" to wake me up')

        self.listen_thread = threading.Thread(target=self.listen_loop, daemon=True)
        self.listen_thread.start()

    def generate_sounds(self):
        os.makedirs('/home/puja/robot_sounds', exist_ok=True)

        silence_path = '/home/puja/robot_sounds/silence.wav'
        if not os.path.exists(silence_path):
            with wave.open(silence_path, 'wb') as f:
                f.setnchannels(1)
                f.setsampwidth(2)
                f.setframerate(48000)
                f.writeframes(struct.pack('<' + 'h' * 4800, *([0] * 4800)))

        all_exist = all(
            os.path.exists(SOUND_MAP.get(key) or f'/home/puja/robot_sounds/{key}.wav')
            for key in SOUNDS_TO_GENERATE
        )

        if all_exist:
            self.get_logger().info('Voice responses already cached, skipping generation')
            return

        self.get_logger().info('Generating voice responses...')
        for key, phrase in SOUNDS_TO_GENERATE.items():
            path = SOUND_MAP.get(key) or f'/home/puja/robot_sounds/{key}.wav'
            if not os.path.exists(path):
                try:
                    response = self.openai_client.audio.speech.create(
                        model="tts-1",
                        voice="nova",
                        input=phrase
                    )
                    response.stream_to_file(path)
                    self.get_logger().info(f'Generated: {phrase}')
                except Exception as e:
                    self.get_logger().error(f'TTS error for {phrase}: {e}')
        self.get_logger().info('Voice responses ready!')

    def publish_awake(self, state: bool):
        msg = Bool()
        msg.data = state
        self.awake_pub.publish(msg)

    def listen_loop(self):
        ring_buffer = []
        triggered = False
        voiced_frames = []

        with sd.RawInputStream(
            samplerate=self.samplerate,
            blocksize=self.frame_size,
            dtype='int16',
            channels=1,
            device=self.device_index
        ) as stream:
            while True:
                frame, _ = stream.read(self.frame_size)
                frame_bytes = bytes(frame)

                frame_np = np.frombuffer(frame_bytes, dtype=np.int16)
                frame_resampled = resample(frame_np, self.vad_frame_size).astype(np.int16)
                is_speech = self.vad.is_speech(frame_resampled.tobytes(), self.vad_samplerate)

                if not triggered:
                    ring_buffer.append((frame_bytes, is_speech))
                    if len(ring_buffer) > 10:
                        ring_buffer.pop(0)
                    num_voiced = sum(1 for _, s in ring_buffer if s)
                    if num_voiced > 7:
                        triggered = True
                        voiced_frames = [f for f, _ in ring_buffer]
                        ring_buffer = []
                        self.get_logger().info('Speech detected...')
                else:
                    voiced_frames.append(frame_bytes)
                    ring_buffer.append((frame_bytes, is_speech))
                    if len(ring_buffer) > 10:
                        ring_buffer.pop(0)
                    num_unvoiced = sum(1 for _, s in ring_buffer if not s)
                    if num_unvoiced > 7:
                        triggered = False
                        ring_buffer = []
                        self.get_logger().info('Speech ended, transcribing...')
                        threading.Thread(
                            target=self.transcribe_and_handle,
                            args=(voiced_frames,),
                            daemon=True
                        ).start()
                        voiced_frames = []

    def transcribe_and_handle(self, frames):
        try:
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as f:
                with wave.open(f.name, 'wb') as wav:
                    wav.setnchannels(1)
                    wav.setsampwidth(2)
                    wav.setframerate(self.samplerate)
                    wav.writeframes(b''.join(frames))

                with open(f.name, 'rb') as audio_file:
                    result = self.openai_client.audio.transcriptions.create(
                        model="whisper-1",
                        file=audio_file
                    )

            text = result.text.lower().strip()
            if not text:
                return

            self.get_logger().info(f'Heard: {text}')

            # check awake timeout
            if self.awake and time_module.time() - self.awake_time > self.awake_timeout:
                self.awake = False
                self.publish_awake(False)
                self.get_logger().info('Timed out, going back to sleep...')

            # wake word
            if 'robot' in text:
                self.awake = True
                self.awake_time = time_module.time()
                self.publish_awake(True)
                self.get_logger().info('Wake word detected!')
                subprocess.Popen(
                    'pw-play /home/puja/robot_sounds/silence.wav && pw-play /home/puja/robot_sounds/awake.wav',
                    shell=True
                )
                remaining = text.replace('robot', '').strip().strip(',').strip()
                if remaining:
                    self.get_logger().info(f'Command in same sentence: {remaining}')
                    self.handle_command(remaining)
                return

            # handle command if awake
            if self.awake:
                self.awake_time = time_module.time()
                if time_module.time() - self.last_cmd_time > self.cooldown:
                    self.handle_command(text)
            else:
                self.get_logger().info('Not awake, ignoring...')

        except Exception as e:
            self.get_logger().error(f'Transcription error: {e}')

    def handle_command(self, text):
        try:
            response = self.openai_client.chat.completions.create(
                model="gpt-4o-mini",
                max_tokens=100,
                messages=[
                    {
                        "role": "system",
                        "content": """You are a robot command parser for a differential drive robot.
Extract a movement command from the user's speech.
Respond with ONLY a JSON object like:
{"command": "forward", "speed": 0.5, "turn": 0.0, "duration": 2.0}

Valid commands: forward, backward, left, right, stop
speed is linear velocity 0.0-1.0
turn is angular velocity -1.0 to 1.0
duration is how long to execute in seconds 0.5-15.0

Examples:
"go forward" -> {"command": "forward", "speed": 0.5, "turn": 0.0, "duration": 2.0}
"go forward a bit" -> {"command": "forward", "speed": 0.5, "turn": 0.0, "duration": 1.0}
"go forward a lot" -> {"command": "forward", "speed": 0.5, "turn": 0.0, "duration": 3.0}
"move forward for 10 seconds" -> {"command": "forward", "speed": 0.5, "turn": 0.0, "duration": 10.0}
"spin around" -> {"command": "left", "speed": 0.0, "turn": 1.0, "duration": 3.0}
"turn right a little" -> {"command": "right", "speed": 0.0, "turn": -1.0, "duration": 0.5}
"turn left a lot" -> {"command": "left", "speed": 0.0, "turn": 1.0, "duration": 2.0}
"stop" -> {"command": "stop", "speed": 0.0, "turn": 0.0, "duration": 0.0}

If no clear movement command, return {"command": "stop", "speed": 0.0, "turn": 0.0, "duration": 0.0}"""
                    },
                    {
                        "role": "user",
                        "content": text
                    }
                ]
            )

            result = json.loads(response.choices[0].message.content)
            command  = result.get("command", "stop")
            speed    = float(result.get("speed", 0.5))
            turn     = float(result.get("turn", 0.0))
            duration = min(float(result.get("duration", 1.0)), 15.0)

            msg = Twist()
            if command == "forward":
                msg.linear.x  =  speed
                msg.angular.z =  turn
            elif command == "backward":
                msg.linear.x  = -speed
                msg.angular.z =  turn
            elif command == "left":
                msg.angular.z =  abs(turn) if turn != 0.0 else 0.8
            elif command == "right":
                msg.angular.z = -abs(turn) if turn != 0.0 else -0.8
            elif command == "stop":
                msg.linear.x  = 0.0
                msg.angular.z = 0.0

            self.get_logger().info(f'Command: {command} speed={speed} turn={turn} duration={duration}')
            self.last_cmd_time = time_module.time()
            self.speak(RESPONSES.get(command, command))

            def execute():
                end_time = time_module.time() + duration
                while time_module.time() < end_time:
                    self.pub.publish(msg)
                    time_module.sleep(0.1)
                stop_msg = Twist()
                self.pub.publish(stop_msg)
                self.get_logger().info('Command complete, stopped.')

            threading.Thread(target=execute, daemon=True).start()

        except Exception as e:
            self.get_logger().error(f'GPT error: {e}')

    def speak(self, text):
        wav = SOUND_MAP.get(text)
        if wav:
            subprocess.Popen(
                f'pw-play /home/puja/robot_sounds/silence.wav && pw-play {wav}',
                shell=True
            )

    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VoiceCmdNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
