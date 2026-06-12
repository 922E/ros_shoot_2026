#!/home/abot/anaconda3/envs/py39/bin/python
# -*- coding: utf-8 -*-

import asyncio
import copy
import gzip
import json
import os
import re
import subprocess
import uuid
import wave

os.environ['KMP_DUPLICATE_LIB_OK'] = 'True'

import pyaudio
import rospy
import websockets
from funasr import AutoModel
from std_msgs.msg import Int32, String


TTS_APPID = '8411427043'
TTS_TOKEN = 'gYpYcMQzB_lyHqChn44GjjkGzge4g5Dl'
TTS_CLUSTER = 'volcano_tts'
TTS_VOICE_TYPE = 'BV001_streaming'
TTS_WS_URL = 'wss://openspeech.bytedance.com/api/v1/tts/ws_binary'
TTS_HEADER = bytearray(b'\x11\x10\x11\x00')
TTS_OUTPUT_DIR = '/tmp'

TTS_REQUEST_TEMPLATE = {
    'app': {
        'appid': TTS_APPID,
        'token': TTS_TOKEN,
        'cluster': TTS_CLUSTER,
    },
    'user': {
        'uid': 'ros_shoot_2026',
    },
    'audio': {
        'voice_type': TTS_VOICE_TYPE,
        'encoding': 'mp3',
        'speed_ratio': 0.9,
        'volume_ratio': 2.0,
        'pitch_ratio': 1.0,
    },
    'request': {
        'reqid': 'uuid',
        'text': '',
        'text_type': 'plain',
        'operation': 'submit',
    },
}


async def send_tts_request(text):
    reqid = str(uuid.uuid4())
    request_json = copy.deepcopy(TTS_REQUEST_TEMPLATE)
    request_json['request']['reqid'] = reqid
    request_json['request']['text'] = text

    payload = json.dumps(request_json, ensure_ascii=False).encode('utf-8')
    payload = gzip.compress(payload)
    request = bytearray(TTS_HEADER)
    request.extend(len(payload).to_bytes(4, 'big'))
    request.extend(payload)

    headers = {'Authorization': 'Bearer; {}'.format(TTS_TOKEN)}
    output_path = os.path.join(TTS_OUTPUT_DIR,
                               'shoot_voice_tts_{}.mp3'.format(reqid))

    rospy.loginfo('[TTS] connecting legacy websocket: %s', TTS_WS_URL)
    with open(output_path, 'wb') as audio_file:
        async with websockets.connect(
                TTS_WS_URL, extra_headers=headers,
                ping_interval=None) as ws:
            await ws.send(bytes(request))
            while not rospy.is_shutdown():
                response = await ws.recv()
                if parse_tts_response(response, audio_file):
                    break

    if os.path.getsize(output_path) <= 0:
        raise RuntimeError('TTS server returned no audio data')
    return output_path


def parse_tts_response(response, audio_file):
    if not isinstance(response, bytes) or len(response) < 4:
        raise RuntimeError('invalid TTS response frame')

    header_size = response[0] & 0x0f
    message_type = response[1] >> 4
    flags = response[1] & 0x0f
    compression = response[2] & 0x0f
    payload = response[header_size * 4:]

    if message_type == 0x0b:
        if flags == 0:
            return False
        sequence = int.from_bytes(payload[:4], 'big', signed=True)
        payload_size = int.from_bytes(payload[4:8], 'big', signed=False)
        audio_file.write(payload[8:8 + payload_size])
        return sequence < 0

    if message_type == 0x0f:
        error_code = int.from_bytes(payload[:4], 'big', signed=False)
        error_size = int.from_bytes(payload[4:8], 'big', signed=False)
        error_payload = payload[8:8 + error_size]
        if compression == 1:
            error_payload = gzip.decompress(error_payload)
        raise RuntimeError('TTS server error {}: {}'.format(
            error_code,
            error_payload.decode('utf-8', errors='replace')))

    if message_type == 0x0c:
        return False

    return True


class Shoot2026Voice(object):
    def __init__(self):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.model_dir = os.path.join(script_dir, 'paraformer-zh')
        self.wav_path = os.path.join(script_dir, 'test.wav')
        self.start_music_path = os.path.join(script_dir, '比赛开始.mp3')
        self.prompt_music_path = os.path.join(script_dir, '提示音.mp3')
        rospy.loginfo('loading FunASR model: %s', self.model_dir)
        self.model = AutoModel(model=self.model_dir, disable_update=True)
        rospy.loginfo('FunASR model loaded')

        self.chinese_pub = rospy.Publisher('chinese_topic', String,
                                           queue_size=10)
        self.voice_words_pub = rospy.Publisher('/voiceWords', String,
                                               queue_size=10)
        self.rotating_id_pub = rospy.Publisher('target_id_rotating', Int32,
                                               queue_size=10)
        self.moving_id_pub = rospy.Publisher('target_id_moving', Int32,
                                             queue_size=10)
        self.recording = False
        self.last_trigger_time = rospy.Time(0)
        self.trigger_cooldown = rospy.get_param('~trigger_cooldown', 8.0)

        rospy.Subscriber('audio_topic', String, self.audio_callback)
        rospy.loginfo('shoot_2026_voice ready')

    def audio_callback(self, msg):
        if msg.data != 'start_recognition':
            rospy.logwarn('ignore unknown audio command: %s', msg.data)
            return

        now = rospy.Time.now()
        since_last = (now - self.last_trigger_time).to_sec()
        if self.recording or since_last < self.trigger_cooldown:
            rospy.logwarn('ignore duplicate voice trigger')
            return

        self.last_trigger_time = now
        self.recording = True
        try:
            self.record_audio()
            text = self.recognize_audio()
            if text:
                self.chinese_pub.publish(String(data=text))
                self.parse_target_text(text)
            else:
                rospy.logwarn('voice recognition returned empty text')
        except Exception as exc:
            rospy.logerr('voice recognition failed: %s', str(exc))
        finally:
            self.recording = False

    def play_music(self, path):
        if not os.path.exists(path):
            rospy.logwarn('music file not found: %s', path)
            return
        try:
            subprocess.call(['mplayer', path])
        except Exception as exc:
            rospy.logwarn('play music failed: %s', str(exc))

    def record_audio(self):
        chunk = 1024
        audio_format = pyaudio.paInt16
        channels = 2
        rate = 16000
        seconds = 5

        self.play_music(self.start_music_path)

        audio = pyaudio.PyAudio()
        sample_width = audio.get_sample_size(audio_format)
        stream = None
        frames = []
        try:
            stream = audio.open(format=audio_format, channels=channels,
                                rate=rate, input=True,
                                frames_per_buffer=chunk)
            for _ in range(0, int(rate / chunk * seconds)):
                frames.append(stream.read(chunk))
        finally:
            if stream is not None:
                stream.stop_stream()
                stream.close()
            audio.terminate()

        self.play_music(self.prompt_music_path)

        with wave.open(self.wav_path, 'wb') as wav_file:
            wav_file.setnchannels(channels)
            wav_file.setsampwidth(sample_width)
            wav_file.setframerate(rate)
            wav_file.writeframes(b''.join(frames))

    def recognize_audio(self):
        result = self.model.generate(input=self.wav_path)
        text = ''
        if isinstance(result, list) and result:
            text = result[0].get('text', '')
        rospy.loginfo('voice result: %s', text)
        return text

    def call_tts(self, text):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            audio_path = loop.run_until_complete(send_tts_request(text))
            rospy.loginfo('[TTS] audio saved: %s', audio_path)
            result = subprocess.call(['mplayer', audio_path])
            if result != 0:
                raise RuntimeError('mplayer exited with code {}'.format(
                    result))
            rospy.loginfo('[TTS] playback completed: %s', text)
        except Exception as exc:
            rospy.logerr('[TTS] synthesize or playback failed: %s',
                         str(exc))
        finally:
            loop.close()

    def parse_target_text(self, text):
        number_mapping = {
            '一': 1, '二': 2, '三': 3, '四': 4,
            '五': 5, '六': 6, '七': 7, '八': 8,
            '1': 1, '2': 2, '3': 3, '4': 4,
            '5': 5, '6': 6, '7': 7, '8': 8,
        }
        number_text = {
            1: '一', 2: '二', 3: '三', 4: '四',
            5: '五', 6: '六', 7: '七', 8: '八',
        }

        recognized_text = text.replace(' ', '')
        tokens = re.findall(r'[一二三四五六七八1-8]', recognized_text)
        numbers = [number_mapping[token] for token in tokens]

        rotating_id = next((value for value in numbers if 1 <= value <= 5),
                           None)
        moving_id = next((value for value in numbers if 6 <= value <= 8),
                         None)

        if rotating_id is None or moving_id is None:
            rospy.logwarn('voice target IDs incomplete: text=%s numbers=%s',
                          recognized_text, str(numbers))
            return

        # Keep target confirmation audio synchronous with the main controller:
        # publish IDs only after TTS returns, so competition_control will not
        # leave VOICE_RECV while the target announcement is still playing.
        voice_text = '旋转靶为{}号，移动靶为{}号'.format(
            number_text[rotating_id], number_text[moving_id])
        self.voice_words_pub.publish(String(data=voice_text))
        self.call_tts(voice_text)

        self.rotating_id_pub.publish(Int32(data=rotating_id))
        self.voice_words_pub.publish(String(
            data='target_id_rotating {}'.format(rotating_id)))
        rospy.loginfo('publish target_id_rotating: %d', rotating_id)

        self.moving_id_pub.publish(Int32(data=moving_id))
        self.voice_words_pub.publish(String(
            data='target_id_moving {}'.format(moving_id)))
        rospy.loginfo('publish target_id_moving: %d', moving_id)

if __name__ == '__main__':
    rospy.init_node('shoot_2026_voice')
    Shoot2026Voice()
    rospy.spin()
