#!/home/abot/anaconda3/envs/py39/bin/python
# -*- coding: utf-8 -*-

import os
import subprocess
import wave

os.environ['KMP_DUPLICATE_LIB_OK'] = 'True'

import pyaudio
import rospy
from funasr import AutoModel
from std_msgs.msg import Int32, String

try:
    from TTS_audio.srv import StringService
except Exception:
    StringService = None


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
        self.tts_client = None
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
        if StringService is None:
            return
        try:
            if self.tts_client is None:
                rospy.wait_for_service('tts_service', timeout=2)
                self.tts_client = rospy.ServiceProxy('tts_service',
                                                     StringService)
            self.tts_client(text)
        except Exception as exc:
            rospy.logwarn('tts_service unavailable: %s', str(exc))

    def parse_target_text(self, text):
        rotating_map = [
            (1, ['一号', '1号', '一']),
            (2, ['二号', '2号', '二']),
            (3, ['三号', '3号', '三']),
            (4, ['四号', '4号', '四']),
            (5, ['五号', '5号', '五']),
        ]
        moving_map = [
            (6, ['六号', '6号', '六']),
            (7, ['七号', '7号', '七']),
            (8, ['八号', '8号', '八']),
        ]

        rotating_id = self.find_target_id(text, rotating_map)
        moving_id = self.find_target_id(text, moving_map)

        if rotating_id is not None:
            self.rotating_id_pub.publish(Int32(data=rotating_id))
            self.voice_words_pub.publish(String(
                data='target_id_rotating {}'.format(rotating_id)))
            rospy.loginfo('publish target_id_rotating: %d', rotating_id)
            self.call_tts(u'旋转靶 {}'.format(rotating_id))

        if moving_id is not None:
            self.moving_id_pub.publish(Int32(data=moving_id))
            self.voice_words_pub.publish(String(
                data='target_id_moving {}'.format(moving_id)))
            rospy.loginfo('publish target_id_moving: %d', moving_id)
            self.call_tts(u'移动靶 {}'.format(moving_id))

        if rotating_id is None or moving_id is None:
            rospy.logwarn('voice target IDs incomplete: rotating=%s moving=%s',
                          rotating_id, moving_id)

    @staticmethod
    def find_target_id(text, target_map):
        for target_id, words in target_map:
            if any(word in text for word in words):
                return target_id
        return None


if __name__ == '__main__':
    rospy.init_node('shoot_2026_voice')
    Shoot2026Voice()
    rospy.spin()
