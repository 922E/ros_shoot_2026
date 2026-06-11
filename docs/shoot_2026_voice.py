#!/home/abot/anaconda3/envs/py39/bin/python
# -*- coding: utf-8 -*-

import os
import subprocess
import wave

os.environ['KMP_DUPLICATE_LIB_OK'] = 'True'

import pyaudio
import rospy
from funasr import AutoModel
from std_msgs.msg import String, Int32

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
        self.model = AutoModel(model=self.model_dir, disable_update=True)

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

        rospy.Subscriber('audio_topic', String, self.audio_callback)

    def audio_callback(self, msg):
        now = rospy.Time.now()
        if self.recording or (now - self.last_trigger_time).to_sec() < 8.0:
            return
        self.last_trigger_time = now
        self.recording = True
        try:
            self.record_audio()
            text = self.recognize_audio()
            if text:
                self.chinese_pub.publish(String(data=text))
                self.parse_target_text(text)
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
        stream = audio.open(format=audio_format, channels=channels,
                            rate=rate, input=True,
                            frames_per_buffer=chunk)
        frames = []
        for _ in range(0, int(rate / chunk * seconds)):
            frames.append(stream.read(chunk))

        self.play_music(self.prompt_music_path)

        stream.stop_stream()
        stream.close()

        with wave.open(self.wav_path, 'wb') as wf:
            wf.setnchannels(channels)
            wf.setsampwidth(audio.get_sample_size(audio_format))
            wf.setframerate(rate)
            wf.writeframes(b''.join(frames))
        audio.terminate()

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

        for target_id, words in rotating_map:
            if any(word in text for word in words):
                self.call_tts('收到，打击{}号靶'.format(target_id))
                self.voice_words_pub.publish(
                    String(data='target_id_rotating {}'.format(target_id)))
                self.rotating_id_pub.publish(Int32(data=target_id))
                rospy.loginfo('publish target_id_rotating: %d', target_id)
                break

        for target_id, words in moving_map:
            if any(word in text for word in words):
                self.call_tts('收到，打击{}号靶'.format(target_id))
                self.voice_words_pub.publish(
                    String(data='target_id_moving {}'.format(target_id)))
                self.moving_id_pub.publish(Int32(data=target_id))
                rospy.loginfo('publish target_id_moving: %d', target_id)
                break


if __name__ == '__main__':
    rospy.init_node('shoot_2026_voice', anonymous=True)
    Shoot2026Voice()
    rospy.spin()