#!/home/abot/anaconda3/envs/py39/bin/python
# -*- coding: utf-8 -*-

import asyncio
import base64
import json
import os
import subprocess
import uuid

import rospy
import websockets
from TTS_audio.srv import StringService, StringServiceResponse


DEFAULT_WS_URL = 'wss://openspeech.bytedance.com/api/v3/tts/bidirection'
DEFAULT_API_KEY = '606448be-ddcf-40ed-86fe-2691b56a5c62'
DEFAULT_RESOURCE_ID = 'volc.service_type.10029'
DEFAULT_SPEAKER = 'zh_male_beijingxiaoye_emo_v2_mars_bigtts'
DEFAULT_SAMPLE_RATE = 24000
DEFAULT_AUDIO_FORMAT = 'mp3'
DEFAULT_PLAYER = 'mplayer'


def _as_bool(value):
    if isinstance(value, str):
        return value.strip().lower() not in ('0', 'false', 'no', 'off')
    return bool(value)


class DoubaoWebsocketTTSService(object):
    def __init__(self):
        self.ws_url = rospy.get_param('~ws_url', DEFAULT_WS_URL)
        self.ws_fallback_urls = rospy.get_param(
            '~ws_fallback_urls',
            ['wss://openspeech.bytedance.com/api/v3/tts/bidirectional'])
        self.api_key = rospy.get_param('~api_key',
                                       os.environ.get('DOUBAO_TTS_API_KEY',
                                                      DEFAULT_API_KEY))
        self.resource_id = rospy.get_param('~resource_id',
                                           DEFAULT_RESOURCE_ID)
        self.speaker = rospy.get_param('~speaker', DEFAULT_SPEAKER)
        self.sample_rate = int(rospy.get_param('~sample_rate',
                                               DEFAULT_SAMPLE_RATE))
        self.audio_format = rospy.get_param('~audio_format',
                                            DEFAULT_AUDIO_FORMAT)
        self.player = rospy.get_param('~player', DEFAULT_PLAYER)
        self.output_dir = rospy.get_param('~output_dir', '/tmp')
        self.timeout = float(rospy.get_param('~timeout', 30.0))
        self.play_realtime = _as_bool(rospy.get_param('~play_realtime',
                                                      True))

        if not os.path.isdir(self.output_dir):
            os.makedirs(self.output_dir)

        rospy.Service('tts_service', StringService, self.handle_request)
        rospy.loginfo(
            'Doubao websocket TTS ready: url=%s speaker=%s format=%s rate=%d',
            self.ws_url, self.speaker, self.audio_format, self.sample_rate)

    def handle_request(self, req):
        text = req.data.strip()
        if not text:
            return StringServiceResponse('error: empty text')

        rospy.loginfo('TTS request: %s', text)
        loop = asyncio.new_event_loop()
        try:
            asyncio.set_event_loop(loop)
            audio_path = loop.run_until_complete(self.synthesize_stream(text))
            size = os.path.getsize(audio_path)
            rospy.loginfo('TTS audio saved: %s (%d bytes)', audio_path, size)
            if size <= 0:
                return StringServiceResponse('error: empty audio file')
            return StringServiceResponse('ok: {}'.format(audio_path))
        except Exception as exc:
            rospy.logerr('TTS failed: %s', str(exc))
            return StringServiceResponse('error: {}'.format(str(exc)))
        finally:
            loop.close()

    async def synthesize_stream(self, text):
        reqid = str(uuid.uuid4())
        output_path = os.path.join(
            self.output_dir, 'tts_audio_{}.{}'.format(
                reqid, self.audio_format))
        headers = {
            'x-api-key': self.api_key,
            'X-Api-Resource-Id': self.resource_id,
            'Connection': 'keep-alive',
            'Content-Type': 'application/json',
        }
        payload = self._build_payload(text)

        player_proc = None
        if self.play_realtime:
            player_proc = subprocess.Popen(
                [self.player, '-really-quiet', '-'],
                stdin=subprocess.PIPE)

        received_bytes = 0
        try:
            urls = [self.ws_url] + [
                item for item in self.ws_fallback_urls
                if item and item != self.ws_url]
            last_error = None
            for url in urls:
                try:
                    rospy.loginfo('connecting Doubao TTS websocket: %s', url)
                    chunk_count = await self._stream_one_url(
                        url, headers, payload, output_path, player_proc)
                    received_bytes += chunk_count
                    last_error = None
                    break
                except websockets.exceptions.InvalidStatusCode as exc:
                    last_error = exc
                    rospy.logwarn('websocket rejected %s: HTTP %s',
                                  url, exc.status_code)
                    if exc.status_code != 404:
                        raise
                except Exception as exc:
                    last_error = exc
                    raise
            if last_error is not None:
                raise last_error
        finally:
            if player_proc is not None:
                if player_proc.stdin is not None:
                    try:
                        player_proc.stdin.close()
                    except IOError:
                        pass
                player_proc.wait()

        rospy.loginfo('TTS websocket stream received %d bytes',
                      received_bytes)
        if received_bytes <= 0:
            raise RuntimeError('websocket finished without audio payload')
        return output_path

    async def _stream_one_url(self, url, headers, payload, output_path,
                              player_proc):
        received_bytes = 0
        with open(output_path, 'wb') as audio_file:
            async with websockets.connect(
                    url, extra_headers=headers,
                    ping_interval=None, close_timeout=2) as ws:
                await ws.send(json.dumps(payload, ensure_ascii=False))
                while not rospy.is_shutdown():
                    try:
                        message = await asyncio.wait_for(
                            ws.recv(), timeout=self.timeout)
                    except asyncio.TimeoutError:
                        raise RuntimeError(
                            'websocket receive timeout after %.1fs' %
                            self.timeout)
                    except websockets.exceptions.ConnectionClosed:
                        break

                    done, audio_chunk = self._parse_ws_message(message)
                    if audio_chunk:
                        audio_file.write(audio_chunk)
                        received_bytes += len(audio_chunk)
                        if player_proc is not None and \
                                player_proc.stdin is not None:
                            try:
                                player_proc.stdin.write(audio_chunk)
                                player_proc.stdin.flush()
                            except IOError:
                                rospy.logwarn('mplayer stdin closed')
                                player_proc = None
                    if done:
                        break
        return received_bytes

    def _build_payload(self, text):
        additions = {
            'disable_markdown_filter': True,
            'enable_language_detector': True,
            'enable_latex_tn': True,
            'disable_default_bit_rate': True,
            'max_length_to_filter_parenthesis': 0,
            'cache_config': {
                'text_type': 1,
                'use_cache': True,
            },
        }
        return {
            'req_params': {
                'text': text,
                'speaker': self.speaker,
                'additions': json.dumps(additions, ensure_ascii=False),
                'audio_params': {
                    'format': self.audio_format,
                    'sample_rate': self.sample_rate,
                },
            },
        }

    def _parse_ws_message(self, message):
        if isinstance(message, bytes):
            stripped = message.lstrip()
            if stripped.startswith(b'{'):
                return self._parse_json_message(
                    json.loads(message.decode('utf-8')))
            if self._looks_like_mp3(message):
                return False, message
            return self._parse_binary_message(message)

        if isinstance(message, str):
            text = message.strip()
            if not text:
                return False, None
            return self._parse_json_message(json.loads(text))

        return False, None

    @staticmethod
    def _looks_like_mp3(data):
        return data.startswith(b'ID3') or \
            data.startswith(b'\xff\xfb') or \
            data.startswith(b'\xff\xf3') or \
            data.startswith(b'\xff\xf2')

    def _parse_binary_message(self, data):
        if len(data) < 4:
            raise RuntimeError('short websocket binary frame: {}'.format(
                data.hex()))

        first = data[0]
        version = first >> 4
        header_words = first & 0x0f
        header_size = header_words * 4
        message_type = data[1] >> 4
        flags = data[1] & 0x0f
        serialization = data[2] >> 4
        compression = data[2] & 0x0f

        if version == 0 or header_size < 4 or header_size > len(data):
            raise RuntimeError(
                'unknown non-audio websocket binary frame len=%d hex=%s' %
                (len(data), data[:80].hex()))

        payload = data[header_size:]

        # Volc binary TTS protocol: 0xb carries audio payload chunks.
        if message_type == 0x0b:
            if flags == 0:
                return False, None
            if len(payload) < 8:
                raise RuntimeError('invalid audio frame len=%d hex=%s' %
                                   (len(data), data[:80].hex()))
            sequence = int.from_bytes(payload[:4], 'big', signed=True)
            payload_size = int.from_bytes(payload[4:8], 'big', signed=False)
            audio_payload = payload[8:8 + payload_size]
            done = sequence < 0
            return done, audio_payload

        # 0xf is an error frame in Volc's binary protocol.
        if message_type == 0x0f:
            text = self._decode_protocol_payload(
                payload, serialization, compression)
            raise RuntimeError('TTS websocket error frame: {}'.format(text))

        # 0xc is a frontend/control response. It is not playable audio.
        if message_type == 0x0c:
            text = self._decode_protocol_payload(
                payload, serialization, compression)
            rospy.loginfo('TTS websocket control frame: %s', text[:300])
            return False, None

        raise RuntimeError(
            'unsupported websocket binary frame type=0x%x len=%d hex=%s' %
            (message_type, len(data), data[:80].hex()))

    def _decode_protocol_payload(self, payload, serialization, compression):
        # Some protocol responses prefix JSON/error text with a 4-byte length.
        if len(payload) >= 4:
            payload_len = int.from_bytes(payload[:4], 'big', signed=False)
            if 0 <= payload_len <= len(payload) - 4:
                payload = payload[4:4 + payload_len]

        if compression == 1:
            import gzip
            payload = gzip.decompress(payload)

        if serialization == 1:
            try:
                return json.dumps(json.loads(payload.decode('utf-8')),
                                  ensure_ascii=False)
            except Exception:
                pass
        return payload.decode('utf-8', 'replace')

    def _parse_json_message(self, data):
        code = data.get('code')
        if code not in (None, 0, '0'):
            raise RuntimeError('TTS API error: {}'.format(
                json.dumps(data, ensure_ascii=False)[:500]))

        audio_text = self._find_audio_string(data)
        audio_chunk = None
        if audio_text:
            audio_chunk = self._decode_audio_text(audio_text)

        done = bool(data.get('done') or data.get('is_last') or
                    data.get('finished') or data.get('end') or
                    data.get('sequence', 0) < 0)
        return done, audio_chunk

    def _decode_audio_text(self, text):
        if text.startswith('data:audio'):
            text = text.split(',', 1)[-1]
        try:
            return base64.b64decode(text, validate=True)
        except TypeError:
            return base64.b64decode(text)
        except Exception:
            return text.encode('latin1')

    def _find_audio_string(self, value):
        if isinstance(value, dict):
            for key in ('audio', 'audio_data', 'data', 'binary', 'payload',
                        'result'):
                item = value.get(key)
                if isinstance(item, str) and len(item) > 32:
                    return item
            for item in value.values():
                found = self._find_audio_string(item)
                if found:
                    return found
        elif isinstance(value, list):
            for item in value:
                found = self._find_audio_string(item)
                if found:
                    return found
        return None


def tts_server():
    rospy.init_node('tts_server')
    DoubaoWebsocketTTSService()
    rospy.spin()


if __name__ == '__main__':
    tts_server()
