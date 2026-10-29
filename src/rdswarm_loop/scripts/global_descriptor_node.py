#!/usr/bin/python3
"""Keyframe-triggered MobileNetVLAD CPU worker; images stay on this machine."""
import time
import threading
from collections import OrderedDict
from pathlib import Path

import numpy as np
import rospy
from sensor_msgs.msg import Image
from rdswarm_msgs.msg import Keyframe, KeyframeSummary, LoopCandidate
from rdswarm_runtime.global_descriptor import MobileNetVLAD, mono8
from rdswarm_runtime.records import EventLog, Worker, frame_path, key_text, write_message


class GlobalNode:
    def __init__(self):
        directory = Path(rospy.get_param('~cache_directory')).resolve()
        directory.mkdir(parents=True, exist_ok=True)
        self.directory = directory
        self.lock = threading.Lock()
        self.images = OrderedDict()
        self.latest_state = None
        self.last_keyframe = {}
        self.last_fallback = {}
        self.summaries = {}
        self.fallback_period = float(rospy.get_param('~fallback_period_s', 3.0))
        if self.fallback_period <= 0:
            raise ValueError('invalid_fallback_period')
        self.log = EventLog(directory / 'global.jsonl')
        self.model = MobileNetVLAD(rospy.get_param('~model_path'),
            rospy.get_param('~width', 320), rospy.get_param('~height', 240),
            rospy.get_param('~threads', 2))
        self.publisher = rospy.Publisher('summaries_out', KeyframeSummary, queue_size=100)
        self.worker = Worker(self.process, self.log, rospy.get_param('~max_pending', 64))
        self.subscriber = rospy.Subscriber('keyframes_in', Keyframe, self.receive,
                                           queue_size=8, buff_size=16 * 1024 * 1024)
        self.summary_sub = rospy.Subscriber('summaries_in', KeyframeSummary, self.summary, queue_size=256)
        self.fallback_pub = rospy.Publisher('fallback_candidates', LoopCandidate, queue_size=16)
        if rospy.get_param('~fallback_enabled', False):
            from rdswarm_msgs.msg import VioState
            self.image_sub = rospy.Subscriber('images_in', Image, self.image, queue_size=2,
                                               buff_size=16*1024*1024)
            self.state_sub = rospy.Subscriber('states_in', VioState, self.state, queue_size=16)
        self.log('model_ready', model_id=self.model.model_id, dimension=4096,
                 model_load_ms=self.model.load_ms)
        rospy.loginfo('MobileNetVLAD ready: CPU, 4096-D, keyframe triggered, %s', self.model.model_id)

    def receive(self, frame):
        if not frame.initialized or not frame.capture_stamp_exact or not frame.image_included:
            self.log('input_rejected', key=key_text(frame.key), reason='uninitialized_or_missing_image')
            return
        with self.lock:
            session = (frame.key.agent_id, frame.key.session_id)
            self.last_keyframe[session] = max(self.last_keyframe.get(session, 0),
                                               frame.header.stamp.to_nsec())
        path = frame_path(self.directory / 'raw', frame.key)
        if not path.exists():
            write_message(path, frame)
        if not self.worker.submit((path, frame.key, 'keyframe')):
            self.log('keyframe_not_scheduled', key=key_text(frame.key), archived=str(path))

    def process(self, job, queue_ms):
        path, key, trigger = job
        if trigger == 'fallback':
            with self.lock:
                session = (key.agent_id, key.session_id)
                if key.frame_id - self.last_keyframe.get(session, key.frame_id) < self.fallback_period*1e9:
                    self.log('fallback_cancelled', key=key_text(key), reason='new_keyframe')
                    return
        summary_path = frame_path(self.directory / ('summaries' if trigger=='keyframe' else 'search_summaries'), key)
        cached = summary_path.exists()
        if cached:
            summary = KeyframeSummary().deserialize(summary_path.read_bytes())
            if summary.global_descriptor_model != self.model.model_id:
                raise ValueError('cached_global_model_mismatch_use_new_cache_directory')
            timing = dict(total_ms=0, inference_ms=0, cached=True)
        else:
            frame = Keyframe().deserialize(path.read_bytes())
            if frame.image.header.stamp != frame.header.stamp:
                raise ValueError('image_capture_time_mismatch')
            descriptor, timing = self.model.compute(mono8(frame.image))
            summary = KeyframeSummary()
            summary.header, summary.key = frame.header, frame.key
            summary.image_width, summary.image_height = frame.image.width, frame.image.height
            summary.feature_count = len(frame.features)
            summary.geometry_available = any(f.position_valid for f in frame.features)
            summary.global_descriptor_valid = True
            summary.global_descriptor_model = self.model.model_id
            summary.global_descriptor = descriptor.tolist()
            write_message(summary_path, summary)
        if trigger == 'keyframe':
            self.publisher.publish(summary)
        else:
            # 超时帧只检索，不能把它的位姿测量直接转移到邻近正式关键帧。 (Ref: 2026-10-28_Add_Global_Retrieval.md)
            with self.lock:
                peers = [s for s in self.summaries.values() if s.key.agent_id != key.agent_id
                         and s.global_descriptor_model == summary.global_descriptor_model]
            ranked = sorted(((float(np.dot(s.global_descriptor, summary.global_descriptor)), s)
                             for s in peers), key=lambda item: -item[0])[:3]
            for score, peer in ranked:
                candidate = LoopCandidate()
                candidate.header = summary.header
                candidate.query_key, candidate.reference_key = summary.key, peer.key
                candidate.similarity, candidate.retrieval_model = score, summary.global_descriptor_model
                candidate.candidate_id = 'search-only:' + key_text(key) + '->' + key_text(peer.key)
                candidate.data_cutoff = rospy.Time.now()
                self.fallback_pub.publish(candidate)
                self.log('fallback_candidate', query=key_text(key), peer=key_text(peer.key),
                         similarity=score, requires_endpoint_reverification=True)
        self.log('global_descriptor', key=key_text(key), capture_ns=summary.header.stamp.to_nsec(),
                 trigger=trigger, dimension=len(summary.global_descriptor),
                 model_id=summary.global_descriptor_model, queue_ms=queue_ms,
                 pending=self.worker.jobs.qsize(), cached=cached, timing=timing)
        rospy.loginfo('stage=global_descriptor frame=%s infer_ms=%.3f queue_ms=%.3f cached=%s',
                      key_text(key), timing['inference_ms'], queue_ms, cached)

    def summary(self, summary):
        if summary.global_descriptor_valid and len(summary.global_descriptor) == 4096:
            with self.lock:
                self.summaries[key_text(summary.key)] = summary

    def image(self, image):
        with self.lock:
            self.images[image.header.stamp.to_nsec()] = image
            while len(self.images) > 32:
                self.images.popitem(last=False)
            self.maybe_fallback()

    def state(self, state):
        with self.lock:
            self.latest_state = state
            self.maybe_fallback()

    def maybe_fallback(self):
        state = self.latest_state
        if state is None or not state.initialized:
            return
        stamp = state.header.stamp.to_nsec()
        session = (state.agent_id, state.session_id)
        self.last_keyframe.setdefault(session, stamp)
        reference = max(self.last_keyframe[session], self.last_fallback.get(session, 0))
        if stamp-reference < self.fallback_period*1e9 or stamp not in self.images:
            return
        frame = Keyframe()
        frame.header, frame.image = state.header, self.images[stamp]
        frame.key.agent_id, frame.key.session_id, frame.key.frame_id = state.agent_id, state.session_id, stamp
        frame.image_included, frame.initialized, frame.capture_stamp_exact = True, True, True
        frame.body_pose_vio, frame.vio_frame_id = state.body_pose_vio, state.vio_frame_id
        path = frame_path(self.directory / 'search_only', frame.key)
        write_message(path, frame)
        if self.worker.submit((path, frame.key, 'fallback')):
            self.last_fallback[session] = stamp
            self.log('fallback_scheduled', key=key_text(frame.key), trigger='no_keyframe_timeout')


if __name__ == '__main__':
    rospy.init_node('global_descriptor')
    node = GlobalNode()
    rospy.spin()
