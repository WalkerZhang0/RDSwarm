#!/usr/bin/python3
"""Automatic same-master retrieval -> describe -> fetch -> BF/PnP baseline."""
import io
import hashlib
import json
import sqlite3
import time
from pathlib import Path

import rospy
from rdswarm_msgs.msg import KeyframeSummary
from rdswarm_msgs.srv import (DescribeKeyframe, GetLocalFeatures, FetchPeerFeatures,
                              QueryLoopCandidates, VerifyLoop)
from rdswarm_runtime.records import EventLog, Worker, frame_path, key_text, write_message


class BaselineNode:
    def __init__(self):
        self.directory = Path(rospy.get_param('~output_directory')).resolve()
        self.log = EventLog(self.directory / 'loops.jsonl')
        self.local = rospy.get_param('~agent_id', 0)
        self.namespaces = {int(k): str(v).rstrip('/') for k, v in
                           rospy.get_param('~agent_namespaces', {'0': '/agent0', '1': '/agent1'}).items()}
        self.top_k = int(rospy.get_param('~top_k', 3))
        self.min_similarity = float(rospy.get_param('~min_similarity', -1.0))
        if self.top_k < 1 or not -1 <= self.min_similarity <= 1 or self.local not in self.namespaces:
            raise ValueError('invalid_retrieval_options')
        self.publish = rospy.get_param('~publish_constraint', True)
        self.worker = Worker(self.process, self.log, rospy.get_param('~max_pending', 128))
        self.subscriber = rospy.Subscriber('summaries_in', KeyframeSummary, self.receive, queue_size=256)
        self.log('baseline_ready', agent_id=self.local, top_k=self.top_k,
                 min_similarity=self.min_similarity, matcher='bf_ratio_mutual',
                 publish_constraint=self.publish)

    def receive(self, summary):
        if (not summary.global_descriptor_valid or len(summary.global_descriptor) != 4096 or
                summary.key.agent_id not in self.namespaces):
            self.log('summary_rejected', key=key_text(summary.key))
            return
        if not self.worker.submit(summary):
            self.log('retrieval_not_scheduled', key=key_text(summary.key))

    def call(self, agent, service, service_type, *args):
        name = self.namespaces[agent] + '/' + service
        rospy.wait_for_service(name, timeout=5)
        return rospy.ServiceProxy(name, service_type)(*args)

    def ensure_described(self, key):
        start = time.perf_counter()
        response = self.call(key.agent_id, 'describe_keyframe', DescribeKeyframe, key, True)
        if not response.success:
            raise RuntimeError('describe:' + response.status)
        # 发布描述结果与 loop 回调异步，等待实际可取回的数据再进入 fetch。 (Ref: 2026-10-28_Add_Global_Retrieval.md)
        for _ in range(60):
            ready = self.call(key.agent_id, 'get_local_features', GetLocalFeatures, key, False)
            if ready.success:
                break
            time.sleep(.05)
        else:
            raise RuntimeError('described_frame_not_received')
        buffer = io.BytesIO()
        ready.packet.serialize(buffer)
        self.log('local_features_ready', key=key_text(key), features=response.feature_count,
                 descriptor_model=response.local_descriptor_model,
                 packet_2d_bytes=len(buffer.getvalue()),
                 service_ms=(time.perf_counter()-start)*1000)

    def process(self, summary, queue_ms):
        with sqlite3.connect(str(self.directory / 'attempts.sqlite')) as database:
            database.execute('CREATE TABLE IF NOT EXISTS finished (pair TEXT PRIMARY KEY, success INTEGER)')
            query = None
            for _ in range(60):
                query = self.call(self.local, 'query_candidates', QueryLoopCandidates,
                                  summary.key, self.top_k, True, 0.0)
                if query.success:
                    break
                if query.status != 'query_global_descriptor_missing':
                    raise RuntimeError(query.status)
                time.sleep(.05)
            if not query.success:
                raise RuntimeError('query_index_not_ready')
            self.log('retrieval', key=key_text(summary.key), queue_ms=queue_ms,
                     candidates=len(query.candidates), trigger='keyframe')
            for rank, candidate in enumerate(query.candidates):
                if candidate.similarity < self.min_similarity:
                    continue
                a, b = candidate.query_key, candidate.reference_key
                if b.agent_id == self.local:
                    a, b = b, a
                if a.agent_id != self.local or b.agent_id == self.local:
                    continue
                pair = json.dumps([key_text(a), key_text(b)])
                if database.execute('SELECT 1 FROM finished WHERE pair=?', (pair,)).fetchone():
                    continue
                self.log('candidate', pair=pair, rank=rank+1, similarity=float(candidate.similarity),
                         retrieval_model=candidate.retrieval_model)
                start = time.perf_counter()
                try:
                    self.ensure_described(a)
                    self.ensure_described(b)
                    fetch_start = time.perf_counter()
                    fetched = self.call(self.local, 'fetch_peer_features', FetchPeerFeatures,
                                        b, self.namespaces[b.agent_id] + '/get_local_features', False)
                    if not fetched.success:
                        raise RuntimeError('fetch:' + fetched.status)
                    fetch_ms = (time.perf_counter()-fetch_start)*1000
                    verify_start = time.perf_counter()
                    result = self.call(self.local, 'verify_loop', VerifyLoop, a, b, self.publish)
                    verify_ms = (time.perf_counter()-verify_start)*1000
                    self.log('verification', pair=pair, rank=rank+1,
                             similarity=float(candidate.similarity), success=result.success,
                             status=result.status, matches=len(result.correspondences.matches),
                             accepted_inliers=result.constraint.inlier_count,
                             covariance_valid=result.constraint.covariance_valid,
                             fetch_ms=fetch_ms, verify_service_ms=verify_ms,
                             total_ms=(time.perf_counter()-start)*1000)
                    if result.success:
                        output = self.directory / 'constraints' / (hashlib.sha256(pair.encode()).hexdigest()+'.bin')
                        write_message(output, result.constraint)
                        write_message(output.with_suffix('.matches.bin'), result.correspondences)
                        local_packet=self.call(self.local,'get_local_features',GetLocalFeatures,a,True)
                        peer_packet=self.call(b.agent_id,'get_local_features',GetLocalFeatures,b,False)
                        if not local_packet.success or not peer_packet.success:
                            raise RuntimeError('verified_bundle_packet_unavailable')
                        write_message(output.with_suffix('.local.bin'),local_packet.packet)
                        write_message(output.with_suffix('.peer.bin'),peer_packet.packet)
                        output.with_suffix('.provenance.json').write_text(json.dumps(dict(
                            observation_id=result.constraint.observation_id,
                            local_key=key_text(a),remote_key=key_text(b),
                            local_stamp_ns=local_packet.packet.header.stamp.to_nsec(),
                            remote_stamp_ns=peer_packet.packet.header.stamp.to_nsec(),
                            geometry_source='vio_conditioned_3d_not_ground_truth',
                            landmark_tracks_available=False,joint_covariance_available=False,
                            calibration_fingerprint_available=False,
                            status='reserved_for_covisibility_review_no_reprojection_factors'),indent=2)+'\n')
                    database.execute('INSERT INTO finished VALUES (?,?)', (pair, int(result.success)))
                    database.commit()
                except Exception as error:
                    self.log('candidate_error', pair=pair, error=str(error))


if __name__ == '__main__':
    rospy.init_node('loop_baseline')
    node = BaselineNode()
    rospy.spin()
