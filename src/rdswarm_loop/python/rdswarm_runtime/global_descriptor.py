"""D2SLAM MobileNetVLAD ONNX contract: NHWC float32 grayscale, range 0..255."""
import hashlib
import time
from pathlib import Path

import cv2
import numpy as np


class MobileNetVLAD:
    def __init__(self, model, width=320, height=240, threads=2):
        import onnxruntime as ort
        if min(width, height, threads) <= 0:
            raise ValueError('invalid_global_model_options')
        self.path = Path(model).resolve()
        self.sha256 = hashlib.sha256(self.path.read_bytes()).hexdigest()
        self.width, self.height = width, height
        self.model_id = ('mobilenetvlad:' + self.sha256 +
                         ':nhwc-gray-u8range-linear-{}x{}-l2'.format(width, height))
        options = ort.SessionOptions()
        options.intra_op_num_threads = threads
        options.inter_op_num_threads = 1
        options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
        start = time.perf_counter()
        self.session = ort.InferenceSession(str(self.path), sess_options=options,
                                            providers=['CPUExecutionProvider'])
        self.load_ms = (time.perf_counter() - start) * 1000
        inputs, outputs = self.session.get_inputs(), self.session.get_outputs()
        if len(inputs) != 1 or inputs[0].name != 'image:0' or inputs[0].type != 'tensor(float)':
            raise ValueError('unsupported_global_model_input')
        if len(inputs[0].shape) != 4 or inputs[0].shape[0] != 1 or inputs[0].shape[3] != 1:
            raise ValueError('expected_nhwc_gray_global_model')
        if len(outputs) != 1 or outputs[0].name != 'descriptor:0' or outputs[0].shape != [1, 4096]:
            raise ValueError('expected_4096d_global_descriptor')
        self.calls = 0

    def compute(self, image):
        start = time.perf_counter()
        if image.dtype != np.uint8 or image.ndim != 2 or image.size == 0:
            raise ValueError('expected_nonempty_mono8_image')
        # 模型内部含归一化；外部保持 0–255，不能套用 SP 的除以 255。 (Ref: 2026-10-28_Add_Global_Retrieval.md)
        tensor = cv2.resize(image, (self.width, self.height), interpolation=cv2.INTER_LINEAR)
        tensor = np.ascontiguousarray(tensor[None, :, :, None], dtype=np.float32)
        pre_ms = (time.perf_counter() - start) * 1000
        inference_start = time.perf_counter()
        descriptor = self.session.run(['descriptor:0'], {'image:0': tensor})[0]
        inference_ms = (time.perf_counter() - inference_start) * 1000
        if descriptor.shape != (1, 4096) or not np.isfinite(descriptor).all():
            raise ValueError('invalid_global_descriptor')
        norm = float(np.linalg.norm(descriptor))
        if norm < 1e-10:
            raise ValueError('zero_global_descriptor')
        descriptor = np.asarray(descriptor.reshape(-1) / norm, dtype=np.float32)
        timing = dict(preprocess_ms=pre_ms, inference_ms=inference_ms,
                      total_ms=(time.perf_counter() - start) * 1000,
                      first_inference=self.calls == 0, provider='CPUExecutionProvider')
        self.calls += 1
        return descriptor, timing


def mono8(message):
    if message.encoding not in ('mono8', '8UC1') or message.step < message.width:
        raise ValueError('mono8_image_required')
    if len(message.data) != message.height * message.step:
        raise ValueError('invalid_image_stride')
    return np.frombuffer(message.data, np.uint8).reshape(message.height, message.step)[:, :message.width]
