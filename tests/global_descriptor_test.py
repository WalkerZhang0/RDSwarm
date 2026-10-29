#!/usr/bin/python3
"""Real model parity with the D2SLAM preprocessing contract and invalid-input rejection."""
import argparse
import cv2
import numpy as np
from rdswarm_runtime.global_descriptor import MobileNetVLAD


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--model',default='models/mobilenetvlad.onnx')
    parser.add_argument('--image',default='results/s3e_loop_validation/offline/0_1660900407165000000.png')
    args=parser.parse_args()
    model=MobileNetVLAD(args.model)
    image=cv2.imread(args.image,0)
    descriptor,timing=model.compute(image)
    reference=cv2.resize(image,(320,240)).astype(np.float32)[None,:,:,None]
    expected=model.session.run(['descriptor:0'],{'image:0':reference})[0].reshape(-1)
    expected/=np.linalg.norm(expected)
    assert descriptor.shape==(4096,) and abs(np.linalg.norm(descriptor)-1)<1e-6
    assert np.max(np.abs(descriptor-expected))<1e-6,'reference_preprocessing_parity'
    repeated,_=model.compute(image)
    assert np.max(np.abs(descriptor-repeated))<1e-6,'deterministic_input'
    for invalid in [image.astype(np.float32)/255,np.zeros((0,0),np.uint8),np.zeros((32,32,3),np.uint8)]:
        try:model.compute(invalid)
        except ValueError:continue
        raise AssertionError('invalid_input_accepted')
    print('PASS: real 4096-D CPU model, 0..255 NHWC reference parity, repeatability, invalid image rejection.')


if __name__=='__main__':main()
