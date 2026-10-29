#!/usr/bin/python3
"""Copy the existing D2SLAM MobileNetVLAD model and record its exact contract."""
import argparse
import json
import shutil
from pathlib import Path

from rdswarm_runtime.global_descriptor import MobileNetVLAD


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=Path(
        '/media/walker/My PSSD/program/D2SLAM/models/netvlad_series/mobilenetvlad_dyn_size.onnx'))
    parser.add_argument('--output', type=Path, default=Path('models/mobilenetvlad.onnx'))
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if args.source.resolve() != args.output.resolve():
        shutil.copyfile(str(args.source), str(args.output))
    model = MobileNetVLAD(args.output)
    manifest = dict(source=str(args.source.resolve()), sha256=model.sha256,
        input_name='image:0', input_layout='NHWC', input_dtype='float32',
        input_channels=1, input_range=[0,255], output_name='descriptor:0', dimension=4096,
        resize='linear fixed width/height', default_width=320, default_height=240,
        reference='D2SLAM/d2frontend/include/d2frontend/CNN/mobilenetvlad_onnx.h',
        reference_commit='4c33871083e4ae97b97973eb5bd88dc6735359d7')
    args.output.with_suffix('.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(json.dumps(manifest, indent=2))


if __name__ == '__main__':
    main()
