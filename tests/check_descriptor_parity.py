#!/usr/bin/env python3
"""Compare C++ CPU sampling with the original ONNX sparse descriptor output only."""
import argparse
import json
from pathlib import Path
import subprocess
import cv2
import numpy as np
import onnxruntime as rt

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--original-model",type=Path,required=True)
    parser.add_argument("--prepared-model",type=Path,required=True)
    parser.add_argument("--probe",type=Path,required=True)
    parser.add_argument("--output",type=Path,default=Path("logs/descriptor_parity"))
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    rng=np.random.RandomState(7)
    image=rng.randint(0,256,(120,160),dtype=np.uint8)
    for _ in range(12):
        x=int(rng.randint(0,140));y=int(rng.randint(0,100))
        cv2.rectangle(image,(x,y),(x+16,y+16),int(rng.randint(0,255)),2)
    cv2.imwrite(str(args.output/"image.png"),image)
    options=rt.SessionOptions();options.intra_op_num_threads=1;options.inter_op_num_threads=1
    session=rt.InferenceSession(str(args.original_model),options,providers=["CPUExecutionProvider"])
    outputs=session.run(None,{"image":image.astype(np.float32)[None,None]/255.0})
    points=outputs[0][0][:40].astype(np.float32)
    if len(points)==0:raise RuntimeError("Original model found no test points; supply a compatible model.")
    source=cv2.FileStorage(str(args.output/"points.yml"),cv2.FILE_STORAGE_WRITE)
    source.write("points",points);source.release()
    subprocess.run([str(args.probe.resolve()),str(args.prepared_model.resolve()),
        str((args.output/"image.png").resolve()),str((args.output/"result.yml").resolve()),
        str((args.output/"points.yml").resolve())],check=True)
    file=cv2.FileStorage(str(args.output/"result.yml"),cv2.FILE_STORAGE_READ)
    desc=file.getNode("descriptors").mat()
    valid=file.getNode("valid").mat().ravel();file.release()
    reference=outputs[2][0][:len(points)]
    assert valid.all() and desc.shape==reference.shape
    cosine=np.sum(desc*reference,axis=1)/(np.linalg.norm(desc,axis=1)*np.linalg.norm(reference,axis=1))
    report={"samples":len(points),"dimension":desc.shape[1],
            "max_absolute_error":float(np.max(np.abs(desc-reference))),
            "minimum_cosine":float(cosine.min()),"mean_cosine":float(cosine.mean()),
            "scope":"synthetic numerical parity, not field matching accuracy"}
    assert report["minimum_cosine"]>0.9999,report
    (args.output/"summary.json").write_text(json.dumps(report,indent=2)+"\n")
    print(json.dumps(report,indent=2))
if __name__=="__main__":
    main()
