#!/usr/bin/env python3
"""One-purpose conversion: preserve weights, expose the two raw SuperPoint heads."""
import argparse
import hashlib
import json
from pathlib import Path
import onnx
from onnx import helper,TensorProto

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source",type=Path)
    parser.add_argument("destination",type=Path)
    parser.add_argument("--detector-head",default="/convPb/Conv_output_0")
    parser.add_argument("--descriptor-head",default="/convDb/Conv_output_0")
    args=parser.parse_args()
    if args.source.resolve()==args.destination.resolve():
        raise SystemExit("Source and destination must differ.")
    model=onnx.load(str(args.source))
    heads=[args.detector_head,args.descriptor_head]
    available={o for node in model.graph.node for o in node.output}
    if not set(heads)<=available:
        raise SystemExit("Requested heads absent; inspect graph and pass explicit head names.")
    needed=set(heads)
    nodes=[]
    for node in reversed(model.graph.node):
        if any(o in needed for o in node.output):
            nodes.append(node); needed.update(node.input)
    nodes.reverse()
    initializers=[v for v in model.graph.initializer if v.name in needed]
    outputs=[helper.make_tensor_value_info(heads[0],TensorProto.FLOAT,[1,65,"hc","wc"]),
             helper.make_tensor_value_info(heads[1],TensorProto.FLOAT,[1,256,"hc","wc"])]
    graph=helper.make_graph(nodes,"rdswarm_superpoint_dense",list(model.graph.input),outputs,initializers)
    converted=helper.make_model(graph,opset_imports=model.opset_import)
    converted.ir_version=model.ir_version
    for old,new in zip(heads,["detector_logits","descriptor_raw"]):
        for node in converted.graph.node:
            for values in [node.input,node.output]:
                for i,value in enumerate(values):
                    if value==old:values[i]=new
        for value in converted.graph.output:
            if value.name==old:value.name=new
    onnx.checker.check_model(converted)
    args.destination.parent.mkdir(parents=True,exist_ok=True)
    onnx.save(converted,str(args.destination))
    digest=hashlib.sha256(args.destination.read_bytes()).hexdigest()
    metadata={"source_sha256":hashlib.sha256(args.source.read_bytes()).hexdigest(),
              "model_sha256":digest,"outputs":["detector_logits","descriptor_raw"],
              "model_id":"superpoint:"+digest,"descriptor_dimension":256,"stride":8,
              "sampling":"half-pixel mapping, align_corners=true, zero padding",
              "source":str(args.source.resolve())}
    args.destination.with_suffix(".json").write_text(json.dumps(metadata,indent=2)+"\n")
    print(json.dumps(metadata,indent=2))
if __name__=="__main__":
    main()
