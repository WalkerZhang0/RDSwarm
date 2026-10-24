#!/usr/bin/env python3
"""Start an isolated ROS master and test the real native VINS export/cache boundary."""
import argparse
import json
import os
from pathlib import Path
import socket
import subprocess
import signal
import time
import xmlrpc.client

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--port",type=int,default=11531)
    parser.add_argument("--devel-space",type=Path,default=Path("devel"))
    parser.add_argument("--engine",type=Path,help="Also verify dual-agent startup/shutdown using an existing engine path; no inference.")
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[1]
    devel=args.devel_space if args.devel_space.is_absolute() else root/args.devel_space
    logdir=root/"logs"/("interface_smoke_"+devel.name)
    logdir.mkdir(parents=True,exist_ok=True)
    env=os.environ.copy()
    env.pop("ROS_HOSTNAME",None)
    env["ROS_MASTER_URI"]=f"http://127.0.0.1:{args.port}"
    env["ROS_IP"]="127.0.0.1"
    env["ROS_LOG_DIR"]=str(logdir)
    processes=[]
    handles=[]
    result={"scope":"synthetic interface validation; no real VIO accuracy or GPU inference claim"}
    try:
        with socket.socket() as probe:
            probe.bind(("127.0.0.1",args.port))
        def start(name,command):
            handle=(logdir/f"{name}.log").open("w")
            handles.append(handle)
            process=subprocess.Popen(command,cwd=root,env=env,stdout=handle,stderr=subprocess.STDOUT)
            processes.append(process)
            return process
        master=start("roscore",["roscore","-p",str(args.port)])
        deadline=time.monotonic()+15
        while time.monotonic()<deadline:
            if master.poll() is not None:
                raise RuntimeError("isolated roscore failed; inspect logs")
            try:
                if xmlrpc.client.ServerProxy(env["ROS_MASTER_URI"]).getPid("/smoke")[0]==1:
                    break
            except (OSError,xmlrpc.client.Error):
                time.sleep(0.1)
        else:
            raise RuntimeError("isolated ROS master not ready")
        start("loop_inputs",[
            str(devel/"lib/rdswarm_loop/rdswarm_loop_input_node"),
            "__ns:=/smoke/agent0","keyframes:=/smoke/agent0/vins/keyframes"])
        if args.engine:
            if not args.engine.is_file(): raise RuntimeError("engine file does not exist")
            dual=start("dual_agent",["roslaunch","rdswarm_bringup","dual_agent.launch",
                "session_id:=startup_test",f"superpoint_engine:={args.engine.resolve()}",
                f"output_a:={root/'results/startup_agent0'}",
                f"output_b:={root/'results/startup_agent1'}"])
            expected={"/agent0/vins","/agent1/vins","/agent0/loop_inputs","/agent1/loop_inputs"}
            launch_deadline=time.monotonic()+20
            master_api=xmlrpc.client.ServerProxy(env["ROS_MASTER_URI"])
            while time.monotonic()<launch_deadline:
                if dual.poll() is not None: raise RuntimeError("dual-agent launch exited early")
                registered=set()
                for group in master_api.getSystemState("/smoke")[2]:
                    for _,nodes in group: registered.update(nodes)
                if expected<=registered: break
                time.sleep(0.1)
            else: raise RuntimeError("dual-agent nodes failed to register")
            dual.send_signal(signal.SIGINT)
            code=dual.wait(timeout=20)
            if code!=0: raise RuntimeError(f"dual-agent shutdown failed: {code}")
            result["dual_agent_startup_shutdown"]={"returncode":code,"nodes":sorted(expected),"inference_run":False}
        contract=subprocess.run(
            [str(devel/"lib/rdswarm_backend/constraint_contract_test")],
            cwd=root,env=env,text=True,capture_output=True,timeout=20)
        result["backend_contract"]={"returncode":contract.returncode,"output":contract.stdout+contract.stderr}
        if contract.returncode: raise RuntimeError("backend contract failed")
        frontend=subprocess.run(
            [str(devel/"lib/vins/rdswarm_export_contract_fixture")],
            cwd=root,env=env,text=True,capture_output=True,timeout=30)
        result["frontend_export"]={"returncode":frontend.returncode,"output":frontend.stdout+frontend.stderr}
        if frontend.returncode: raise RuntimeError("native frontend export/cache fixture failed")
        result["passed"]=True
    except Exception as exc:
        result["passed"]=False
        result["error"]=str(exc)
    finally:
        # 只回收本脚本启动的子进程，避免干扰既有 ROS master。 (Ref: 2026-10-23_Init_RDSwarm_Framework.md)
        for process in reversed(processes):
            if process.poll() is None: process.terminate()
        for process in reversed(processes):
            try: process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill(); process.wait(timeout=5)
        for handle in handles: handle.close()
        (logdir/"summary.json").write_text(json.dumps(result,indent=2,ensure_ascii=False)+"\n")
    print(json.dumps(result,indent=2,ensure_ascii=False))
    return 0 if result["passed"] else 1

if __name__=="__main__":
    raise SystemExit(main())
