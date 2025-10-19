#!/usr/bin/env python3
"""
Visualizer (E3) - corrected:
 - reads x,y from IDs 12,13 (safe recv)
 - builds trajectory, plots against reference
 - computes & appends KPIs to E3_results.csv (includes noise/disturbance)
 - uses VSI timing consistently
"""
import struct, sys, argparse, math, csv, os
import numpy as np
import matplotlib.pyplot as plt

PythonGateways = 'pythonGateways/'
sys.path.append(PythonGateways)

import VsiCommonPythonApi as vsiCommonPythonApi
import VsiCanPythonGateway as vsiCanPythonGateway

DEFAULT_DT = 0.02

class Visualizer:
    def __init__(self, args):
        self.componentId = 2
        self.localHost = args.server_url
        self.domain = args.domain
        self.portNum = args.port if args.port is not None else 50103

        self.traj_x = []
        self.traj_y = []

        self.path_type = args.path_type
        self.path_length = args.path_length
        self.path_x, self.path_y = self.make_reference()

        # logging
        self.Kp = args.Kp; self.Ki = args.Ki; self.Kd = args.Kd
        self.noise = args.noise; self.disturbance = args.disturbance

        # timing
        self.simulationStepNs = 0
        self.simulationStep = DEFAULT_DT
        self.totalSimulationTimeNs = 0

        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.line_robot, = self.ax.plot([], [], 'b-', label='Trajectory')
        self.line_path, = self.ax.plot(self.path_x, self.path_y, 'r--', label='Reference')
        self.ax.legend(); self.ax.grid(True)

    def make_reference(self):
        xs = np.linspace(0, self.path_length, 400)
        if self.path_type == 'straight':
            ys = np.zeros_like(xs)
        elif self.path_type == 'sine':
            ys = np.sin(xs * 0.5) * 2.0
        elif self.path_type == 'curved':
            ys = 0.5 * xs + 2.0 * np.sin(0.2 * xs)
        else:
            ys = np.zeros_like(xs)
        return xs, ys

    def mainThread(self):
        dSession = vsiCommonPythonApi.connectToServer(self.localHost, self.domain, self.portNum, self.componentId)
        vsiCanPythonGateway.initialize(dSession, self.componentId)
        vsiCommonPythonApi.waitForReset()

        self.totalSimulationTimeNs = vsiCommonPythonApi.getTotalSimulationTime()
        self.simulationStepNs = vsiCommonPythonApi.getSimulationStep()
        if self.simulationStepNs and self.simulationStepNs>0:
            self.simulationStep = self.simulationStepNs / 1e9
        else:
            self.simulationStepNs = int(DEFAULT_DT * 1e9)
            self.simulationStep = DEFAULT_DT

        nextExpectedTime = vsiCommonPythonApi.getSimulationTimeInNs()

        while vsiCommonPythonApi.getSimulationTimeInNs() < self.totalSimulationTimeNs:
            x = self.safe_recv_double(12, default=None)
            y = self.safe_recv_double(13, default=None)
            if x is not None and y is not None:
                self.traj_x.append(x); self.traj_y.append(y)

                self.line_robot.set_data(self.traj_x, self.traj_y)
                self.ax.relim(); self.ax.autoscale_view()
                plt.pause(0.01)

            # advance
            nextExpectedTime += self.simulationStepNs
            vsiCommonPythonApi.advanceSimulation(nextExpectedTime - vsiCommonPythonApi.getSimulationTimeInNs())

        # finalize and compute metrics
        plt.ioff(); plt.show()
        self.save_metrics()

    def safe_recv_double(self, cid, default=None):
        try:
            packed = vsiCanPythonGateway.recvVariableFromCanPacket(8, 0, 64, cid)
        except Exception:
            packed = None
        if not packed:
            return default
        n = struct.calcsize('=d')
        return struct.unpack('=d', packed[:n])[0]

    def save_metrics(self):
        if len(self.traj_x) == 0:
            print("Visualizer: no trajectory samples collected.")
            return
        ref_y = np.interp(self.traj_x, self.path_x, self.path_y)
        errors = np.array(self.traj_y) - ref_y

        overshoot = float(np.max(np.abs(errors)))
        final_value = float(errors[-1])
        tol = 0.05
        settling_time = len(errors)
        for i in range(len(errors)):
            if np.all(np.abs(errors[i:] - final_value) <= tol):
                settling_time = i
                break
        steady_state_error = float(np.mean(errors[-max(1, len(errors)//10):]))

        file_exists = os.path.isfile('E3_results.csv')
        with open('E3_results.csv', 'a', newline='') as f:
            w = csv.writer(f)
            if not file_exists:
                w.writerow(['Kp','Ki','Kd','Noise','Disturbance','Overshoot','SettlingTime','SteadyStateError'])
            w.writerow([self.Kp, self.Ki, self.Kd, self.noise, self.disturbance,
                        overshoot, settling_time, steady_state_error])
        print("✅ E3 metrics appended to E3_results.csv")

def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--domain', default='AF_UNIX')
    p.add_argument('--server-url', default='localhost')
    p.add_argument('--port', type=int, default=50103)
    p.add_argument('--path-type', choices=['straight','sine','curved'], default='curved')
    p.add_argument('--path-length', type=float, default=20.0)
    p.add_argument('--Kp', type=float, default=1.5)
    p.add_argument('--Ki', type=float, default=0.0)
    p.add_argument('--Kd', type=float, default=0.1)
    p.add_argument('--noise', type=float, default=0.02)
    p.add_argument('--disturbance', type=float, default=0.2)
    return p.parse_args()

def main():
    args = parse_args()
    Visualizer(args).mainThread()

if __name__ == '__main__':
    main()
