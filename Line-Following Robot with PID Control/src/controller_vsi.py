#!/usr/bin/env python3
"""
Controller (E3) - corrected:
 - reads simulator state on IDs 12..14
 - publishes commands on IDs 16..17
 - uses VSI timing for dt (derived from getSimulationStep)
 - unchanged control law otherwise
"""
import struct, sys, argparse, math

PythonGateways = 'pythonGateways/'
sys.path.append(PythonGateways)

import VsiCommonPythonApi as vsiCommonPythonApi
import VsiCanPythonGateway as vsiCanPythonGateway

DEFAULT_DT = 0.02


class Controller:
    def __init__(self, args):
        self.componentId = 1
        self.localHost = args.server_url
        self.domain = args.domain
        self.portNum = args.port if args.port is not None else 50102

        # Gains (you can override via CLI args if desired)
        self.Kp_lat = args.Kp_lat
        self.Kp_head = args.Kp_head
        self.Kd_lat = args.Kd_lat
        self.v_nom = args.v_nom

        self.prev_lat_err = 0.0

        # timing
        self.simulationStepNs = 0
        self.simulationStep = DEFAULT_DT
        self.totalSimulationTimeNs = 0

        self.path_type = args.path_type

    def reference_path(self, x):
        if self.path_type == "straight":
            return 0.0, 0.0
        elif self.path_type == "sine":
            y = 2.0 * math.sin(0.5 * x)
            dy_dx = 2.0 * 0.5 * math.cos(0.5 * x)
            return y, dy_dx
        elif self.path_type == "curved":
            # same curved model as visualizer/simulator interpretation (if used)
            y = 0.5 * x + 2.0 * math.sin(0.2 * x)
            dy_dx = 0.5 + 2.0 * 0.2 * math.cos(0.2 * x)
            return y, dy_dx
        else:
            return 0.0, 0.0

    def mainThread(self):
        dSession = vsiCommonPythonApi.connectToServer(self.localHost, self.domain, self.portNum, self.componentId)
        vsiCanPythonGateway.initialize(dSession, self.componentId)
        vsiCommonPythonApi.waitForReset()

        # timing
        self.totalSimulationTimeNs = vsiCommonPythonApi.getTotalSimulationTime()
        self.simulationStepNs = vsiCommonPythonApi.getSimulationStep()
        if self.simulationStepNs and self.simulationStepNs > 0:
            self.simulationStep = self.simulationStepNs / 1e9
        else:
            self.simulationStepNs = int(DEFAULT_DT * 1e9)
            self.simulationStep = DEFAULT_DT

        nextExpectedTime = vsiCommonPythonApi.getSimulationTimeInNs()

        while vsiCommonPythonApi.getSimulationTimeInNs() < self.totalSimulationTimeNs:
            # safe receive state (IDs 12..14)
            x = self.safe_recv_double(12, default=0.0)
            y = self.safe_recv_double(13, default=0.0)
            theta = self.safe_recv_double(14, default=0.0)

            # reference and errors
            y_ref, dy_dx = self.reference_path(x)
            desired_theta = math.atan2(dy_dx, 1.0)

            lat_err = y_ref - y
            heading_err = (desired_theta - theta + math.pi) % (2 * math.pi) - math.pi
            d_lat = (lat_err - self.prev_lat_err) / (self.simulationStep if self.simulationStep>0 else DEFAULT_DT)
            self.prev_lat_err = lat_err

            # control law (PD + heading + feedforward)
            omega_cmd = (self.Kp_lat * lat_err +
                         self.Kd_lat * d_lat +
                         self.Kp_head * heading_err +
                         dy_dx * 0.5)

            v_cmd = self.v_nom * max(0.3, 1.0 - abs(heading_err))

            # publish commands on IDs 16,17
            self.send_double(16, v_cmd)
            self.send_double(17, omega_cmd)

            # advance VSI simulation
            nextExpectedTime += self.simulationStepNs
            vsiCommonPythonApi.advanceSimulation(nextExpectedTime - vsiCommonPythonApi.getSimulationTimeInNs())

    def safe_recv_double(self, cid, default=0.0):
        try:
            packed = vsiCanPythonGateway.recvVariableFromCanPacket(8, 0, 64, cid)
        except Exception:
            packed = None
        if not packed:
            return default
        n = struct.calcsize('=d')
        return struct.unpack('=d', packed[:n])[0]

    def send_double(self, cid, val):
        vsiCanPythonGateway.setCanId(cid)
        payload = struct.pack('=d', float(val))
        vsiCanPythonGateway.setCanPayloadBits(payload, 0, 64)
        vsiCanPythonGateway.setDataLengthInBits(64)
        vsiCanPythonGateway.sendCanPacket()

def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--domain', default='AF_UNIX')
    p.add_argument('--server-url', default='localhost')
    p.add_argument('--port', type=int, default=50102)
    p.add_argument('--path-type', choices=['straight','sine','curved'], default='curved')
    p.add_argument('--Kp_lat', type=float, default=1.5)
    p.add_argument('--Kp_head', type=float, default=2.5)
    p.add_argument('--Kd_lat', type=float, default=0.1)
    p.add_argument('--v_nom', type=float, default=1.0)
    return p.parse_args()

def main():
    args = parse_args()
    Controller(args).mainThread()

if __name__ == '__main__':
    main()
