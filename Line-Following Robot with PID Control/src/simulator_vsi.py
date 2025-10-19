#!/usr/bin/env python3
"""
Simulator (E3) - corrected:
 - simulator publishes x,y,theta,t on CAN IDs 12,13,14,15
 - reads control v,omega on CAN IDs 16,17
 - noise: Gaussian on v and omega (std = --noise)
 - disturbance: occasional transient angular impulse (magnitude = --disturbance)
 - uses VSI timing (getSimulationStep/getSimulationTimeInNs/advanceSimulation) correctly
"""
from __future__ import print_function
import struct, sys, argparse, math, random
import numpy as np

PythonGateways = 'pythonGateways/'
sys.path.append(PythonGateways)

import VsiCommonPythonApi as vsiCommonPythonApi
import VsiCanPythonGateway as vsiCanPythonGateway

DEFAULT_DT = 0.02  # seconds (if VSI step is 0)


class Simulator:
    def __init__(self, args):
        self.componentId = 0
        self.localHost = args.server_url
        self.domain = args.domain
        self.portNum = args.port if args.port is not None else 50101

        # initial state
        self.x = args.init_x
        self.y = args.init_y
        self.theta = args.init_theta
        self.t = 0.0

        # control inputs (last known)
        self.v = 0.0
        self.omega = 0.0

        # noise & disturbance
        self.noise_std = args.noise
        self.disturbance = args.disturbance
        self.dist_prob = args.dist_prob
        self.dist_duration_s = args.dist_duration

        # internal disturbance state
        self._dist_counter = 0
        self._dist_omega = 0.0

        # VSI timing
        self.simulationStepNs = 0
        self.totalSimulationTimeNs = 0
        self.simulationStep = DEFAULT_DT

    def mainThread(self):
        dSession = vsiCommonPythonApi.connectToServer(self.localHost, self.domain, self.portNum, self.componentId)
        vsiCanPythonGateway.initialize(dSession, self.componentId)
        vsiCommonPythonApi.waitForReset()

        # read VSI timing info
        self.totalSimulationTimeNs = vsiCommonPythonApi.getTotalSimulationTime()
        self.simulationStepNs = vsiCommonPythonApi.getSimulationStep()
        if self.simulationStepNs and self.simulationStepNs > 0:
            self.simulationStep = self.simulationStepNs / 1e9
        else:
            self.simulationStepNs = int(DEFAULT_DT * 1e9)
            self.simulationStep = DEFAULT_DT

        nextExpectedTime = vsiCommonPythonApi.getSimulationTimeInNs()

        while vsiCommonPythonApi.getSimulationTimeInNs() < self.totalSimulationTimeNs:
            # safely receive control (IDs 16,17). If packet empty, keep previous values.
            v_val, _ = self.safe_recv_double(16, default=self.v)
            omega_val, _ = self.safe_recv_double(17, default=self.omega)
            self.v = v_val
            self.omega = omega_val

            # noise
            v_noisy = self.v + random.gauss(0.0, self.noise_std)
            omega_noisy = self.omega + random.gauss(0.0, self.noise_std)

            # occasional transient disturbance: set counter and dist omega
            if self._dist_counter <= 0 and self.disturbance > 0 and random.random() < self.dist_prob:
                sign = 1 if random.random() < 0.5 else -1
                self._dist_omega = sign * self.disturbance
                self._dist_counter = max(1, int(self.dist_duration_s / self.simulationStep))

            if self._dist_counter > 0:
                omega_noisy += self._dist_omega
                self._dist_counter -= 1
                if self._dist_counter == 0:
                    self._dist_omega = 0.0

            # propagate dynamics
            self.x += v_noisy * math.cos(self.theta) * self.simulationStep
            self.y += v_noisy * math.sin(self.theta) * self.simulationStep
            self.theta += omega_noisy * self.simulationStep
            self.t += self.simulationStep

            # publish state (12..15)
            self.send_double(12, self.x)
            self.send_double(13, self.y)
            self.send_double(14, self.theta)
            self.send_double(15, self.t)

            # advance simulation in VSI (ns)
            nextExpectedTime += self.simulationStepNs
            vsiCommonPythonApi.advanceSimulation(nextExpectedTime - vsiCommonPythonApi.getSimulationTimeInNs())

    # helper: safe recv returning default if no data
    def safe_recv_double(self, cid, default=0.0):
        packed = None
        try:
            packed = vsiCanPythonGateway.recvVariableFromCanPacket(8, 0, 64, cid)
        except Exception:
            packed = None
        if not packed:
            return default, b''
        n = struct.calcsize('=d')
        val = struct.unpack('=d', packed[:n])[0]
        return val, packed[n:]

    def send_double(self, cid, val):
        vsiCanPythonGateway.setCanId(cid)
        payload = struct.pack('=d', float(val))
        vsiCanPythonGateway.setCanPayloadBits(payload, 0, 64)
        vsiCanPythonGateway.setDataLengthInBits(64)
        vsiCanPythonGateway.sendCanPacket()

def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument('--domain', default='AF_UNIX')
    ap.add_argument('--server-url', default='localhost')
    ap.add_argument('--port', type=int, default=50101)
    ap.add_argument('--noise', type=float, default=0.02, help='std dev for Gaussian noise on v/omega')
    ap.add_argument('--disturbance', type=float, default=0.2, help='angular impulse magnitude (rad/s)')
    ap.add_argument('--dist-prob', type=float, default=0.01, help='probability per step of a disturbance event')
    ap.add_argument('--dist-duration', type=float, default=0.1, help='duration (s) of disturbance impulse')
    ap.add_argument('--init-x', type=float, default=0.0)
    ap.add_argument('--init-y', type=float, default=0.0)
    ap.add_argument('--init-theta', type=float, default=0.0)
    return ap.parse_args()

def main():
    args = parse_args()
    Simulator(args).mainThread()

if __name__ == '__main__':
    main()
