#!/usr/bin/env python3
import argparse
import os
import socket
import sys
import time
import urllib.request

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


def fq_node_name(namespace: str, name: str) -> str:
    if not namespace or namespace == '/':
        return f'/{name}'
    return f'{namespace.rstrip("/")}/{name}'


def tcp_ready(port: int) -> bool:
    try:
        with socket.create_connection(('127.0.0.1', port), timeout=1.5):
            return True
    except OSError:
        return False


def http_ready(port: int) -> bool:
    try:
        with urllib.request.urlopen(f'http://127.0.0.1:{port}/', timeout=2.0):
            return True
    except Exception:
        return False


def pid_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False


def main() -> int:
    parser = argparse.ArgumentParser(description='Single-participant bringup smoke readiness probe')
    parser.add_argument('--mode', required=True)
    parser.add_argument('--launch-pid', type=int, required=True)
    parser.add_argument('--timeout', type=float, required=True)
    parser.add_argument('--service', action='append', default=[])
    parser.add_argument('--node', action='append', default=[])
    parser.add_argument('--tcp', type=int, action='append', default=[])
    parser.add_argument('--http', type=int, action='append', default=[])
    args = parser.parse_args()

    rclpy.init()
    node = Node('bringup_smoke_probe')
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    deadline = time.monotonic() + args.timeout
    try:
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)

            if not pid_alive(args.launch_pid):
                print(f'Launch for mode {args.mode} exited unexpectedly')
                return 1

            service_names = {name for name, _ in node.get_service_names_and_types()}
            node_names = {
                fq_node_name(namespace, name)
                for name, namespace in node.get_node_names_and_namespaces()
            }

            missing_services = [name for name in args.service if name not in service_names]
            missing_nodes = [name for name in args.node if name not in node_names]
            missing_tcp = [port for port in args.tcp if not tcp_ready(port)]
            missing_http = [port for port in args.http if not http_ready(port)]

            if not (missing_services or missing_nodes or missing_tcp or missing_http):
                return 0

            time.sleep(0.4)

        print(f'Timed out waiting for bringup smoke readiness in mode {args.mode}')
        if args.service:
            print(f'missing_services={missing_services}')
        if args.node:
            print(f'missing_nodes={missing_nodes}')
        if args.tcp:
            print(f'missing_tcp={missing_tcp}')
        if args.http:
            print(f'missing_http={missing_http}')
        return 1
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
