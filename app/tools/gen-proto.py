#!/usr/bin/env python3
"""Generate Swift protobuf + gRPC stubs without protoc.

launch-proto's build.rs already compiles the .proto files (pure-Rust protox)
into a FileDescriptorSet. protoc plugins only need a CodeGeneratorRequest on
stdin, so we wrap that descriptor set ourselves and feed it to the plugins.

Usage: tools/gen-proto.py   (run `cargo build -p launch-proto` first, and build
the plugins: swift build -c release --product protoc-gen-swift
             swift build -c release --product protoc-gen-grpc-swift-2)
"""
import glob, os, subprocess, sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.dirname(ROOT)
PKG = os.path.join(ROOT, "LaunchMonitor")
OUT = os.path.join(PKG, "Sources", "LaunchMonitor", "Generated")
TARGETS = ["launchmonitor/v1/types.proto", "launchmonitor/v1/shot.proto", "launchmonitor/v1/calibration.proto"]


def varint(n):
    out = bytearray()
    while True:
        b = n & 0x7F
        n >>= 7
        out.append(b | (0x80 if n else 0))
        if not n:
            return bytes(out)


def read_varint(buf, i):
    shift = n = 0
    while True:
        b = buf[i]
        i += 1
        n |= (b & 0x7F) << shift
        if not b & 0x80:
            return n, i
        shift += 7


def fields(buf):
    i = 0
    while i < len(buf):
        key, i = read_varint(buf, i)
        num, wire = key >> 3, key & 7
        if wire == 2:
            n, i = read_varint(buf, i)
            yield num, buf[i : i + n]
            i += n
        elif wire == 0:
            v, i = read_varint(buf, i)
            yield num, v
        else:
            raise SystemExit(f"unsupported wire type {wire}")


def ld(num, payload):
    return varint(num << 3 | 2) + varint(len(payload)) + payload


def request(fds, params):
    req = b"".join(ld(1, t.encode()) for t in TARGETS)
    if params:
        req += ld(2, params.encode())
    # FileDescriptorSet.file (1) -> CodeGeneratorRequest.proto_file (15)
    for num, payload in fields(fds):
        assert num == 1
        req += ld(15, payload)
    return req


def run(plugin, params, fds):
    resp = subprocess.run([plugin], input=request(fds, params), capture_output=True, check=True).stdout
    for num, payload in fields(resp):
        if num == 1:
            sys.exit(f"{plugin}: {payload.decode()}")
        if num == 15:
            name = content = None
            for n, v in fields(payload):
                if n == 1:
                    name = v.decode()
                elif n == 15:
                    content = v
            if name.endswith(".grpc.swift") and content.count(b"\n") < 20:
                continue  # file without services: header only
            path = os.path.join(OUT, os.path.basename(name))
            with open(path, "wb") as f:
                f.write(content)
            print("wrote", os.path.relpath(path, REPO))


def main():
    cands = glob.glob(os.path.join(REPO, "target", "*", "build", "launch-proto-*", "out", "launchmonitor_descriptor.bin"))
    if not cands:
        sys.exit("descriptor not found; run `cargo build -p launch-proto` first")
    fds = open(max(cands, key=os.path.getmtime), "rb").read()
    os.makedirs(OUT, exist_ok=True)
    rel = os.path.join(PKG, ".build", "release")
    run(os.path.join(rel, "protoc-gen-swift"), "Visibility=Public", fds)
    run(os.path.join(rel, "protoc-gen-grpc-swift-2"), "Visibility=Public,Server=false", fds)


main()
