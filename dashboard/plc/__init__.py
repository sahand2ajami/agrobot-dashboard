"""Vendored gRPC client stubs for the PLC Gateway (RobotService).

These are a *client-side copy* of the stubs from ``~/plc_gateway/gRPC-Gateway-Agrobot``,
regenerated against the dashboard's protobuf 4.25.x runtime (the upstream copies are
generated with protobuf 6.x and won't import here). gRPC wire format is version
independent, so this 4.x client talks to the 6.x Gateway server fine.

To regenerate after a proto change (copy the new robot_control.proto in first):
    python3 -m pip install 'grpcio-tools==1.62.3'   # build-only
    python3 -m grpc_tools.protoc -I dashboard/plc \
        --python_out=dashboard/plc --grpc_python_out=dashboard/plc \
        dashboard/plc/robot_control.proto
    # then change the generated `import robot_control_pb2` line in
    # robot_control_pb2_grpc.py to `from . import robot_control_pb2`
"""
