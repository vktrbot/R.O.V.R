# R.O.V.R

A four-wheeled mobile robot teleoperated over the internet using a Meta Quest 3 VR headset, with a live first-person video feed and a 5-DOF robotic arm controlled via inverse kinematics from the operator's VR controller movements.

Built as a team project for the Mechatronics Exercises course at Aalto University.

[ROVR_info.pdf](https://github.com/user-attachments/files/27192174/ROVR_info.pdf)


## Demo

### Robotic Arm Demo
https://github.com/user-attachments/assets/638f44d5-a422-4bfc-b795-b36a408cdfbc

### Camera Pan/Tilt and Driving Demo
https://github.com/user-attachments/assets/84d23001-c920-4975-bea1-24dbc5f30848

### Driving Demo - External View and Operator View
https://github.com/user-attachments/assets/6d4637f3-454d-421b-a35a-25c15557b606

---

## What it does

- The operator wears a Quest 3 headset and sees the robot's pole-mounted camera feed in real time.
- Head tracking drives the camera pan/tilt servos — look around in VR, the camera follows.
- Controller tracking drives the robotic arm — move the VR controller in 3D space, the arm's IK solver computes joint angles and sends them to the servos.
- Thumbsticks drive the chassis (DC motor + steering servo).
- Control data and video are streamed over WebRTC, so the robot can be operated remotely over the internet, not just on a local network.

---

## Tech stack

**Robot OS**: Raspberry Pi 5 · Ubuntu · ROS 2 Jazzy

**Robot software**: Python · WebRTC · gpiozero · pyserial

**VR client**: Unity · C# · Unity.WebRTC

**IK**: Custom damped-Jacobian solver on Unity ArticulationBody

**Network**: WebRTC peer-to-peer · WebSocket signaling · TURN

**Signaling server**: FastAPI + uvicorn · HMAC challenge-nonce auth · lobby pairing

**Hardware**: MD10C motor driver · ST3215 servos · 120W DC motor · TGY-S9010 steering servo

---

## My contributions

I was responsible for the robot software stack — from Raspberry Pi control software to the Unity VR client and the signaling server.

## Architecture

The system is split into three main parts:

- **Robot**: Raspberry Pi 5 running ROS 2 nodes for chassis, arm, camera, and WebRTC communication.
- **Operator**: Unity application on Meta Quest 3 handling VR input, IK, video display, and command streaming.
- **Signaling server**: FastAPI WebSocket server used for authentication, lobby pairing, and WebRTC session setup.

After signaling, control data and video are exchanged through WebRTC between the operator and the robot.

<img width="625" height="565" alt="rovr_logic" src="https://github.com/user-attachments/assets/b502c80a-510a-49ad-ba80-4ec710a122cb" />

### Robot side — Raspberry Pi · Python · ROS 2

- **ROS 2 architecture.** Designed the package layout (`rovr_arm`, `rovr_camera`, `rovr_chassis`, `rovr_common`, `rovr_interfaces`, `rovr_bringup`) so each subsystem has clean core logic.
- **Custom ROS messages** (`ArmCommand`, `CameraCommand`, `ArmTelemetry`) including per-joint load / temperature / jam-detected / torque-enabled telemetry to control the servomotors.
- **WebRTC communications module.** Wrote the client that connects to the signaling server, negotiates the peer connection, registers the data channel + video track, parses incoming JSON command frames (drive / camera / arm), and bridges them into ROS 2 topics.
- **Low-latency video pipeline.** Configured v4l2 capture with low-delay flags and wrote a wrapper track that drops stale queued frames before sending — meaningfully cuts motion-to-photon latency in the headset.
- **Chassis control.** PWM + direction signals to the MD10C motor driver, steering servo via Pi GPIO. Added soft-start/soft-stop slew limiting and a direction-coast safety so reverse voltage is never applied to a spinning DC motor.
- **Servo bus driver.** Implemented control of the ST3215 serial bus (8 servos: 5 arm joints + gripper + 2 pan/tilt servos for camera), including position calibration, soft limits, and a homing routine that returns the arm to a safe pose when commands stop arriving.
- **Robot-arm bring-up.** Configured the bus driver boards, assigned servo IDs, set per-joint mechanical limits.

### VR client — Unity · C#

- **Damped-Jacobian IK solver.** Custom IK for the 5 DoF arm in Unity. Supports per-joint soft limits and speed limits, adaptive step scaling on no-improvement steps, error-reduction safety checks, and a settle-handshake so the next command isn't sent until the previous one converges within tolerance.

<img width="625" height="370" alt="arm" src="https://github.com/user-attachments/assets/17184dd0-3b2a-4799-ade9-b8ad4502bf5b" />

- **VR I/O.** Headset pose → camera pan/tilt target. Controller pose → arm IK target. Joysticks → chassis command. Video frames → in-headset texture.
- **Networking.** WebSocket client to the signaling server, Unity.WebRTC peer connection with data channel + video track receiver, structured JSON command frames sent at a configurable rate.

### Signaling server

- **FastAPI** WebSocket endpoint with a challenge-nonce handshake against per-device secrets, lobby creation and lobby join, ICE relay between paired clients. Deployed on a small VPS.

The mechanical design (chassis, arm) and the electronics layout were handled by other team members.

---

## Running it

**Full stack on the robot:**

```bash
bash scripts/run_full_stack.sh real     # real hardware
bash scripts/run_full_stack.sh mock     # simulated hardware
```

**Subsystem tests:**

```bash
python3 scripts/test_drive_keyboard.py             # drive with WASD, no VR client needed
python3 scripts/test_arm_positions.py              # cycle the arm through preset poses
python3 scripts/test_camera_positions.py           # exercise pan/tilt
python3 scripts/calibrate_arm.py                   # full arm calibration
python3 scripts/recalibrate_joint.py --joint {n}   # single-joint recalibration
```

---

## Next steps

- **Chassis node stability.** Under continuous driving, the chassis node can crash after approximately 5–10 minutes, while the arm, camera, and video pipeline remain operational. The likely cause is a GPIO/PWM resource leak
- **URDF + MoveIt integration.** A proper URDF model would enable collision-aware planning with MoveIt and replace the current custom IK solver with a more standard robotics stack
- **Operator HUD.** Battery level, round-trip latency, and per-joint load indicators would make remote operation safer and easier

---
