# R.O.V.R

A four-wheeled mobile robot teleoperated from a Meta Quest 3 VR headset over the internet, with a live first-person video feed and a 6-axis robotic arm controlled via inverse kinematics from the operator's hand movements.

Built as a team project for the Mechatronics Exercises course at Aalto University.

[ROVR_info.pdf](https://github.com/user-attachments/files/27192174/ROVR_info.pdf)


## Demo

https://github.com/user-attachments/assets/8a07a5b6-451d-4334-9e74-df3e29b0b973

https://github.com/user-attachments/assets/29166c4f-d11a-4eac-a6c9-1b25f7da4b84

https://github.com/user-attachments/assets/33ea84b9-004b-4aba-8b35-16cac6187755

---

## What it does

- The operator wears a Quest 3 headset and sees the robot's pole-mounted camera feed in real time.
- Head tracking drives the camera pan/tilt servos — look around in VR, the camera follows.
- Controller tracking drives the robotic arm — move the VR controller in 3D space, the arm's IK solver computes joint angles and sends them to the servos.
- Thumbsticks drive the chassis (DC motor + steering servo).
- All control + video runs peer-to-peer over WebRTC, so the robot can be operated from anywhere with internet, not just the local network.

---

## Tech stack

**Robot OS**: Raspberry Pi 5 · Ubuntu · ROS 2 Jazzy

**Robot software**: Python · WebRTC · gpiozero · pyserial

**VR client**: Unity · C# · Unity.WebRTC

**IK**: Custom damped-Jacobian solver on Unity ArticulationBody

**Network**: WebRTC peer-to-peer · WebSocket signaling · TURN

**Signaling server**: FastAPI + uvicorn · HMAC challenge-nonce auth · lobby pairing

**Hardware**: MD10C motor driver · STS3215 servos · 120W DC-motor · TGY-S9010 steering servo

---

## My contributions

I owned the robot's software stack — from Pi firmware up to the Unity VR client and the signaling server.

<img width="625" height="565" alt="rovr_logic" src="https://github.com/user-attachments/assets/b502c80a-510a-49ad-ba80-4ec710a122cb" />

### Robot side — Raspberry Pi · Python · ROS 2

- **ROS 2 architecture.** Designed the package layout (`rovr_arm`, `rovr_camera`, `rovr_chassis`, `rovr_common`, `rovr_interfaces`, `rovr_bringup`) so each subsystem has clean core logic.
- **Custom ROS messages** (`ArmCommand`, `CameraCommand`, `ArmTelemetry`) including per-joint load / temperature / jam-detected / torque-enabled telemetry to control the servomotors.
- **WebRTC communications module.** Wrote the client that connects to the signaling server, negotiates the peer connection, registers the data channel + video track, parses incoming JSON command frames (drive / camera / arm), and bridges them into ROS 2 topics.
- **Low-latency video pipeline.** Configured v4l2 capture with low-delay flags and wrote a wrapper track that drops stale queued frames before sending — meaningfully cuts motion-to-photon latency in the headset.
- **Chassis control.** PWM + direction signals to the MD10C motor driver, steering servo via Pi GPIO. Added soft-start/soft-stop slew limiting and a direction-coast safety so reverse voltage is never applied to a spinning DC motor.
- **Servo bus driver.** Implemented control of the STS3215 serial bus (8 servos: 6 arm servos + 2 pan/tilt), including position calibration, soft limits, and a homing routine that returns the arm to a safe pose when commands stop arriving.
- **Robot-arm bring-up.** Configured the bus driver boards, assigned servo IDs, set per-joint mechanical limits.

### VR client — Unity · C#

- **Damped-Jacobian IK solver.** Custom IK for the 6-axis arm in Unity. Supports per-joint soft limits and speed limits, adaptive step scaling on no-improvement steps, error-reduction safety checks, and a settle-handshake so the next command isn't sent until the previous one converges within tolerance.

<img width="625" height="370" alt="arm" src="https://github.com/user-attachments/assets/17184dd0-3b2a-4799-ade9-b8ad4502bf5b" />

- **VR I/O.** Headset pose → camera pan/tilt target. Controller pose → arm IK target. Joysticks → chassis command. Video frames → in-headset texture.
- **Networking.** WebSocket client to the signaling server, Unity.WebRTC peer connection with data channel + video track receiver, structured JSON command frames sent at a configurable rate.

### Signaling server

- **FastAPI** WebSocket endpoint with a challenge-nonce handshake against per-device secrets, lobby creation and lobby join, ICE relay between paired clients. Deployed on a small VPS.

The mechanical design (chassis, arm) and the electronics layout were handled by other team members.

---

## Running it

**Full stack on the robot:**
bash scripts/run_full_stack.sh real     # real hardware

bash scripts/run_full_stack.sh mock     # simulated hardware

**Subsystem tests:**
python3 scripts/test_drive_keyboard.py             # drive with WASD, no VR client needed

python3 scripts/test_arm_positions.py              # cycle the arm through preset poses

python3 scripts/test_camera_positions.py           # exercise pan/tilt

python3 scripts/calibrate_arm.py                   # full arm calibration

python3 scripts/recalibrate_joint.py --joint {n}   # single-joint recalibration

---

## Next steps

- **Stability of the chassis node.** Under continuous driving the drive node currently crashes after ~5–10 minutes (everything else — arm, camera, video — keeps running). Most likely a GPIO/PWM resource leak; on the to-do list.
- **URDF + MoveIt.** A proper URDF model would unlock MoveIt for collision-aware planning and let me retire the bespoke IK in favor of something supported.
- **Operator HUD.** Battery, round-trip latency, and per-joint load indicators would make remote operation a lot safer than it is today.

---
