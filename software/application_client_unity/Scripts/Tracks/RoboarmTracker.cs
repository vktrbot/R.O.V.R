using System;
using Data;
using Events;
using Events.Types;
using Newtonsoft.Json;
using Services;
using Unity.WebRTC;
using UnityEngine;
using Utils;

namespace Tracks {
    public class RoboarmTracker : MonoBehaviour {
        
        [Serializable]
        public class Joint {
            public ArticulationBody Target;
            public ArticulationJacobianIK.DriveAxis Axis;
            public bool Invert;
        }

        [SerializeField] private Joint[] joints;
        
        private Listener<DeadManSwitch> _deadMenSwitchListener;
        private Listener<OnDataChannelCreated> _onChannelCreated;
        
        private RTCDataChannel _rtcDataChannel;
        private RoboarmLocation _location;
        private bool _deadManSwitchState;

        private void Awake() {
            _onChannelCreated = new Listener<OnDataChannelCreated>(OnChannelCreated);
            EventBus<OnDataChannelCreated>.Register(_onChannelCreated, this);

            _deadMenSwitchListener = new Listener<DeadManSwitch>(OnDeadMan);
            EventBus<DeadManSwitch>.Register(_deadMenSwitchListener, this);
            
            _location = new RoboarmLocation();
        }

        private void OnDeadMan(DeadManSwitch e) {
            _deadManSwitchState = e.IsActive;
        }

        private void OnChannelCreated(OnDataChannelCreated e) {
            _rtcDataChannel = e.Channel;
        }

        private void Update() {
            if (_rtcDataChannel == null) return;
            var points = CollectPoints();
            var state = VrStore.Instance.State;
            _location.deadman = _deadManSwitchState;
            _location.joints_deg = points;
            _location.gripper = state;
            
            _rtcDataChannel.Send(JsonConvert.SerializeObject(Container.Of("arm", _location)));
        }

        private float[] CollectPoints() {
            var points = new float[joints.Length];
            for (var i = 0; i < joints.Length; i++) {
                var joint =  joints[i];
                var angle = joint.Axis switch{
                    ArticulationJacobianIK.DriveAxis.X => joint.Target.transform.localRotation.eulerAngles.x,
                    ArticulationJacobianIK.DriveAxis.Y => joint.Target.transform.localRotation.eulerAngles.y,
                    ArticulationJacobianIK.DriveAxis.Z => joint.Target.transform.localRotation.eulerAngles.z,
                    _ => 0,
                };
                var drive = joint.Target.xDrive;
                points[i] = Math.Clamp(UnityUtils.NormalizeAngle(angle), drive.lowerLimit, drive.upperLimit) * (joint.Invert ? -1 : 1);
            }
            return points;
        }
    }
}