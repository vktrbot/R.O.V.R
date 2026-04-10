using Data;
using Events;
using Events.Types;
using Newtonsoft.Json;
using Unity.WebRTC;
using UnityEngine;
using UnityEngine.InputSystem;

namespace Services.Roboarm {
    public class TestShifter : MonoBehaviour {
        
        private Vector3 _homePosition;        
        
        private Listener<OnDataChannelCreated> _onChannelCreated;
        private Listener<DeadManSwitch> _switch;
        private Listener<PositioningControllerEvent> _positioningController;

        [SerializeField] private InputActionReference positionAction;
        [SerializeField] private Transform headset;
        [SerializeField] private Transform armatureHome;
        
        private RTCDataChannel _channel;
        private Vector3 _controllerPosition;
        private Vector3 _deadMenOrigin;
        private Vector3 _armDirection;

        private bool _isActive;
        
        private void Awake() {
            _homePosition = transform.position;

            _switch = new Listener<DeadManSwitch>(OnSwitch);
            EventBus<DeadManSwitch>.Register(_switch, this);

            _positioningController = new Listener<PositioningControllerEvent>(OnPositioning);
            EventBus<PositioningControllerEvent>.Register(_positioningController, this);

            _onChannelCreated = new Listener<OnDataChannelCreated>(OnChannel);
            EventBus<OnDataChannelCreated>.Register(_onChannelCreated, this);
        }

        private void Start() {
            _armDirection = _homePosition - armatureHome.position;
            _armDirection.Normalize();
            _isActive = true; // TODO: Remove
        }

        private void OnChannel(OnDataChannelCreated e) {
            _channel = e.Channel;
        }

        private void OnPositioning(PositioningControllerEvent e) {
            _controllerPosition =  e.Position;
        }

        private void OnSwitch(DeadManSwitch e) {
            // if (_channel == null) return;
            if (!_isActive && e.IsActive) {
                _deadMenOrigin = _controllerPosition;
            }
            _isActive = e.IsActive;
            if (!e.IsActive) {
                transform.position = _homePosition;
            }
        }

        private struct TargetShiftData {
            public float home_x;
            public float home_y;
            public float home_z;
            public float shift_x;
            public float shift_y;
            public float shift_z;
        }
        
        private float _step;

        private void FixedUpdate() {
            // if (_channel == null) return;
            _step += Time.fixedDeltaTime;
            var x = Mathf.Sin(_step * 10f);
            var y = Mathf.Cos(_step * 10f);
            var z = Mathf.Cos(_step * 10f);
            _controllerPosition = _deadMenOrigin + new Vector3(x, y, z);
            if (_isActive) {
                
                var qRd = Quaternion.Euler(_armDirection.x * 360, _armDirection.y * 360, _armDirection.z * 360);
                var aMov = _controllerPosition - _deadMenOrigin;
                var aMov_i = qRd * aMov;
                transform.position = _homePosition + aMov;
            }
        }

    }
}
