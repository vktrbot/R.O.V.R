using Data;
using Events;
using Events.Types;
using Newtonsoft.Json;
using Unity.WebRTC;
using UnityEngine;
using UnityEngine.InputSystem;

namespace Services.Roboarm {
    public class TargetShifter : MonoBehaviour {

        private const float DirectionEpsilon = 0.0001f;

        private Vector3 _homePosition;

        private Listener<OnDataChannelCreated> _onChannelCreated;
        private Listener<DeadManSwitch> _switch;
        private Listener<PositioningControllerEvent> _positioningController;

        [SerializeField] private Side controllerSide = Side.RIGHT;
        [SerializeField] private Transform headset;
        [SerializeField] private Transform armatureHome;

        private RTCDataChannel _channel;
        private Vector3 _controllerPosition;
        private Vector3 _deadMenOrigin;
        private Vector3 _armDirection;
        private Quaternion _controllerToArmRotation = Quaternion.identity;

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
            _armDirection = ResolveArmDirection();
        }

        private void OnChannel(OnDataChannelCreated e) {
            _channel = e.Channel;
        }

        private void OnPositioning(PositioningControllerEvent e) {
            if (e.Side != controllerSide)
                return;

            _controllerPosition = e.Position;
        }

        private void OnSwitch(DeadManSwitch e) {
            if (_channel == null) return;
            if (!_isActive && e.IsActive) {
                _deadMenOrigin = _controllerPosition;
                _controllerToArmRotation = ResolveControllerToArmRotation();
            }

            _isActive = e.IsActive;

            if (!e.IsActive) {
                transform.position = _homePosition;
            }
        }

        private void FixedUpdate() {
            if (_channel == null) return;
            if (_isActive) {
                var shift = _controllerPosition - _deadMenOrigin;
                var remappedShift = _controllerToArmRotation * shift;
                transform.position = _homePosition + remappedShift;
            }
        }

        private Quaternion ResolveControllerToArmRotation() {
            Vector3 headsetForward = ProjectToHorizontal(
                headset != null ? headset.forward : Vector3.forward,
                Vector3.forward);

            Vector3 armForward = ProjectToHorizontal(_armDirection, headsetForward);
            return Quaternion.FromToRotation(headsetForward, armForward);
        }

        private Vector3 ResolveArmDirection() {
            if (armatureHome == null)
                return Vector3.forward;

            Vector3 direction = _homePosition - armatureHome.position;
            if (direction.sqrMagnitude > DirectionEpsilon)
                return direction.normalized;

            return armatureHome.forward.sqrMagnitude > DirectionEpsilon
                ? armatureHome.forward.normalized
                : Vector3.forward;
        }

        private static Vector3 ProjectToHorizontal(Vector3 direction, Vector3 fallback) {
            Vector3 planarDirection = Vector3.ProjectOnPlane(direction, Vector3.up);
            if (planarDirection.sqrMagnitude > DirectionEpsilon)
                return planarDirection.normalized;

            Vector3 planarFallback = Vector3.ProjectOnPlane(fallback, Vector3.up);
            if (planarFallback.sqrMagnitude > DirectionEpsilon)
                return planarFallback.normalized;

            return Vector3.forward;
        }
    }
}
