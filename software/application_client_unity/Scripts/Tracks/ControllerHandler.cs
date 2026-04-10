using Data;
using Events;
using Events.Types;
using Newtonsoft.Json;
using Services;
using Unity.WebRTC;
using UnityEngine;

namespace Tracks {
    public class ControllerHandler : MonoBehaviour {

        private Listener<OnGrabEvent> _onGrab;
        private Listener<OnPrimaryButtonEvent> _onPrimaryButton;
        private Listener<OnSecondaryButtonEvent>  _onSecondaryButton;
        private Listener<OnStickEvent> _onStick;
        private Listener<OnDataChannelCreated> _onChannelCreated;
        
        private IControllerHandler _mainControllerHandler;
        private IControllerHandler _secondaryControllerHandler;
        
        private RTCDataChannel _rtcChannel;


        private void Awake() {
            _onGrab = new Listener<OnGrabEvent>(OnGrab);
            EventBus<OnGrabEvent>.Register(_onGrab, this);
            
            _onPrimaryButton = new Listener<OnPrimaryButtonEvent>(OnPrimaryButton);
            EventBus<OnPrimaryButtonEvent>.Register(_onPrimaryButton, this);
            
            _onSecondaryButton = new Listener<OnSecondaryButtonEvent>(OnSecondaryButton);
            EventBus<OnSecondaryButtonEvent>.Register(_onSecondaryButton, this);
            
            _onStick = new Listener<OnStickEvent>(OnStick);
            EventBus<OnStickEvent>.Register(_onStick, this);

            _onChannelCreated = new Listener<OnDataChannelCreated>(OnChannelCreated);
            EventBus<OnDataChannelCreated>.Register(_onChannelCreated, this);

            _mainControllerHandler = null;
            _secondaryControllerHandler = null;
        }

        private void OnChannelCreated(OnDataChannelCreated e) {
            _rtcChannel = e.Channel;
            _mainControllerHandler = new MainControllerHandler(e.Channel);
            _secondaryControllerHandler = new SecondaryControllerHandler();
        }

        private void OnGrab(OnGrabEvent e) {
            var controller = ChooseHandler(e.Side);
            controller.HandleGrab(e.IsActive);
        }

        private IControllerHandler ChooseHandler(Side side) {
            if(side == Side.LEFT) return _mainControllerHandler;
            if (side == Side.RIGHT) return _secondaryControllerHandler;
            return null;
        }

        private void OnPrimaryButton(OnPrimaryButtonEvent e) {
            var controller = ChooseHandler(e.Side);
            controller.HandlePrimaryButton(e.IsActive);
        }

        private void OnSecondaryButton(OnSecondaryButtonEvent e) {
            var controller = ChooseHandler(e.Side);
            controller.HandleSecondaryButton(e.IsActive);
        }

        private void OnStick(OnStickEvent e) {
            var controller = ChooseHandler(e.Side);
            controller.HandleStick(e.Position);
        }

        private void Update() {
            _mainControllerHandler?.Update();
            _secondaryControllerHandler?.Update();
        }
    }

    public interface IControllerHandler {
        void HandleGrab(bool isActive);
        void HandlePrimaryButton(bool isActive);
        void HandleSecondaryButton(bool isActive);
        void HandleStick(Vector2 direction);

        public void Update();
    }

    class MainControllerHandler : IControllerHandler {

        private RTCDataChannel _channel;
        
        private Vector2 _direction;
        
        private LocomotionData _locomotion;
        
        public MainControllerHandler(RTCDataChannel channel) {
            _channel = channel;
        }
        
        public void HandleGrab(bool isActive) {
            return;
        }

        public void HandlePrimaryButton(bool isActive) {
            if (!isActive) return;
            VrStore.Instance.DecreasePower();
        }

        public void HandleSecondaryButton(bool isActive) {
            if (!isActive) return;
            VrStore.Instance.IncreasePower();
        }

        public void HandleStick(Vector2 direction) {
            _direction = direction;
        }


        private void CalculateLocomotionData() {
            var obliqueLine = ClampToBounds(_direction.y);
            var power = VrStore.Instance.Power;
            _locomotion.power = power;
            _locomotion.direction = new Vector(_direction.x, obliqueLine);
        }

        private int ClampToBounds(float x, float threshold = 0.2f) => Mathf.Abs(x) <= threshold ? 0 : (int)Mathf.Sign(x);

        public void Update() {
            CalculateLocomotionData();
            _channel.Send(JsonConvert.SerializeObject(Container.Of("drive", _locomotion)));
        }
    }
    
    class SecondaryControllerHandler : IControllerHandler {
        
        public void HandleGrab(bool isActive) {
            DeadManSwitch e = new DeadManSwitch(isActive);
            EventBus<DeadManSwitch>.Raise(e);
        }

        public void HandlePrimaryButton(bool isActive) {
            VrStore.Instance.State = "open";
        }

        public void HandleSecondaryButton(bool isActive) {
            VrStore.Instance.State = "closed";
        }

        public void HandleStick(Vector2 isActive) {
            return;
        }

        public void Update() {
            return;
        }
    }
}