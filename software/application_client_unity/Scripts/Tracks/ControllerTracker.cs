using System;
using Events;
using Events.Types;
using Unity.WebRTC;
using UnityEngine;
using UnityEngine.InputSystem;

public class ControllerTracker : MonoBehaviour {

    [SerializeField] private Side side;
    
    [SerializeField] private InputActionReference positionAction;
    [SerializeField] private InputActionReference rotationAction;
    [SerializeField] private InputActionReference triggerAction;
    [SerializeField] private InputActionReference gripAction;
    [SerializeField] private InputActionReference primary2DAxisAction;
    [SerializeField] private InputActionReference primaryButtonAction;
    [SerializeField] private InputActionReference secondaryButtonAction;
    
    private Listener<OnDataChannelCreated> _dataChannelCreated;
    
    private RTCDataChannel _rtcDataChannel;

    private bool _lastDeadMenState;

    private bool _triggerPressed;
    private bool _gripPressed;

    private void Awake() {
        _dataChannelCreated = new Listener<OnDataChannelCreated>(OnConnected);
        EventBus<OnDataChannelCreated>.Register(_dataChannelCreated, this);
    }

    private void OnConnected(OnDataChannelCreated e) {
        _rtcDataChannel = e.Channel;
        InitActions();
    }

    private void InitActions() {
        positionAction?.action.Enable();
        rotationAction?.action.Enable();
        triggerAction?.action.Enable();
        gripAction?.action.Enable();
        primary2DAxisAction?.action.Enable();
        primaryButtonAction?.action.Enable();
        secondaryButtonAction?.action.Enable();

        if (primaryButtonAction != null) {
            primaryButtonAction.action.performed += OnMainButtonPressed;
        }

        if (secondaryButtonAction != null) {
            secondaryButtonAction.action.performed += OnSecondaryButtonPressed;
        }

        if (triggerAction != null) {
            triggerAction.action.performed += OnTriggerPressed;
            triggerAction.action.canceled += OnTriggerPressed;
        }

        if (gripAction != null) {
            gripAction.action.performed += OnGripPressed;
            gripAction.action.canceled += OnGripPressed;
        }
    }

    private void OnTriggerPressed(InputAction.CallbackContext obj) {
        _triggerPressed = obj.performed;
    }

    private void OnGripPressed(InputAction.CallbackContext obj) {
        _gripPressed = obj.performed;
    }

    private void OnMainButtonPressed(InputAction.CallbackContext _) {
        OnPrimaryButtonEvent e = new OnPrimaryButtonEvent(side, primaryButtonAction.action.triggered);
        EventBus<OnPrimaryButtonEvent>.Raise(e);
    }

    private void OnSecondaryButtonPressed(InputAction.CallbackContext obj) {
        OnSecondaryButtonEvent e = new OnSecondaryButtonEvent(side, secondaryButtonAction.action.triggered);
        EventBus<OnSecondaryButtonEvent>.Raise(e);
    }

    private void Update() {
        if (_rtcDataChannel == null) return;
        UpdatePosition();
        CheckOnGrab();
        UpdateStick();
    }

    private void UpdateStick() {
        if (primary2DAxisAction == null) return;
        var dir = primary2DAxisAction.action.ReadValue<Vector2>();
        OnStickEvent e = new OnStickEvent(side, dir);
        EventBus<OnStickEvent>.Raise(e);
    }

    private void UpdatePosition() {
        if(positionAction == null || rotationAction == null) return;
        PositioningControllerEvent e =  new PositioningControllerEvent(side, positionAction.action.ReadValue<Vector3>(), rotationAction.action.ReadValue<Quaternion>());
        EventBus<PositioningControllerEvent>.Raise(e);
    }

    private void CheckOnGrab() {
        var newState = _triggerPressed && _gripPressed;
        if (newState != _lastDeadMenState) {
            _lastDeadMenState = newState;
            OnGrabEvent e = new OnGrabEvent(side, _lastDeadMenState);
            EventBus<OnGrabEvent>.Raise(e);
        }
    }
}
