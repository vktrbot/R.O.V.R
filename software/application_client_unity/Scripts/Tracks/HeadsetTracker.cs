using Data;
using Events;
using Events.Types;
using Newtonsoft.Json;
using Unity.WebRTC;
using UnityEngine;
using UnityEngine.InputSystem;
using Utils;

public class HeadsetTracker : MonoBehaviour {
    
    [SerializeField] private InputActionReference headsetRotation;

    private Listener<OnDataChannelCreated> _dataChannelCreated;
    private RTCDataChannel _rtcDataChannel;

    private HeadsetMove _headsetMovement;

    private void Awake() {
        _dataChannelCreated = new Listener<OnDataChannelCreated>(OnConnected);
        EventBus<OnDataChannelCreated>.Register(_dataChannelCreated, this);
        _headsetMovement = new HeadsetMove();
    }

    private void OnConnected(OnDataChannelCreated e) {
        _rtcDataChannel = e.Channel;
        InitHeadset();
    }

    private void InitHeadset() {
        headsetRotation?.action.Enable();
    }

    private void Update() {
        if(_rtcDataChannel == null) return;
        if(headsetRotation == null) return;

        var rotation = headsetRotation.action.ReadValue<Quaternion>();
        var euler = rotation.eulerAngles;
        _headsetMovement.yaw_deg = -UnityUtils.NormalizeAngle(euler.y);
        _headsetMovement.pitch_deg = -UnityUtils.NormalizeAngle(euler.x);
        _rtcDataChannel.Send(JsonConvert.SerializeObject(Container.Of("camera", _headsetMovement)));
    }

    
}
