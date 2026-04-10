using System;
using System.Collections;
using System.Collections.Generic;
using System.Net;
using System.Text;
using Events;
using Events.Types;
using Unity.WebRTC;
using UnityEngine;

namespace Connection {
    public class WebRtcConnector : MonoBehaviour {

        private RTCPeerConnection _peer;
        
        private RTCSessionDescription _remoteNatDescription = new();
        private RTCSessionDescription _localNatDescription = new();
        
        private RTCDataChannel _dataChannel;

        private List<RTCIceCandidate> _iceBuffer = new();

        public string LocalNatDescription => _localNatDescription.sdp;

        private Listener<OnIceCandidateReceiveEvent> _listener;
        
        private VideoStreamTrack _videoStreamTrack;
        
        static bool TryGetIpFromCandidate(string candidate, out IPAddress ip) {
            ip = null;
            if (string.IsNullOrEmpty(candidate)) return false;

            var parts = candidate.Split(' ', StringSplitOptions.RemoveEmptyEntries);
            if (parts.Length < 6) return false;

            var addr = parts[4];

            if (addr.Length > 2 && addr[0] == '[' && addr[^1] == ']')
                addr = addr.Substring(1, addr.Length - 2);

            return IPAddress.TryParse(addr, out ip);
        }

        static bool IsIPv4(string candidate) {
            return TryGetIpFromCandidate(candidate, out var ip) && ip.AddressFamily == System.Net.Sockets.AddressFamily.InterNetwork;
        }

        static bool IsIPv6(string candidate) {
            return TryGetIpFromCandidate(candidate, out var ip) && ip.AddressFamily == System.Net.Sockets.AddressFamily.InterNetworkV6;
        }

        public IEnumerator InitWithDescription(string description) {
            InitListeners();
            RaiseLifecycleEvent(WebRtcLifecycleState.Connecting);
            var config = new RTCConfiguration { iceServers = new[] {
                new RTCIceServer { urls = new[] { "##your TURN here##" }, username = "##your TURN user here##", credential = "##your TURN credential here##" }
            } };

            _peer = new RTCPeerConnection(ref config);
            _peer.OnIceConnectionChange = s => {
                Debug.Log("ICE conn: " + s);
                RaiseLifecycleEvent(MapIceState(s));
            };
            _peer.OnConnectionStateChange = s => {
                Debug.Log("PC state: " + s);
                RaiseLifecycleEvent(MapPeerState(s));
            };
            _peer.OnIceGatheringStateChange = s => Debug.Log("ICE gathering: " + s);

            _peer.OnTrack = e => {
                if (e.Track is VideoStreamTrack video) {
                    OnVideoTrackObtained @event = new OnVideoTrackObtained(video);
                    EventBus<OnVideoTrackObtained>.Raise(@event);
                }
            };
            
            _peer.OnDataChannel = ch => {
                OnDataChannelCreated channelCreated = new OnDataChannelCreated(ch);
                EventBus<OnDataChannelCreated>.Raise(channelCreated);
                _dataChannel = ch;
                _dataChannel.OnMessage += OnMessage;
            };
            _peer.OnIceCandidate = cand => {
                if (cand != null) {
                    
                    if (IsIPv6(cand.Candidate)) return;
                    if (!RTCIceProtocol.Udp.Equals(cand.Protocol)) return;
                    
                    Debug.Log($"Sending ICE candidate: {cand.Candidate}, SdpMid: {cand.SdpMid},  SdpMLineIndex: {cand.SdpMLineIndex}");
                    OnIceCandidateEvent e = new OnIceCandidateEvent(cand.Candidate, cand.SdpMid, cand.SdpMLineIndex ?? 0);
                    EventBus<OnIceCandidateEvent>.Raise(e);
                }
            };
            description = description.Replace("\r\n", "\n").Replace("\n", "\r\n"); 
            _remoteNatDescription = new RTCSessionDescription { type = RTCSdpType.Offer, sdp = description, }; 
            var setRemoteOp = _peer.SetRemoteDescription(ref _remoteNatDescription); 
            yield return setRemoteOp;
            if (setRemoteOp.IsError) {
                Debug.LogError($"SetRemoteDescription failed: {setRemoteOp.Error.message}"); 
                yield break;
            }
            
            var answer = _peer.CreateAnswer(); 
            yield return answer;
            if (answer.IsError) {
                Debug.LogError($"CreateAnswer failed: {answer.Error.message}"); 
                yield break;
            } 
            _localNatDescription = answer.Desc; 
            var localDescription = _peer.SetLocalDescription(ref _localNatDescription); 
            yield return localDescription;
            if (localDescription.IsError) {
                Debug.LogError($"SetLocalDescription failed: {localDescription.Error.message}"); 
                yield break;
            }

            yield return null;
        }

        private void InitListeners() {
            _listener = new Listener<OnIceCandidateReceiveEvent>(OnIceCandidate);
            EventBus<OnIceCandidateReceiveEvent>.Register(_listener, this);
        }

        private void OnIceCandidate(OnIceCandidateReceiveEvent e) {
            var candidate = e.iceCandidate;
            _iceBuffer.Add(candidate);
        }

        private void Update() {
            if (_peer.GatheringState != RTCIceGatheringState.Gathering || _iceBuffer.Count < 1) return;
            var candidate = _iceBuffer[0];
            _peer.AddIceCandidate(candidate);
            _iceBuffer.RemoveAt(0);
        }

        private void OnMessage(byte[] bytes) {
            string message = Encoding.UTF8.GetString(bytes, 0, bytes.Length);
            OnMessageObtained messageObtained = new OnMessageObtained(_dataChannel.Label,  message);
            EventBus<OnMessageObtained>.Raise(messageObtained);
        }

        private static WebRtcLifecycleState MapPeerState(RTCPeerConnectionState state) {
            return state switch {
                RTCPeerConnectionState.New => WebRtcLifecycleState.Connecting,
                RTCPeerConnectionState.Connecting => WebRtcLifecycleState.Connecting,
                RTCPeerConnectionState.Connected => WebRtcLifecycleState.Connected,
                RTCPeerConnectionState.Disconnected => WebRtcLifecycleState.Disconnected,
                RTCPeerConnectionState.Failed => WebRtcLifecycleState.Failed,
                RTCPeerConnectionState.Closed => WebRtcLifecycleState.Closed,
                _ => WebRtcLifecycleState.Connecting
            };
        }

        private static WebRtcLifecycleState MapIceState(RTCIceConnectionState state) {
            return state switch {
                RTCIceConnectionState.New => WebRtcLifecycleState.Connecting,
                RTCIceConnectionState.Checking => WebRtcLifecycleState.Connecting,
                RTCIceConnectionState.Connected => WebRtcLifecycleState.Connected,
                RTCIceConnectionState.Completed => WebRtcLifecycleState.Connected,
                RTCIceConnectionState.Disconnected => WebRtcLifecycleState.Disconnected,
                RTCIceConnectionState.Failed => WebRtcLifecycleState.Failed,
                RTCIceConnectionState.Closed => WebRtcLifecycleState.Closed,
                _ => WebRtcLifecycleState.Connecting
            };
        }

        private static void RaiseLifecycleEvent(WebRtcLifecycleState state) {
            EventBus<WebRtcLifecycleEvent>.Raise(new WebRtcLifecycleEvent(state));
        }
    }
}
