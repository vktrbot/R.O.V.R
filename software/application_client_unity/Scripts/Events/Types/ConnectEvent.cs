using Unity.WebRTC;
using Utils.Attributes;

namespace Events.Types {

    public enum ConnectEventType {
        CONNECTED,
        CONNECTING,
        DISCONNECTED,
        DISCONNECTING
    }
    
    [DoNotLog]
    public class ConnectEvent : IEvent {
        public ConnectEventType Type;

        public ConnectEvent(ConnectEventType type) {
            Type = type;
        }
    }

    [DoNotLog]
    public class ConnectSocketServerEvent : IEvent {
        public string Secret;

        public ConnectSocketServerEvent(string secret) {
            Secret = secret;
        }
    }

    [DoNotLog]
    public class JoinLobbyEvent : IEvent {
        public string LobbyID;

        public JoinLobbyEvent(string lobbyID) {
            LobbyID = lobbyID;
        }
    }

    public class OnVideoTrackObtained : IEvent {
        public VideoStreamTrack Track;

        public OnVideoTrackObtained(VideoStreamTrack track) {
            Track = track;
        }
    }
    
    [DoNotLog]
    public class OnIceCandidateEvent : IEvent {
        public string candidate;
        public string sdpMid;
        public int sdpMLineIndex;

        public OnIceCandidateEvent(string candidate, string sdpMid, int sdpMLineIndex) {
            this.candidate = candidate;
            this.sdpMid = sdpMid;
            this.sdpMLineIndex = sdpMLineIndex;
        }
    }

    [DoNotLog]
    public class OnIceCandidateReceiveEvent : IEvent {
        public RTCIceCandidate iceCandidate;

        public OnIceCandidateReceiveEvent(RTCIceCandidate iceCandidate) {
            this.iceCandidate = iceCandidate;
        }
    }
    
}