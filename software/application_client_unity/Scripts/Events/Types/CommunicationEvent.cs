using Unity.WebRTC;

namespace Events.Types {
    public enum WebRtcLifecycleState {
        Connecting,
        Connected,
        Disconnected,
        Failed,
        Closed
    }

    public class OnDataChannelCreated : IEvent {
        public RTCDataChannel Channel;
        public OnDataChannelCreated(RTCDataChannel channel) {
            Channel = channel;
        }
    }

    public class OnMessageObtained : IEvent {
        public string ChannelName;
        public string Message;

        public OnMessageObtained(string channelName, string message) {
            ChannelName = channelName;
            Message = message;
        }
    }

    public class WebRtcLifecycleEvent : IEvent {
        public WebRtcLifecycleState State;

        public WebRtcLifecycleEvent(WebRtcLifecycleState state) {
            State = state;
        }
    }
}
