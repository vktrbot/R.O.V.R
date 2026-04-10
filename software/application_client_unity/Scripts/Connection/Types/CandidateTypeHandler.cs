using System.Collections.Generic;
using Events;
using Events.Types;
using Newtonsoft.Json.Linq;
using Unity.WebRTC;

namespace Connection.Types {
    public class CandidateTypeHandler : IWebSocketTypeHandler {

        public HandlerResult Handle(Dictionary<string, object> args) {
            if (args["content"] is not JObject contentObj) {
                return new HandlerResult { IsHandled = false, Status = HandlerStatus.ERR, ErrorMessage = "Content of the payload is not defined" };
            }

            var content = contentObj.ToObject<Dictionary<string, object>>();
            var candidate = content["candidate"] as string;
            var sdpMid = content["sdpMid"] as string;
            var sdpMLineIndex = content["sdpMLineIndex"] as int?;

            // Debug.Log($"Handling candidate: {candidate}, sdpMid: {sdpMid}, sdpMLineIndex: {sdpMLineIndex}");
            RTCIceCandidateInit init = new() {
                candidate = candidate,
                sdpMid = sdpMid,
                sdpMLineIndex = sdpMLineIndex
            };
            RTCIceCandidate iceCandidate = new(init);
            OnIceCandidateReceiveEvent e = new(iceCandidate);
            EventBus<OnIceCandidateReceiveEvent>.Raise(e);
            return new HandlerResult
            {
                Answer = null,
                IsHandled = true,
                Status = HandlerStatus.OK
            };
        }
    }
}
