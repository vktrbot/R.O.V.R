using System;
using System.Collections.Generic;
using System.Text;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using UnityEngine;
using utils;

namespace Connection.Types {
    
    
    public class ChallengeTypeSerialization : INetworkSerializable {

        private string _deviceId;
        private string _signature;
        
        private struct Answer {
            public string device_id;
            public string signature;
            public string type;

            public Answer(string type, string deviceId, string signature) {
                this.type = type;
                this.device_id = deviceId;
                this.signature = signature;
            }
        }
        
        public ChallengeTypeSerialization(string deviceId, string signature) {
            this._deviceId = deviceId;
            this._signature = signature;
        }
        
        public string Serialize() {
            var answer = new Answer(type: "auth", deviceId: _deviceId, signature: _signature);
            return JsonConvert.SerializeObject(answer);
        }
    }
    
    public class ChallengeTypeHandler : IWebSocketTypeHandler {
        public HandlerResult Handle(Dictionary<string, object> args) {
            var contentObj = args["content"] as JObject;
            if (contentObj == null) {
                return new HandlerResult {IsHandled = false, Status = HandlerStatus.ERR, ErrorMessage = "Content of the payload is not defined"};
            }
            
            var content =  contentObj.ToObject<Dictionary<string, object>>();
            
            if (!content.TryGetValue("nonce", out var nonce) || !content.TryGetValue("timestamp", out var timestamp)) {
                return new HandlerResult { IsHandled = false, Status = HandlerStatus.ERR, ErrorMessage = $"Nonce {nonce} or timestamp is not presented." };
            }
            
            var encoded = HMACEncoder.GenerateAnswer("user-1", nonce as string, (long)timestamp);
            INetworkSerializable serializer = new ChallengeTypeSerialization("user-1", encoded);
            return new HandlerResult { IsHandled = true, Answer = serializer, Status = HandlerStatus.OK};
        }
    }
}