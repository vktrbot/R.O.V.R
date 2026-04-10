using System;
using System.Collections;
using System.Collections.Generic;
using Events;
using MikeSchweitzer.WebSocket;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using UnityEngine;

namespace Connection.Types {
    public class OfferTypeHandler : IWebSocketTypeHandler {

        private class WebSocketInitializer : IUnityWorker {

            private string _natDescription;
            
            public WebSocketInitializer(string description) {
                    _natDescription = description;
            }

            [Serializable]
            public class AnswerMsg {
                public string type;
                public string description;
            }
            
            public IEnumerator Work(GameObject go, WebSocketConnection socket) {
                var connector = go.AddComponent<WebRtcConnector>();
                var worker = connector.InitWithDescription(_natDescription);
                yield return worker;
                while (worker.Current != null) {
                    yield return worker.Current;
                }
                
                var answer = connector.LocalNatDescription;
                var answerObj = new AnswerMsg { description = answer };
                var payload = new Payload("answer", answerObj);
                socket.AddOutgoingMessage(JsonConvert.SerializeObject(payload));
            }
        }
        
        public HandlerResult Handle(Dictionary<string, object> args) {
            var contentObj = args["content"] as JObject;
            if (contentObj == null) {
                return new HandlerResult {IsHandled = false, Status = HandlerStatus.ERR, ErrorMessage = "Content of the payload is not defined"};
            }

            var content = contentObj.ToObject<Dictionary<string, object>>();
            var description = content["description"] as string;
            if (description == null) {
                return new HandlerResult {
                    IsHandled = false,
                    ErrorMessage = "Offer description is not defined",
                    Status = HandlerStatus.ERR
                };
            }

            var initializer = new WebSocketInitializer(description);


            return new HandlerResult {
                IsHandled = true,
                Status = HandlerStatus.OK,
                Answer = initializer
            };

        }
    }
}