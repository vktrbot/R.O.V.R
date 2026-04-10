using System.Collections.Generic;

namespace Connection.Types {
    public class AuthTypeHandler : IWebSocketTypeHandler {
        public HandlerResult Handle(Dictionary<string, object> args) {
            
            return new HandlerResult {
                Status = HandlerStatus.OK,
                IsHandled = true
            };
        }
    }
}