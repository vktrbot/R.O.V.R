using System.Collections.Generic;

namespace Events.Policies.Errors {
    public static class ErrorPolicyTable {
        private static Dictionary<ErrorPolicy, IErrorPolicyHandler> _handlers;

        static ErrorPolicyTable() {
            _handlers = new Dictionary<ErrorPolicy, IErrorPolicyHandler>();
            _handlers.Add(ErrorPolicy.SWALLOW, new SwallowErrorPolicy());
            _handlers.Add(ErrorPolicy.HALT, new HaltErrorPolicy());
            _handlers.Add(ErrorPolicy.REPEAT, new RepeatErrorPolicy());
        }
        
        public static IErrorPolicyHandler GetHandler(ErrorPolicy policy) {
            return _handlers[policy];
        }
    }
}