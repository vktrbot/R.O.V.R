using System;
using Utils.Attributes;

namespace Events {
    public class EventBusContext<T> where T : IEvent {
        public ErrorPolicy ErrorPolicy;
        public string EventName;
        public bool DoNotLog;
        public bool IgnoreErrorLog;

        public EventBusContext() {
            var policyAttribute = Attribute.GetCustomAttribute(typeof(T), typeof(ErrorHandlingAttribute)) as ErrorHandlingAttribute;
            ErrorPolicy = policyAttribute?.Policy ?? ErrorPolicy.SWALLOW;
            DoNotLog = Attribute.IsDefined(typeof(T), typeof(DoNotLogAttribute));
            IgnoreErrorLog = Attribute.IsDefined(typeof(T), typeof(IgnoreErrorLogAttribute));
            var name = Attribute.GetCustomAttribute(typeof(T), typeof(NameAttribute)) as NameAttribute;
            if (name == null) {
                EventName = typeof(T).Name;
            } else {
                EventName = name.Name;
            }
        }
    }
}