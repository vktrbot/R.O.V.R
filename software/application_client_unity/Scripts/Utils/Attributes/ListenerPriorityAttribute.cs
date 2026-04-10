using System;
using Events;

namespace Utils.Attributes {
    [System.AttributeUsage(System.AttributeTargets.Field)]
    public class ListenerPriorityAttribute : Attribute {
        public Priority Priority { get; }

        public ListenerPriorityAttribute(Priority priority) {
            Priority = priority;
        }
    }
}