using System;
using Events;

namespace Utils.Attributes {
    [AttributeUsage(AttributeTargets.Class)]
    public class ErrorHandlingAttribute : Attribute {
        public ErrorPolicy Policy { get; }
        public ErrorHandlingAttribute(ErrorPolicy policy) {
            Policy = policy;
        }
    }
}