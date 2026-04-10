using System;

namespace Utils.Attributes {
    [AttributeUsage(AttributeTargets.Class)]
    public class NameAttribute : Attribute {
        public string Name;
        
        public NameAttribute(string name) {
            Name = name;
        }
    }
}