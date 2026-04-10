using System;

namespace Events {
    public interface IEvent { }
    public interface ICancellable {
        bool Cancelled { get; set; }
    }

    public interface IEventListener<T> where T : IEvent {
        
        /// <summary>
        /// Represents an event handler for a specific event type.
        /// The OnEvent property allows attaching an action that takes a single argument of type T to handle the event.
        /// </summary>
        public Action<T> OnEvent { get; set; }

        /// <summary>
        /// Represents an event binding with no arguments. This is used to subscribe to and trigger events that do not require any specific data.
        /// </summary>
        public Action OnEventNoArgs { get; set; }
    }
}