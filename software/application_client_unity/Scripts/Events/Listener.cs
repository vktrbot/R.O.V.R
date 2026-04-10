using System;

namespace Events {
    public class Listener<T> : IEventListener<T> where T : IEvent {
        public Action<T> OnEvent { get; set; } = _ => {};
        public Action OnEventNoArgs { get; set; } = () => {};

        public Listener(Action<T> @event) {
            OnEvent = @event;
        }
        
        public Listener(Action @event) {
            OnEventNoArgs = @event;
        }
        
        public Listener() {}

        public void Add(Action onEvent) => OnEventNoArgs += onEvent;
        public void Remove(Action onEvent) => OnEventNoArgs -= onEvent;
        
        public void Add(Action<T> onEvent) => OnEvent += onEvent;
        public void Remove(Action<T> onEvent) => OnEvent -= onEvent;
    }
}