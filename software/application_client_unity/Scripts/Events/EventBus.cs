using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using Events.Policies.Errors;
using UnityEngine;
using utils;
using Utils.Attributes;

namespace Events {
    public static class EventBus<T> where T : IEvent {
        
        private static readonly SortedList<ListenerCell> _listeners = new(new PriorityComparator());

        private static readonly EventBusContext<T> _context = new();
        
        public static void Raise(T @event) {
            Guid id = Guid.NewGuid();
            var hash = id.GetHashCode() & 0xFFFF;
            var executionPolicy = ErrorPolicyTable.GetHandler(_context.ErrorPolicy);
            if (!_context.DoNotLog) {
                Debug.Log($"[{_context.EventName}-{hash:00000}] Start handling event. Params: {@event}");
            }
            foreach (var cell in _listeners) {
                var result = executionPolicy.ExecuteHandle(() => {
                    cell.Listener.OnEvent(@event);
                    cell.Listener.OnEventNoArgs();
                });
                if (!result.Handled && !_context.IgnoreErrorLog) {
                    Debug.LogError($"[{_context.EventName}-{hash:00000}] Error while handling event. {result.Exception}");
                }
            }
            if (_context.DoNotLog) return;
            Debug.Log($"[{_context.EventName}-{hash:00000}] Event handled. Params: {@event}");
        }

        
        
        public static void Register(IEventListener<T> listener, object owner) {
            ListenerCell cell = new ListenerCell(listener, owner);
            _listeners.Add(cell);
        }
        
        public static void Unregister(IEventListener<T> listener) {
            ListenerCell cell = _listeners.FirstOrDefault(l => ReferenceEquals(l.Listener, listener));
            _listeners.Remove(cell);
        }

        private class ListenerCell {
            public readonly IEventListener<T> Listener;
            public readonly object Owner;

            public ListenerCell(IEventListener<T> listener, object owner) {
                Listener = listener;
                Owner = owner;
            }
        }
        
        private class PriorityComparator : IComparer<ListenerCell> {
            public int Compare(ListenerCell x, ListenerCell y) {
                if (x == null) return -1;
                if (y == null) return 1;
                Priority xEventPriority = ResolvePriorityFromOwnerField(x.Owner, x.Listener) ?? Priority.LOW;
                Priority yEventPriority = ResolvePriorityFromOwnerField(y.Owner, y.Listener) ?? Priority.LOW;
                return xEventPriority - yEventPriority;
            }
            
            private static Priority? ResolvePriorityFromOwnerField(object owner, IEventListener<T> instance) {
                    var flags = BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic;
                    foreach (var f in owner.GetType().GetFields(flags)) {
                        if (!typeof(IEventListener<T>).IsAssignableFrom(f.FieldType)) continue;
                        var value = f.GetValue(owner);
                        if (!ReferenceEquals(value, instance)) continue;
            
                        var attr = f.GetCustomAttribute<ListenerPriorityAttribute>(inherit: true);
                        return attr?.Priority;
                    }
                    return null;
                }
        }
        
        public static void Clear() {
            _listeners.Clear();
        }
    }
}