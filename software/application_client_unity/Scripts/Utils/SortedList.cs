using System.Collections;
using System.Collections.Generic;

namespace utils {
    public class SortedList<T> : ICollection<T> {
        private readonly List<T> _mInnerList;
        private readonly IComparer<T> _mComparer;

        public SortedList() : this(Comparer<T>.Default) { }

        public SortedList(IComparer<T> comparer) {
            _mInnerList = new List<T>();
            _mComparer = comparer;
        }

        public void Add(T item) {
            int insertIndex = FindIndexForSortedInsert(_mInnerList, _mComparer, item);
            _mInnerList.Insert(insertIndex, item);
        }

        public bool Contains(T item) {
            return IndexOf(item) != -1;
        }

        /// <summary>
        /// Searches for the specified object and returns the zero-based index of the first occurrence within the entire SortedList<T>
        /// </summary>
        public int IndexOf(T item) {
            int insertIndex = FindIndexForSortedInsert(_mInnerList, _mComparer, item);
            if (insertIndex == _mInnerList.Count) {
                return -1;
            }
            if (_mComparer.Compare(item, _mInnerList[insertIndex]) == 0) {
                int index = insertIndex;
                while (index > 0 && _mComparer.Compare(item, _mInnerList[index - 1]) == 0) {
                    index--;
                }
                return index;
            }
            return -1;
        }

        public bool Remove(T item) {
            int index = IndexOf(item);
            if (index >= 0) {
                _mInnerList.RemoveAt(index);
                return true;
            }
            return false;
        }

        public void RemoveAt(int index) {
            _mInnerList.RemoveAt(index);
        }

        public void CopyTo(T[] array) {
            _mInnerList.CopyTo(array);
        }

        public void CopyTo(T[] array, int arrayIndex) {
            _mInnerList.CopyTo(array, arrayIndex);
        }

        public void Clear() {
            _mInnerList.Clear();
        }

        public T this[int index] {
            get {
                return _mInnerList[index];
            }
        }

        public IEnumerator<T> GetEnumerator() {
            return _mInnerList.GetEnumerator();
        }

        IEnumerator IEnumerable.GetEnumerator() {
            return _mInnerList.GetEnumerator();
        }

        public int Count {
            get {
                return _mInnerList.Count;
            }
        }

        public bool IsReadOnly {
            get {
                return false;
            }
        }

        public static int FindIndexForSortedInsert(List<T> list, IComparer<T> comparer, T item) {
            if (list.Count == 0) {
                return 0;
            }

            int lowerIndex = 0;
            int upperIndex = list.Count - 1;
            int comparisonResult;
            while (lowerIndex < upperIndex) {
                int middleIndex = (lowerIndex + upperIndex) / 2;
                T middle = list[middleIndex];
                comparisonResult = comparer.Compare(middle, item);
                if (comparisonResult == 0) {
                    return middleIndex;
                }
                else if (comparisonResult > 0) {
                    upperIndex = middleIndex - 1;
                }
                else {
                    lowerIndex = middleIndex + 1;
                }
            }

            // At this point any entry following 'middle' is greater than 'item',
            // and any entry preceding 'middle' is lesser than 'item'.
            // So we either put 'item' before or after 'middle'.
            comparisonResult = comparer.Compare(list[lowerIndex], item);
            if (comparisonResult < 0) {
                return lowerIndex + 1;
            }
            else {
                return lowerIndex;
            }
        }
    }
}