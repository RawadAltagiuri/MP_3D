#include <vector>
#include <exception>
#include <math.h>
#include <limits>

/*
Implementation of a priority queue with min-max binary-heap.
*/
template<typename T>
class PriorityQueue {
    public:
        //Constructors
        PriorityQueue(size_t maxSizeA = std::numeric_limits<unsigned int>::max());
        PriorityQueue(std::initializer_list<T>);

        //Operations
        void setMaxSize(size_t);
        void insert(const T&);
        T poll();

        //Accessors
        T peek() const;
        const std::vector<T>& inner() const;
        T at(size_t) const;
        size_t size() const;
        bool empty() const;
        bool full() const;

        //Destructors.
        ~PriorityQueue();
    
    private:
        size_t getParent(size_t);
        size_t getFirstChild(size_t);

        bool isEvenLevel(size_t);

        void bubbleUp(size_t);
        void bubbleDown(size_t);

        void bubbleUpMin(size_t);
        void bubbleUpMax(size_t);

        void bubbleDownMin(size_t);
        void bubbleDownMax(size_t);
    private:
        std::vector<T> v;
        size_t maxSize;
};

template<typename T>
PriorityQueue<T>::PriorityQueue(size_t maxSizeA): v{}, maxSize{maxSizeA} {}

template<typename T>
PriorityQueue<T>::PriorityQueue(std::initializer_list<T> list): v{}, maxSize{list.size()} {
    for (auto it : list) {
        insert(it);
    }
}

template<typename T>
void PriorityQueue<T>::setMaxSize(size_t maxSizeA) {
    maxSize = maxSizeA;
}

template<typename T>
void PriorityQueue<T>::insert(const T& element) {
    if (full() && v.size() > 2) {
        size_t max {v[1] < v[2] ? size_t{2} : size_t{1}};
        v[max] = v[v.size() - 1];
        v.pop_back();
        bubbleDown(max);
    }

    v.push_back(element);
    bubbleUp(v.size() - 1);
}

template<typename T>
T PriorityQueue<T>::poll() {
    if (empty())
        throw std::exception{};
    
    T root {std::move(v[0])};
    v[0] = std::move(v[v.size() - 1]);
    v.pop_back();
    bubbleDown(0);

    return root;
}

template<typename T>
T PriorityQueue<T>::peek() const {
    if (empty())
        throw std::exception{};
    return v[0];
}

template<typename T>
const std::vector<T>& PriorityQueue<T>::inner() const {
    return v;
}

template<typename T>
T PriorityQueue<T>::at(size_t index) const {
    return v[index];
}

template<typename T>
size_t PriorityQueue<T>::size() const {
    return v.size();
}

template<typename T>
bool PriorityQueue<T>::empty() const {
    return v.empty();
}

template<typename T>
bool PriorityQueue<T>::full() const {
    return v.size() >= maxSize;
}

template<typename T>
PriorityQueue<T>::~PriorityQueue() {};

template<typename T>
size_t PriorityQueue<T>::getParent(size_t index) {
    if (index == 0)
        return 0;
    else
        return (index - 1) / 2;
}

template<typename T>
size_t PriorityQueue<T>::getFirstChild(size_t index) {
    return index * 2 + 1;
}

template<typename T>
bool PriorityQueue<T>::isEvenLevel(size_t index) {
    size_t level {size_t(std::log2(index + 1))};
    return level % 2 == 0; 
}

template<typename T>
void PriorityQueue<T>::bubbleUp(size_t index) {
    if (index == 0)
        return;

    size_t parent {getParent(index)};
    if (parent >= 0) {
        if (isEvenLevel(index)) {
            if (v[parent] < v[index]) {
                std::swap(v[index], v[parent]);
                bubbleUpMax(parent); 
            } else 
                bubbleUpMin(index);
        } else {
            if (v[index] < v[parent]) {
                std::swap(v[index], v[parent]);
                bubbleUpMin(parent);
            } else { 
                bubbleUpMax(index);
            }
        }
    }
}

template<typename T>
void PriorityQueue<T>::bubbleUpMin(size_t index) {
    size_t parent {getParent(index)};
    if (parent == 0)
        return;

    size_t grandParent {getParent(parent)};
    
    if (v[index] < v[grandParent]) {
        std::swap(v[index], v[grandParent]);
        bubbleUpMin(grandParent);
    }
}

template<typename T>
void PriorityQueue<T>::bubbleUpMax(size_t index) {
    size_t parent {getParent(index)};
    if (parent == 0)
        return;

    size_t grandParent {getParent(parent)};
    
    if (v[grandParent] < v[index]) {
        std::swap(v[index], v[grandParent]);
        bubbleUpMax(grandParent);
    }    
}

template<typename T>
void PriorityQueue<T>::bubbleDown(size_t index) {
    if (isEvenLevel(index))
        bubbleDownMin(index);
    else
        bubbleDownMax(index);
}

template<typename T>
void PriorityQueue<T>::bubbleDownMin(size_t index) {
    size_t child1 {getFirstChild(index)};
    size_t child2 {child1 + 1};

    size_t grandChild11 {getFirstChild(child1)};
    if (grandChild11 < v.size()) {
        size_t minGrandChild {grandChild11};

        for (size_t i = grandChild11 + 1; i < grandChild11 + 4 && i < v.size(); i++) {
            minGrandChild = v[i] < v[minGrandChild] ? i : minGrandChild;
        }

        if (v[minGrandChild] < v[index]) {
            std::swap(v[minGrandChild], v[index]);
            if (v[getParent(minGrandChild)] < v[minGrandChild])
                std::swap(v[getParent(minGrandChild)], v[minGrandChild]);
            bubbleDown(minGrandChild);
        }
    } else if (child1 < v.size()) {
        size_t minChild {child1};
        if (child2 < v.size())
            minChild = v[child1] < v[child2] ? child1 : child2;

        if (v[minChild] < v[index])
            std::swap(v[minChild], v[index]);
    }
}

template<typename T>
void PriorityQueue<T>::bubbleDownMax(size_t index) {
    size_t child1 {getFirstChild(index)};
    size_t child2 {child1 + 1};

    size_t grandChild11 {getFirstChild(child1)};
    if (grandChild11 < v.size()) {
        size_t maxGrandChild {grandChild11};

        for (size_t i = grandChild11 + 1; i < grandChild11 + 4 && i < v.size(); i++) {
            maxGrandChild = v[maxGrandChild] < v[i] ? i : maxGrandChild;
        }

        if (v[index] < v[maxGrandChild]) {
            std::swap(v[maxGrandChild], v[index]);
            if (v[maxGrandChild] < v[getParent(maxGrandChild)])
                std::swap(v[getParent(maxGrandChild)], v[maxGrandChild]);
            bubbleDown(maxGrandChild);
        }
    } else if (child1 < v.size()) {
        size_t maxChild {child1};
        if (child2 < v.size())
            maxChild = v[child2] < v[child1] ? child1 : child2;

        if (v[index] < v[maxChild])
            std::swap(v[maxChild], v[index]);
    }
}