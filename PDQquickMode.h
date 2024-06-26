#ifndef PDQHEAP_H
#define PDQHEAP_H
#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <ctime>

/*
This is a custom class used to implement a priority deque, it is a min heap that can be used to store the top k elements of a stream of data.

Author: Rawad Altagiuri
*/

template <class T>
class priority_deque {
    public:
    std::vector<T>vec; //the heap    
    int maxSize;
    int currLeaf; //the leaf we switch with the new value when we reach maximum size.
    bool check; //this is to keep check if we either found the leaves indexes, or the leaves values according to our mode.
    int firstLeafIndex; //the index of the first leaf node.
   
    public:
        //constructor and destructor
        priority_deque();// max size set to 100 by default, mode set to normal by default
        priority_deque(int);//for setting the max size, mode set to normal by default
        ~priority_deque();//

        
        //Operations
        T poll(); //returns and removes the top element.
        void add(T); 
        void clear(); //clears the heap
        void setMaxSize(int); //sets the maximum size of the heap

        //Util
        int size(); //
        bool empty(); //true if empty
        T peek();


    private:
        void bubbleUp(int);
        void bubbleDown(int);
        int findLeavesIndexes();
        void addingAtMaxSize(T);
        
};


/*
initilizes the priority deque
Time : O(1)
Space : O(1)
*/
template <class T>
priority_deque<T>::priority_deque(){
    this->maxSize = 100;
    this->check = false;
    // this->quickMode = false;
    this->currLeaf = 0;
    this->firstLeafIndex = 0;
}

/*
initilizes the priority deque with a max size
TIME: O(1)
SPACE: O(1)
*/
template <class T>
priority_deque<T>::priority_deque(int maxSize){
    this->maxSize = maxSize;
    this->check = false;
    // this->quickMode = false;
    this->currLeaf = 0;
    this->firstLeafIndex = 0;
}

/*
destructs the priority deque
TIME: O(N)
SPACE: O(1)
*/
template <class T>
priority_deque<T>::~priority_deque(){
    this->vec.clear();
}

/*
returns the highest priority value
TIME: O(1)
SPACE: O(1)
*/
template <class T>
T priority_deque<T>::peek(){
    if(this->vec.size()==0){
        throw std::runtime_error("Priority Queue is empty");
    }
    return this->vec[0];
}



/*
sets the max size of the priority deque
TIME: O(1)
SPACE: O(1)
*/
template <class T>
void priority_deque<T>::setMaxSize(int maxSize){
    this->check = false; //we reset the flag to false so we can get the indexes of the leaves after changing the size.
    this->maxSize = maxSize;
}

/*
TIME : O(1)
SPACE : O(1)
*/
template <class T> //O(1)
bool priority_deque<T>::empty(){
    return this->vec.size() == 0;
}

/*
TIME: O(N)
SPACE: O(1)
*/
template <class T> //O(1)
void priority_deque<T>::clear(){
    this->vec.clear();
}

/*
TIME: O(1)
SPACE: O(1)
*/
template <class T>
int priority_deque<T>::size(){
    return this->vec.size();
}



/*
removes the highest priority element and returns it.
TIME: O(Log n)
SPACE: O(1)
*/
template <class T>
T priority_deque<T>::poll() {
    if(this->vec.size() == 0){
        throw std::runtime_error("Priority Queue is empty");
    }
    T temp = this->vec[0];
    this->vec[0] = this->vec[this->vec.size() - 1];
    this->vec.pop_back();
    this->bubbleDown(0);
    return temp;
}


/*
If we did not reach the max element it just adds and element and bubbles it up O(log n) time, O(1) space
If we reach the maximum size there are 2 cases:
1) QuickMode = true. in this mode we choose a random leaf and then replace it with the new element and bubble it up one O(log n) operation and O(1) space. This results in some new values getting lost if the branch it gets assigned to is full of nodes equal to the new value. This is better used in cases where you might NOT have multiple duplicates.
2) QuickMode = false. in this mode we choose the largest leaf and replace it with the new value, this Mode guarantees all added values will end up inside the Tree. This is better used in cases where you might have multiple duplicates.
TIME : O(LOG N)
SPACE: O(1)
*/
template <class T>
void priority_deque<T>::add(T data){
     if(this->maxSize == 0) return; //if for some reason someone put the maxSize as 0 i guess.

     if(this->vec.size() < this->maxSize){ //if we haven't reached the maximum size yet, we just add the value to the end of the vector and bubble it up.
        this->vec.push_back(data);
        this->bubbleUp(this->vec.size()-1);
        return;
     }

     if(!this->check){ //If we are in quickMode we need to find the index of the first leaf
        this->firstLeafIndex = findLeavesIndexes();
        this->check=true;
     }

    this->addingAtMaxSize(data); //If we are in quickMode
}



/*
Adding for QuickMode after we reach max size
TIME: O(LOG N)
SPACE: O(1)
*/
template <class T>
void priority_deque<T>::addingAtMaxSize(T data){
    //srand(static_cast<unsigned int>(time(0)));
    //if we reach the maximum size, we start switching out the leaf nodes with the new values, this assure that all the new values we insert can go into all the branches.
    this->currLeaf = this->firstLeafIndex + (rand() % (this->vec.size() - this->firstLeafIndex));
    this->vec[currLeaf] = data;
    this->bubbleUp(currLeaf);
}

/*
Bubbling up values to keep the heap property
TIME: O(Log n)
SPACE: O(1)
*/
template <class T>
void priority_deque<T>::bubbleUp(int index){
    int parent = 0;
    if(index%2==1){
        parent = (index-1)/2;
    }
    else{
        parent = (index-2)/2;
    }
    
    while(parent >= 0 && this->vec[index].first[0] < this->vec[parent].first[0]){ //we swap the value with the parent until the parent is smaller than the current value or we reach the root.
        std::swap(this->vec[index], this->vec[parent]);
        index = parent;
        if(index%2==1){
            parent = (index-1)/2;
        }
        else{
            parent = (index-2)/2;
        }
    }
}

/*
Bubbling down values to keep the heap property
TIME: O(Log n)
SPACE: O(1)
*/
template <class T>
void priority_deque<T>::bubbleDown(int index) { 
    int size = this->vec.size();
    while (true) {
        int left = 2 * index + 1;
        int right = 2 * index + 2;
        int smallest = index;
        
        if (left < size && this->vec[left].first[0] < this->vec[smallest].first[0]) {
            smallest = left;
        }
        
        if (right < size && this->vec[right].first[0] < this->vec[smallest].first[0]) { // not sure about this, before editing it was this->vec[right]
            smallest = right;
        }
        
        if (smallest != index) {
            std::swap(this->vec[index], this->vec[smallest]);
            index = smallest;
        } else {
            break;
        }
    }
}

/*
Finding the first leaf node in the structure, this is used to choose a random node between the leaf nodes.
TIME: O(Log n)
SPACE: O(1)
*/
template<class T>
int priority_deque<T>::findLeavesIndexes(){
    int minLeave = 0;
    for(int i = this->vec.size()-1; i >= 0; i--){
        int left = (2*i)+1;
        int right = (2*i)+2;
        if(left >= this->vec.size() && right >=this->vec.size()){
            minLeave = i;
        }
    }
    return minLeave;
}





#endif