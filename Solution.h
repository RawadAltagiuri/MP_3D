#include "Cost.h"

template<typename T>
struct Solution {
    public:
        Solution(Solution<T>* prevSolution, const Cost& cost, const T& config);

        Solution(Solution<T>&& other);
        Solution(const Solution<T>& other);

        Solution<T>& operator=(Solution<T>&& other) = delete;
        Solution<T>& operator=(const Solution<T>& other) = delete;

        void addChildren();
        void removeChildren();

        bool isTerminal();

        //Operators
        bool operator<(const Solution<T>& other) const {
            return this->cost.totalCost < other.cost.totalCost;
        }

        bool operator==(const Solution<T>& other) const {
            return this->cost.totalCost == other.cost.totalCost;
        }

    public:
        Solution<T>* prevSolution;
        const Cost cost;
        T config;

    private:
        unsigned int activeChildren;
};

template<typename T>
Solution<T>::Solution(Solution<T>* prevSolutionA, const Cost& costA, const T& configA): 
    prevSolution{prevSolutionA},
    cost{costA}, 
    config{configA}, 
    activeChildren{0} {}

template<typename T>
Solution<T>::Solution(Solution<T>&& other): prevSolution{other.prevSolution}, cost{other.cost}, config{other.config} {}

template<typename T>
Solution<T>::Solution(const Solution<T>& other): prevSolution{other.prevSolution}, cost{other.cost}, config{other.config} {}

template<typename T>
void Solution<T>::addChildren() {
    ++activeChildren;
}

template<typename T>
void Solution<T>::removeChildren() {
    --activeChildren;
}

template<typename T>
bool Solution<T>::isTerminal() {
    return activeChildren == 0;
}


