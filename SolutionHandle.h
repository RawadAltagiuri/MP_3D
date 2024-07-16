#include "Solution.h"

template<typename T>
struct SolutionHandle {
    Solution<T>* sol;

    SolutionHandle(Solution<T>* solA);

    SolutionHandle(SolutionHandle<T>&& other);
    SolutionHandle(const SolutionHandle<T>& other);

    SolutionHandle<T>& operator=(SolutionHandle<T>&& other);
    SolutionHandle<T>& operator=(const SolutionHandle<T>& other);

    std::vector<Solution<T>*> backtractPath();

    void destroyHandle();

    bool operator<(const SolutionHandle<T>&) const;
    bool operator==(const SolutionHandle<T>&) const;

    private:
        void backtractPathR(std::vector<Solution<T>*>& v, Solution<T>* curSol);

    private:
        void removeSolution(Solution<T>*);
};

template<typename T>
SolutionHandle<T>::SolutionHandle(Solution<T>* solA): sol{solA} {};

template<typename T>
SolutionHandle<T>::SolutionHandle(SolutionHandle<T>&& other) {
    this->sol = other.sol;
}

template<typename T>
SolutionHandle<T>::SolutionHandle(const SolutionHandle<T>& other) {
    this->sol = other.sol;
}

template<typename T>
SolutionHandle<T>& SolutionHandle<T>::operator=(SolutionHandle<T>&& other) {
    this->sol = other.sol;
    return *this;
}

template<typename T>
SolutionHandle<T>& SolutionHandle<T>::operator=(const SolutionHandle<T>& other) {
    this->sol = other.sol;
    return *this;
}

template<typename T>
std::vector<Solution<T>*> SolutionHandle<T>::backtractPath() {
        std::vector<Solution<T>*> path {};
        backtractPathR(path, sol);
        return path;
}

template<typename T>
void SolutionHandle<T>::destroyHandle() {
    removeSolution(sol);
}

template<typename T>
bool SolutionHandle<T>::operator<(const SolutionHandle<T>& other) const {
    return *sol < *other.sol;
}

template<typename T>
bool SolutionHandle<T>::operator==(const SolutionHandle<T>& other) const {
    return *sol == *other.sol;
}

template<typename T>
void SolutionHandle<T>::removeSolution(Solution<T>* sol) {
        Solution<T>* parentSolution {sol->prevSolution};
        delete sol;

        if (parentSolution == nullptr)
            return;

        parentSolution->removeChildren();
        if (parentSolution->isTerminal())
            removeSolution(parentSolution);
        
}

template<typename T>
void SolutionHandle<T>::backtractPathR(std::vector<Solution<T>*>& v, Solution<T>* curSol) {
        if (curSol->prevSolution != nullptr)
            backtractPathR(v, curSol->prevSolution);
        v.push_back(curSol);
}