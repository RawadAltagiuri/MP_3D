#include "mex.hpp"
#include "mexAdapter.hpp"
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <set>
#include <string>
#include <exception>

#include <limits>

#include <functional>

#include "SolutionHandle.h"
#include "PriorityQueue.h"

using namespace matlab::mex;
using namespace matlab::data;

using CT = std::vector<std::vector<double>>;

ArrayFactory arrFactory;

namespace Util {
    std::vector<std::vector<double>> matrixToVector(Array& matrix) {
        int numOfRows = matrix.getDimensions()[0];
        int numOfColumns = matrix.getDimensions()[1];

        std::vector<std::vector<double>> c {};
        for (int i = 0; i < numOfRows; i++) {
            std::vector<double> row {};
            for (int j = 0; j < numOfColumns; j++) {
                row.push_back(matrix[i][j]);
            }
            c.push_back(std::move(row));
        }

        return c;
    }

    TypedArray<double> vectorToMatrix(const std::vector<std::vector<double>>& v) {
        unsigned long long int rows {v.size()};
        unsigned long long columns {v[0].size()};

        TypedArray<double> matrix {arrFactory.createArray<double>({rows, columns})};
        for (int i = 0; i < rows; i++) {
            const std::vector<double>& row {v[i]}; 
            for (int j = 0; j < columns; j++) {
                matrix[i][j] = row[j];
            }
        }
        
        return matrix;
    }

    std::string vector2DtoString(const std::vector<std::vector<double>>& v) {
        std::string sRep {};
        sRep.append("[ ");
        for (int i = 0; i < v.size(); i++) {
            sRep.append("[ ");
            for (int j = 0; j < v[0].size(); j++) {
                sRep.append(std::to_string(v[i][j]));
                sRep.append(" ");   
            }
            sRep.append("], ");
        }
        sRep.append("]");

        return sRep;
    }

    Array matrixPath(const std::vector<Solution<CT>*>& path) {
        Array matlabPath {arrFactory.createCellArray({1, path.size()})};
        for (int i = 0; i < path.size(); i++) {
            Solution<CT>* curSol {path[i]};
            matlabPath[i] = Util::vectorToMatrix(curSol->config);
        }

        return matlabPath;
    }
    
    struct ArrayHash {
        size_t operator()(const Array& arr) const {
            std::hash<double> hd;
            size_t hashValue = 0;
            size_t multip = 1;
            size_t rows {arr.getDimensions()[0]};
            size_t columns {arr.getDimensions()[1]};

            for (size_t i = rows - 1; i > rows - 2; i--) {
                for (size_t j = 0; j < columns; j++) {
                    double d {arr[i][j]};
                    hashValue ^= hd(d);
                }
            }
            return hashValue;
        }
    };

    struct ArrayEquator {
        bool operator()(const Array& a1, const Array& a2) const {
            ArrayDimensions a1Dims {a1.getDimensions()};
            ArrayDimensions a2Dims {a2.getDimensions()};

            if (!(a1Dims[0] == a2Dims[0] && a1Dims[1] == a2Dims[1]))
                return false;
            else {
                for (int i = 0; i < a1Dims[0]; i++) {
                    for (int j = 0; j < a1Dims[1]; j++) {
                        double d1 {a1[i][j]};
                        double d2 {a2[i][j]};
                        if (d1 != d2)
                            return false;
                    }
                }
            }
            return true;
        }
    };

    struct VectorHash2D {
        size_t operator()(const std::vector<std::vector<double>> v) const {
            std::hash<double> hd;
            size_t totalHash = 0;

            size_t rows {v.size()};
            size_t columns {v[0].size()};

            for (size_t i = rows - 1; i > rows - 2; i--) {
                for (size_t j = 0; j < columns; j++) {
                    totalHash ^= hd(v[i][j]);
                }
            }
            return totalHash;
        }
    };

    struct VectorEquator2D {
        bool operator()(const std::vector<std::vector<double>>& v1, const std::vector<std::vector<double>>& v2) const {
            size_t rows {v1.size()};
            size_t columns {v1[0].size()};

            if (!(rows == v2.size() && columns == v2[0].size())) 
                return false;
            
            else {
                for (size_t i = 0; i < rows; i++) {
                    for (size_t j = 0; j < columns; j++) {
                        if (v1[i][j] != v2[i][j])
                            return false;
                    }
                }
            }

            return true;
        }
    };
}


class MexFunction : public matlab::mex::Function {
    std::ostringstream stream;
    void displayOnMATLAB(std::ostringstream& stream) {
        std::shared_ptr<matlab::engine::MATLABEngine> matlabPtr = getEngine();
        // Pass stream content to MATLAB fprintf function
        matlabPtr->feval(u"fprintf", 0,
            std::vector<Array>({ arrFactory.createScalar(stream.str()) }));
        // Clear stream buffer
        stream.str("");
    }

    private:
        std::unordered_map<int, PriorityQueue<SolutionHandle<CT>>> activeHeaps {};
        std::unordered_map<int, std::unordered_set<CT, Util::VectorHash2D, Util::VectorEquator2D>> activeSets {};
        
        unsigned int instanceId;
        PriorityQueue<unsigned int> availables {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    public:
        void operator()(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs) override {
            parseAction(outputs, inputs);
        }

        void destroyAll() {
            for (auto instanceIt : activeHeaps) {
                destroy(instanceIt.first); 
            }

            activeHeaps.clear();
            activeSets.clear();
        }

        void destroy(unsigned int id) {
            auto& heap {activeHeaps[id]};
            auto& duplicateSet {activeSets[id]};

            while (!heap.empty()) {
                heap.poll().destroyHandle();
            }       
            duplicateSet.clear();

            activeHeaps.erase(id);
            activeSets.erase(id);       

            availables.insert(id);
        }

        ~MexFunction() {
            destroyAll();
        }
        

        void parseAction(matlab::mex::ArgumentList& outputs, matlab::mex::ArgumentList& inputs) {
            TypedArray<MATLABString> actions {std::move(inputs[0])};
            const std::string& action {actions[0]}; 
            if (action == "init") {
                init(outputs, inputs);
            } 
            else if (action == "peek") {
                peek(outputs, inputs);
            }
            else if (action == "insertAny") {
                insertAny(outputs, inputs);
            }
            else if (action == "insertPath") {
                insertPath(outputs, inputs);
            }
            else if (action == "addInvalidConfig") {
                addInvalidConfig(outputs, inputs);
            }
            else if (action == "expandHead") {
                expandHead(outputs, inputs);
            }
            else if (action == "size") {
                size(outputs, inputs);
            }
            else if (action == "inner") {
                inner(outputs, inputs);
            }
            else if (action == "extractHead") {
                extractHead(outputs, inputs);
            }
            else if (action == "destroy") {
                destroy(outputs, inputs);
            }
            else if (action == "destroyAll") {
                destroyAll(outputs, inputs);
            }
            else {
                throw std::exception {};
            }
        }

        void init(matlab::mex::ArgumentList& outputs, matlab::mex::ArgumentList& inputs) {
            if (availables.size() == 0)
                throw std::exception {};

            instanceId = availables.poll();

            double d = inputs[1][0];

            activeHeaps.insert({instanceId, {}});
            activeHeaps[instanceId].setMaxSize(inputs[1][0]);

            activeSets.insert({instanceId, {}});
            outputs[0] = arrFactory.createScalar(instanceId);
        }

        void peek(matlab::mex::ArgumentList& outputs, matlab::mex::ArgumentList& inputs) {
            instanceId = inputs[1][0];
            auto& heap {activeHeaps[instanceId]};
            auto& duplicateSet {activeSets[instanceId]};

            if (heap.empty()) {
                outputs[0] = arrFactory.createEmptyArray();
            }
            else {
                Solution<CT>* headSolution {heap.peek().sol};

                outputs[0] = Util::vectorToMatrix(headSolution->config);
                Array costs {arrFactory.createArray<double>({1, 3})};
                costs[0] = headSolution->cost.trueCost;
                costs[1] = headSolution->cost.heuristicCost;
                costs[2] = headSolution->cost.totalCost;
                outputs[1] = costs;
            }
        }

        void insertAny(matlab::mex::ArgumentList& outputs, matlab::mex::ArgumentList& inputs) {
            instanceId = inputs[1][0];
            auto& heap {activeHeaps[instanceId]};
            auto& duplicateSet {activeSets[instanceId]};

            heap.insert({new Solution<CT>{nullptr, {inputs[3][0], inputs[3][1], inputs[3][2]}, Util::matrixToVector(inputs[2])}});
        }

        void insertPath(matlab::mex::ArgumentList& outputs, matlab::mex::ArgumentList& inputs) {
            instanceId = inputs[1][0];
            auto& heap {activeHeaps[instanceId]};
            auto& duplicateSet {activeSets[instanceId]};

            Array path {std::move(inputs[2])};
            Array costs {std::move(inputs[3])};

            Array config = std::move(path[0]);
            std::vector<std::vector<double>> configVector {Util::matrixToVector(config)};
            Solution<CT>* prevSolution = new Solution<CT>{nullptr, {costs[0], costs[1], costs[2]}, configVector};
            //duplicateSet.insert(configVector);
            for (int i = 1; i < path.getDimensions()[0]; i++) {
                config = std::move(path[i]);
                configVector = Util::matrixToVector(config);
                

                prevSolution->addChildren();
                Solution<CT>* curSol {new Solution<CT>{prevSolution, {costs[0], costs[1], costs[2]}, configVector}};

                prevSolution = curSol;
            }

            heap.insert({prevSolution});
            //duplicateSet.insert(configVector);
        }

        void addInvalidConfig(matlab::mex::ArgumentList& outputs, matlab::mex::ArgumentList& inputs) {
            instanceId = inputs[1][0];
            auto& heap {activeHeaps[instanceId]};
            auto& duplicateSet {activeSets[instanceId]};

            auto config {std::move(inputs[2])};
            //duplicateSet.insert({Util::matrixToVector(config)});
        }

        void expandHead(matlab::mex::ArgumentList& outputs, matlab::mex::ArgumentList& inputs) {
            instanceId = inputs[1][0];
            auto& heap {activeHeaps[instanceId]};
            auto& duplicateSet {activeSets[instanceId]};

            Array children {std::move(inputs[2])};

            SolutionHandle<CT> headSolution {heap.poll()};
            auto numOfChildren {children.getDimensions()[0]};
            if (numOfChildren == 0) {
                headSolution.destroyHandle();
                return;
            }
            

            for (int i = 0; i < numOfChildren; i++) {
                Array child = std::move(children[i]);
                
                Array childConfig = std::move(child[0]);
                Array childCost = std::move(child[1]);

                auto cv {Util::matrixToVector(childConfig)};
                //if (duplicateSet.find({cv}) == duplicateSet.end()) {
                    SolutionHandle<CT> solHandle {new Solution<CT>(headSolution.sol, {childCost[0], childCost[1], childCost[2]}, cv)};
                    heap.insert(solHandle);
                    headSolution.sol->addChildren();
                    //duplicateSet.insert({cv});
                //}
            }

            
            if (headSolution.sol->isTerminal()) {
                headSolution.destroyHandle();
                return;
            }
            
        }

        void size(matlab::mex::ArgumentList& outputs, matlab::mex::ArgumentList& inputs) {
            instanceId = inputs[1][0];
            auto& heap {activeHeaps[instanceId]};
            auto& duplicateSet {activeSets[instanceId]};

            outputs[0] = arrFactory.createScalar(heap.size());
        }

        void inner(matlab::mex::ArgumentList& outputs, matlab::mex::ArgumentList& inputs) {
            instanceId = inputs[1][0];
            auto& heap {activeHeaps[instanceId]};
            auto& duplicateSet {activeSets[instanceId]};

            const auto& solHandles {heap.inner()};
            CellArray arr {arrFactory.createCellArray({1, solHandles.size()})};
            for (int i = 0; i < solHandles.size(); i++) {
                arr[i] = Util::vectorToMatrix(solHandles[i].sol->config);
            }

            outputs[0] = arr;
        }


        void extractHead(matlab::mex::ArgumentList& outputs, matlab::mex::ArgumentList& inputs) {
            instanceId = inputs[1][0];
            auto& heap {activeHeaps[instanceId]};
            auto& duplicateSet {activeSets[instanceId]};

            SolutionHandle<CT> headHandle {heap.peek()};
            std::vector<Solution<CT>*> solPath {headHandle.backtractPath()};

            TypedArray<double> costMatrix {arrFactory.createArray<double>({1, 3})};
            costMatrix[0] = (solPath[solPath.size() - 1])->cost.trueCost;
            costMatrix[1] = (solPath[solPath.size() - 1])->cost.heuristicCost;
            costMatrix[2] = (solPath[solPath.size() - 1])->cost.totalCost;
            
            outputs[0] = Util::matrixPath(solPath);
            outputs[1] = std::move(costMatrix);
        }

        void destroy(matlab::mex::ArgumentList& outputs, matlab::mex::ArgumentList& inputs) {
            instanceId = inputs[1][0];
            if (activeHeaps.find(instanceId) == activeHeaps.end()) {
                return;
            }
            destroy(instanceId);
        }      

        void destroyAll(matlab::mex::ArgumentList& outputs, matlab::mex::ArgumentList& inputs) {
            destroyAll();
        }
};


