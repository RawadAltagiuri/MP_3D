#include "mex.h"
#include "PDQquickMode.h"

//Previously Named wrapperPDQquickMode
// This program works as the translator between MATLAB and C++. It uses the priority_deque class (in the PDQHeap.h file) and the mex.h library to create a MEX file that can be used in MATLAB.
priority_deque<std::pair<std::vector<double>, std::vector<std::vector<double>>>> *pq = NULL;
// The gateway function
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    // Check for proper number of arguments
    // if(nrhs < 1) {
    //     mexErrMsgIdAndTxt("PriorityDeque:nrhs", "At least one input required.");
    // }

    // Command string
    
    char *command = mxArrayToString(prhs[0]);

    // Initialize, e.g: pq_ptr = priority_deque_mex('init')
    if(strcmp(command, "init") == 0) { //strcmp returns 0 if the two strings are equal
        // Create a priority_deque instance and store the pointer in plhs[0]
        pq = new priority_deque<std::pair<std::vector<double>, std::vector<std::vector<double>>>>();
        plhs[0] = mxCreateNumericMatrix(1, 1, mxINDEX_CLASS, mxREAL);
        memcpy(mxGetData(plhs[0]), &pq, sizeof(pq));
        return;
    }

    // Get the priority_deque instance
    
    if(nrhs > 1) {
        memcpy(&pq, mxGetData(prhs[1]), sizeof(pq));
    }

    // Add, e.g : priority_deque_mex('add', pq_ptr, value)
    if(strcmp(command, "add") == 0) {
        if(nrhs != 3) {
            mexErrMsgIdAndTxt("PriorityDeque:nrhs", "Three inputs required.");
        }

        std::pair<std::vector<double>, std::vector<std::vector<double>>> input;
        // Accessing the first element of the cell array (double)

        mxArray *cellElement1 = mxGetCell(prhs[2], 0);
        double * GHF = mxGetPr(cellElement1);
        std::vector<double> value;
        mwSize numColsFirst = mxGetN(cellElement1);
        for (mwSize i = 0; i < numColsFirst; ++i) {
            value.push_back(GHF[i]);
        }
        input.first = value;
        
        // // // Accessing the second element of the cell array (vector<vector<double>>)
        mxArray *matrix = mxGetCell(prhs[2], 1);

        // // Get the number of elements in the second element cell array


        // // Iterate over the second element cell array
        double *matrixData = mxGetPr(matrix);
        mwSize numRows = mxGetM(matrix);
        mwSize numColsSecond = mxGetN(matrix);
        std::vector<std::vector<double>> inputMat;
        for (mwSize i = 0; i < numRows; ++i) {
            std::vector<double> row;
            for (mwSize j = 0; j < numColsSecond; ++j) {
                row.push_back(matrixData[i + j * numRows]);
            }
            inputMat.push_back(row);
        }
        input.second = inputMat;
            // input.second = pair;
        pq->add(input);
        return;
    }

    // setMaxSize e.g : priority_deque_mex('setMaxSize', pq_ptr, value)
    if(strcmp(command, "setMaxSize") == 0) {
        if(nrhs != 3) {
            mexErrMsgIdAndTxt("PriorityDeque:nrhs", "Three inputs required.");
        }
        double value = mxGetScalar(prhs[2]);
        pq->setMaxSize((int) value);
        return;
    }

    // Poll e.g : priority_deque_mex('poll', pq_ptr) gpt
    if (strcmp(command, "poll") == 0) {
       if (nlhs != 2) {
            mexErrMsgIdAndTxt("PriorityDeque:nlhs", "Two outputs required.");
        }

        // Get the pair returned by the priority deque
        std::pair<std::vector<double>, std::vector<std::vector<double>>> result = pq->poll();
        std::vector<double> firstElement = result.first;
        std::vector<std::vector<double>> secondElement = result.second;

        // Return the first element as a MATLAB vector of doubles
        mwSize firstSize = firstElement.size();
        mxArray* firstOutput = mxCreateDoubleMatrix(1, firstSize, mxREAL);
        double* firstOutputData = mxGetPr(firstOutput);
        for (mwSize i = 0; i < firstSize; ++i) {
            firstOutputData[i] = firstElement[i];
        }
        plhs[0] = firstOutput;

        // Return the second element as a MATLAB matrix
        mwSize numRows = secondElement.size();
        mwSize numCols = (numRows > 0) ? secondElement[0].size() : 0;
        mxArray* secondOutput = mxCreateDoubleMatrix(numRows, numCols, mxREAL);
        double* secondOutputData = mxGetPr(secondOutput);
        for (mwSize j = 0; j < numCols; ++j) {
            for (mwSize i = 0; i < numRows; ++i) {
                secondOutputData[i + j * numRows] = secondElement[i][j];
            }
        }
        plhs[1] = secondOutput;

        return;
    }


    // Poll e.g : priority_deque_mex('poll', pq_ptr) gpt
    if (strcmp(command, "peek") == 0) {
        if (nlhs != 2) {
            mexErrMsgIdAndTxt("PriorityDeque:nlhs", "Two outputs required.");
        }

        // Get the pair returned by the priority deque
        std::pair<std::vector<double>, std::vector<std::vector<double>>> result = pq->peek();
        std::vector<double> firstElement = result.first;
        std::vector<std::vector<double>> secondElement = result.second;

        // Return the first element as a MATLAB vector of doubles
        mwSize firstSize = firstElement.size();
        mxArray* firstOutput = mxCreateDoubleMatrix(1, firstSize, mxREAL);
        double* firstOutputData = mxGetPr(firstOutput);
        for (mwSize i = 0; i < firstSize; ++i) {
            firstOutputData[i] = firstElement[i];
        }
        plhs[0] = firstOutput;

        // Return the second element as a MATLAB matrix
        mwSize numRows = secondElement.size();
        mwSize numCols = (numRows > 0) ? secondElement[0].size() : 0;
        mxArray* secondOutput = mxCreateDoubleMatrix(numRows, numCols, mxREAL);
        double* secondOutputData = mxGetPr(secondOutput);
        for (mwSize j = 0; j < numCols; ++j) {
            for (mwSize i = 0; i < numRows; ++i) {
                secondOutputData[i + j * numRows] = secondElement[i][j];
            }
        }
        plhs[1] = secondOutput;

        return;
    }



    // Size e.g : priority_deque_mex('size', pq_ptr)
    if(strcmp(command, "size") == 0) {
        if(nlhs != 1) {
            mexErrMsgIdAndTxt("PriorityDeque:nlhs", "One output required.");
        }
        plhs[0] = mxCreateDoubleScalar(pq->size());
        return;
    }

    // Empty e.g : priority_deque_mex('empty', pq_ptr)
    if(strcmp(command, "empty") == 0) {
        if(nlhs != 1) {
            mexErrMsgIdAndTxt("PriorityDeque:nlhs", "One output required.");
        }
        plhs[0] = mxCreateLogicalScalar(pq->empty());
        return;
    }



    // Clear e.g : priority_deque_mex('clear', pq_ptr)
    if(strcmp(command, "clear") == 0) {
        pq->clear();
        return;
    }

    // Deallocate e.g : priority_deque_mex('delete', pq_ptr)
    if(strcmp(command, "delete") == 0) {
        delete pq;
        return;
    }
}
