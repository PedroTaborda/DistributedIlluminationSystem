

#include <iostream>

#include "consensus.hpp"



int main()
{
    ConsensusSolver solver1, solver2;

    int nNodes = 2;
    double newCost[2] = {1.0, 2.0};
    double gainsMeToOthers1[2] = {10.0, 3};
    double gainsMeToOthers2[2] = {2, 7.0};
    solver1.start(nNodes, 0, newCost, gainsMeToOthers1);
    solver2.start(nNodes, 1, newCost, gainsMeToOthers2);

    for (int i = 0; i < 2; i++) {
        double *sol1 = solver1.optimumSolution();
        double *sol2 = solver2.optimumSolution();

        printf("Di1[%d]: %f, %f\n", i, sol1[0], sol1[1]);
        printf("Di2[%d]: %f, %f\n", i, sol2[0], sol2[1]);

        solver1.updateDiMean(sol1);
        solver1.updateDiMean(sol2);
        solver2.updateDiMean(sol1);
        solver2.updateDiMean(sol2);

        printf("DiMean 1: %f, %f\n", solver1.diMean[0], solver1.diMean[1]);
        printf("DiMean 2: %f, %f\n", solver2.diMean[0], solver2.diMean[1]);

        solver1.finishIter();
        solver2.finishIter();
    }
}
