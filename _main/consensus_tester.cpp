
#include <stdio.h>
#include "consensus.hpp"



#ifndef DEBUG
int main()
#else
int not_actually_main()
#endif
{
    ConsensusSolver solver1, solver2;

    int nNodes = 2;
    double localCost1 = 1;
    double localCost2 = 3;
    double gainsOthersToMe1[2] = {200.0, 50.0};
    double gainsOthersToMe2[2] = {50.0, 200.0};
    solver1.li = 80;
    solver1.oi = 50;
    solver1.rho = 7.0;

    solver2.li = 270;
    solver2.oi = 50;
    solver2.rho = 7.0;

    solver1.start(nNodes, 0, localCost1, gainsOthersToMe1, 50);
    solver2.start(nNodes, 1, localCost2, gainsOthersToMe2, 50);

    for (int i = 0; i < 15; i++) {
        printf("Iteration %d\n", i);
        double *sol1 = solver1.optimumSolution();
        double *sol2 = solver2.optimumSolution();

        printf("Di1[%d]: %f, %f\n", i, sol1[0], sol1[1]);
        printf("Di2[%d]: %f, %f\n", i, sol2[0], sol2[1]);

        solver1.updateDiMean(sol1);
        solver1.updateDiMean(sol2);

        solver2.updateDiMean(sol1);
        solver2.updateDiMean(sol2);

        //printf("DiMean 1: %f, %f\n", solver1.diMean[0], solver1.diMean[1]);
        //printf("DiMean 2: %f, %f\n", solver2.diMean[0], solver2.diMean[1]);

        solver1.finishIter();
        solver2.finishIter();
    }

    return 0;
}
