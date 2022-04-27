
#include <stdio.h>
#include "consensus.hpp"

#ifdef CONSENSUS_TESTER
void consensusSolverEx2();
void consensusSolverEx3();

int main()
{
    consensusSolverEx2();
    // consensusSolverEx3();
    return 0;
}
#endif

void consensusSolverEx2()
{
    ConsensusSolver solver1, solver2, solver3;

    int nNodes = 2;
    double localCost1 = 100.0;
    double localCost2 = 1.0;
    //    double gainsOthersToMe1[2] = {40.0, 8.0};
    //    double gainsOthersToMe2[2] = {6.0, 37.0};
    double gainsOthersToMe1[2] = {86.05, 51.32};
    double gainsOthersToMe2[2] = {17.12, 71.53};

    solver1.li = 40.0;
    solver2.li = 0.0;

    solver1.rho = 7.0;
    solver2.rho = 7.0;

    double oi1 = 1.1385;
    double oi2 = 0.5116;

    solver1.start(nNodes, 0, localCost1, gainsOthersToMe1, oi1);
    solver2.start(nNodes, 1, localCost2, gainsOthersToMe2, oi2);

    for (int i = 0; i < 22; i++)
    {
        //printf("Iteration %d\n", i);
        double *sol1 = solver1.optimumSolution();
        double *sol2 = solver2.optimumSolution();

        //printf("Di1[%d]: %f, %f\n", i, sol1[0], sol1[1]);
        //printf("Di2[%d]: %f, %f\n", i, sol2[0], sol2[1]);
        printf("[%d]: %f, %f", i, sol1[0], sol1[1]);
        printf(", %f, %f\n", sol2[0], sol2[1]);

        solver1.updateDiMean(sol1);
        solver1.updateDiMean(sol2);

        solver2.updateDiMean(sol1);
        solver2.updateDiMean(sol2);

        // printf("DiMean 1: %f, %f\n", solver1.diMean[0], solver1.diMean[1]);
        // printf("DiMean 2: %f, %f\n", solver2.diMean[0], solver2.diMean[1]);

        solver1.finishIter();
        solver2.finishIter();
    }
}

void consensusSolverEx3()
{
    ConsensusSolver solver1, solver2, solver3;

    int nNodes = 3;
    double localCost1 = 1;
    double localCost2 = 1;
    double localCost3 = 1;
    double gainsOthersToMe1[3] = {200.0, 50.0, 50.0};
    double gainsOthersToMe2[3] = {50.0, 200.0, 50.0};
    double gainsOthersToMe3[3] = {50.0, 50.0, 200.0};
    solver1.li = 100;
    solver1.rho = 7.0;

    solver2.li = 0;
    solver2.rho = 7.0;

    solver3.li = 0;
    solver3.rho = 7.0;

    solver1.start(nNodes, 0, localCost1, gainsOthersToMe1, 20);
    solver2.start(nNodes, 1, localCost2, gainsOthersToMe2, 30);
    solver3.start(nNodes, 2, localCost3, gainsOthersToMe3, 40);

    for (int i = 0; i < 21; i++)
    {
        printf("Iteration %d\n", i);
        double *sol1 = solver1.optimumSolution();
        double *sol2 = solver2.optimumSolution();
        double *sol3 = solver3.optimumSolution();

        printf("Di1[%d]: %f, %f, %f\n", i, sol1[0], sol1[1], sol1[2]);
        printf("Di2[%d]: %f, %f, %f\n", i, sol2[0], sol2[1], sol2[2]);
        printf("Di3[%d]: %f, %f, %f\n", i, sol3[0], sol3[1], sol3[2]);

        solver1.updateDiMean(sol1);
        solver1.updateDiMean(sol2);
        solver1.updateDiMean(sol3);

        solver2.updateDiMean(sol1);
        solver2.updateDiMean(sol2);
        solver2.updateDiMean(sol3);

        solver3.updateDiMean(sol1);
        solver3.updateDiMean(sol2);
        solver3.updateDiMean(sol3);

        // printf("DiMean 1: %f, %f\n", solver1.diMean[0], solver1.diMean[1]);
        // printf("DiMean 2: %f, %f\n", solver2.diMean[0], solver2.diMean[1]);

        solver1.finishIter();
        solver2.finishIter();
        solver3.finishIter();
    }
}
