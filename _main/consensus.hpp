#ifndef CONSENSUS_HPP
#define CONSENSUS_HPP

#include "math_utils.hpp"

inline constexpr unsigned int MAX_DEVICES = 16; // duplicated in calibration.hpp

constexpr double maxDutyCycle = 1.0;
constexpr double minDutyCycle = 0.0;

enum ConsensusState
{
    CONSENSUS_STATE_NOT_STARTED,
    CONSENSUS_STATE_COMPUTING_LOCAL,
    CONSENSUS_STATE_WAITING_FOR_NEIGHBORS,
    CONSENSUS_STATE_PREPARE_NEXT_ITERATION
};

class ConsensusSolver {
public:
    void setState(ConsensusState state) {
        this->state = state;
    }
    void start(int nNodes, int myI, double *newCost, double *gainsMeToAll){
        setState(CONSENSUS_STATE_COMPUTING_LOCAL);
        this->I = myI;
        this->nNodes = nNodes;
        this->state = CONSENSUS_STATE_COMPUTING_LOCAL;
        for (int i = 0; i < nNodes; i++) {
            this->cost[i] = newCost[i];
            this->ki[i] = gainsMeToAll[i];
        }
        this->iteration = 0;

        initLagrangeMultipliers();
        resetDiMean();
    }

    double *optimumSolution()
    {
        computezi();

        static double *sol = argming();

        printf("Di--[%d]: %f, %f\n", I, sol[0], sol[1]);
        printf("isFeasible: %d\n", isFeasible(sol));

        if (isFeasible(sol))
            return sol;

        static double *S[5] = {
            diS1(),
            diS2(),
            diS3(),
            diS4(),
            diS5()};

        double costs[5] = {
            fPlus(S[0]),
            fPlus(S[1]),
            fPlus(S[2]),
            fPlus(S[3]),
            fPlus(S[4])};

        unsigned int best = argmin(5, costs);

        for (int i = 0; i < nNodes; i++) {
            di[i] = S[best][i];
        }

        return S[best];
    }

//private:
    ConsensusState state = CONSENSUS_STATE_NOT_STARTED;
    unsigned int nNodes = 1;
    int I = 0; // index of this node
    int iteration = 0;

    double li = 1.0;
    double oi = 1.0;
    double localCost = 1.0;
    double ki[MAX_DEVICES]; // staticGains (calibration.hpp)
    double rho = 1.0;

    double lagrangeMultipliers[MAX_DEVICES];
    double currentSolution[MAX_DEVICES];

    double di[MAX_DEVICES];

    double diMean[MAX_DEVICES];
    double diCount = 0; // stored as double to avoid integer division - valid up to values much larger than MAX_DEVICES

    double zi[MAX_DEVICES];

    double cost[MAX_DEVICES];

    void finishIter(){
        computeNextLagrangeMultipliers();
        diCount = 0;
        iteration++;
    }
    double fPlus(double *d){
        return isFeasible(d) ? costFunction(d) : __DBL_MAX__;
    }
    double costFunction(double *d){
        return dot(nNodes, d, cost);
    }
    double *argming(){
        // g(di) = 0.5 rho di'*di - di'*zi (quadratic cost)
        static double diGlobalMin[MAX_DEVICES];
        for (int i = 0; i < nNodes; i++) {
            diGlobalMin[i] = zi[i]/rho;
        }
        return diGlobalMin;
    }
    double *diS1(){
        static double diS1[MAX_DEVICES];
        for (int i = 0; i < nNodes; i++) {
            diS1[i] = zi[i]/rho - ki[i] * (oi-li+ dot(nNodes, ki, zi)/rho) / dot(nNodes, ki, ki);
        }
        return diS1;
    }
    double *diS2(){
        static double diS2[MAX_DEVICES];
        for (int i = 0; i < nNodes; i++) {
            diS2[i] = (i==I ? 0.0 : zi[i]/rho ) ;
        }
        return diS2;
    }
    double *diS3(){
        static double diS3[MAX_DEVICES];
        for (int i = 0; i < nNodes; i++) {
            diS3[i] = (i == I ? 1.0 : zi[i] / rho);
        }
        return diS3;
    }
    double *diS4(){
        static double diS4[MAX_DEVICES];
        double alpha = 0.0;
        for (int i = 0; i < nNodes; i++) {
            if (i == I) {
                diS4[i] = 0.0;
            } else {
                alpha = ki[i]/(dot(nNodes, ki, ki) - ki[I] * ki[I]);
                diS4[i] = zi[i] / rho - alpha * (oi - li - (dot(nNodes, ki, zi) + ki[I]*zi[I]) / rho);
            }
        }
        return diS4;
    }
    double *diS5(){
        static double diS5[MAX_DEVICES];
        double alpha = 0.0;
        for (int i = 0; i < nNodes; i++) {
            if (i == I) {
                diS5[i] = 1.0;
            } else {
                alpha = ki[i] / (dot(nNodes, ki, ki) - ki[I] * ki[I]);
                diS5[i] = zi[i] / rho - alpha * (oi - li + ki[I] - (dot(nNodes, ki, zi) + ki[I] * zi[I]) / rho);
            }
        }
        return diS5;
    }
    void computezi(){
        for (int i = 0; i < nNodes; i++) {
            zi[i] = rho * diMean[i] - lagrangeMultipliers[i] - (I == i ? 0.0 : cost[i]);
            printf("zi[%d]: %f\n", i, zi[i]);
        }
    }
    void computeNextLagrangeMultipliers(){
        for(int i = 0; i < nNodes; i++){
            lagrangeMultipliers[i] += rho * (di[i] - diMean[i]);
        }
    }
    void initLagrangeMultipliers(){
        for(int i = 0; i < nNodes; i++){
            lagrangeMultipliers[i] = 1.0;
        }
    }
    void updateDiMean(double newDi[]){
        if (diCount == nNodes) {
            setState(CONSENSUS_STATE_PREPARE_NEXT_ITERATION);
            return;
        }
        for(int i = 0; i < nNodes; i++){
            diMean[i] = (diCount * diMean[i] + newDi[i]) / (diCount + 1.0);
        }
        diCount++;
    }
    void resetDiMean(){
        for(int i = 0; i < nNodes; i++){
            diMean[i] = 0.0;
        }
        diCount = 0;
    }
    bool isFeasible(double diCandidate[])
    {
        bool feasible = diCandidate[I] >= minDutyCycle;
        feasible &= diCandidate[I] <= maxDutyCycle;
        feasible &= -dot(nNodes, ki, diCandidate) <= oi - li;
        return feasible;
    }
};

#endif //CONSENSUS_HPP
