#ifndef CONSENSUS_HPP
#define CONSENSUS_HPP

#include <Wire.h>

#include "comms.hpp"
#include "globals.hpp"
#include "math_utils.hpp"
#include "network.hpp"

constexpr double TOL = 1e-4;

constexpr double maxDutyCycle = 1.0;
constexpr double minDutyCycle = 0.0;

enum ConsensusState
{
    CONSENSUS_STATE_NOT_STARTED,
    CONSENSUS_STATE_COMPUTING_LOCAL,
    CONSENSUS_STATE_WAITING_FOR_NEIGHBORS,
    CONSENSUS_STATE_WAITING_CONSENSUS
};

class ConsensusSolver {
public:
    void setState(ConsensusState state) {
        this->state = state;
    }
    void start(unsigned int nNodes, int myI, double newLocalCost, double *gainsMeToAll, double externalIlluminance){
        setState(CONSENSUS_STATE_NOT_STARTED);
        this->I = myI;
        this->nNodes = nNodes;
        for (unsigned int i = 0; i < nNodes; i++) {
            this->localCost = newLocalCost;
            this->ki[i] = gainsMeToAll[i];
        }
        this->iteration = 0;
        this->oi = externalIlluminance;

        initLagrangeMultipliers();
        resetDiMean();
    }

    bool active() {
        return state != CONSENSUS_STATE_NOT_STARTED;
    }

    void setIlluminanceReference(double lumminanceReference) {
        if(li != lumminanceReference) {
            li = lumminanceReference;
            setState(CONSENSUS_STATE_COMPUTING_LOCAL);

            for(uint8_t i = 0; i < network.getNumberNodesNetwork(); i++)
            {
                if(network.getNetwork()[i] == myID) continue;
                signed char address = addr_offset + network.getNetwork()[i];
                int ret;
                SEND_MSG(address, RETRY_TIMEOUT_MS,
                    Wire.write(MSG_TYPE_CONSENSUS_START);,
                ret)
            }

            resetDiMean();
            resetDi();
            resetIteration();
            initLagrangeMultipliers();
        }
    }

    void setLocalCost(double cost) {
        localCost = cost;
    }

    double *optimumSolution()
    {
        setState(CONSENSUS_STATE_WAITING_FOR_NEIGHBORS);
        computezi();

        sol = argming();

        if (isFeasible(sol)){
            for (unsigned int i = 0; i < nNodes; i++)
            {
                di[i] = sol[i];
            }
            //printf("Best: g\n");
            return sol;
        }

        S[0] = getdiS1();
        S[1] = getdiS2();
        S[2] = getdiS3();
        S[3] = getdiS4();
        S[4] = getdiS5();

        double costs[5] = {
            fPlus(S[0]),
            fPlus(S[1]),
            fPlus(S[2]),
            fPlus(S[3]),
            fPlus(S[4])};

        unsigned int best = argmin(5, costs);

        for (unsigned int i = 0; i < nNodes; i++) {
            di[i] = S[best][i];
        }
        //printf("Costs: %f, %f, %f, %f, %f\n", costs[0] > 100000 ? -1 : costs[0], costs[1] > 100000 ? -1 : costs[1], costs[2] > 100000 ? -1 : costs[2], costs[3] > 100000 ? -1 : costs[3], costs[4] > 100000 ? -1 : costs[4]);
        //printf("Best: %d\n", best);
        return S[best];
    }

    ConsensusState state = CONSENSUS_STATE_NOT_STARTED;
    unsigned int nNodes = 1;
    unsigned int I = 0; // index of this node
    unsigned int iteration = 0;

    double li = 1.0f;
    double oi;
    double localCost = 1.0;
    double ki[MAX_DEVICES]; // staticGains (calibration.hpp)
    double rho = 7.0;

    double lagrangeMultipliers[MAX_DEVICES];
    double currentSolution[MAX_DEVICES];

    double di[MAX_DEVICES];

    double diMean[MAX_DEVICES];
    double _diMean[MAX_DEVICES];
    double diCount = 0; // stored as double to avoid integer division - valid up to values much larger than MAX_DEVICES

    double zi[MAX_DEVICES];
    double diGlobalMin[MAX_DEVICES];

    double diS1[MAX_DEVICES];
    double diS2[MAX_DEVICES];
    double diS3[MAX_DEVICES];
    double diS4[MAX_DEVICES];
    double diS5[MAX_DEVICES];
    double *S[5];
    double *sol;

    void finishIter(){
        computeNextLagrangeMultipliers();
        diCount = 0;
        iteration++;
        DEBUG_PRINT("Iteration %d\n", iteration)
        if(iteration < 4)
            setState(CONSENSUS_STATE_COMPUTING_LOCAL);
        else {
            DEBUG_PRINT("Finished consensus.\n")
            for(uint8_t i = 0; i < nNodes; i++)
                DEBUG_PRINT("d[%hhu] = %lf\n", i, di[i])
            setState(CONSENSUS_STATE_NOT_STARTED);
        }
    }

    double fPlus(double *d){
        return isFeasible(d) ? costFunction(d) : __DBL_MAX__;
    }
    double costFunction(double *d){
        return 0.5*rho*dot(nNodes, d, d) - dot(nNodes, d, zi);
    }
    
    double *argming(){
        // g(di) = 0.5 rho di'*di - di'*zi (quadratic cost)
        for (unsigned int i = 0; i < nNodes; i++) {
            diGlobalMin[i] = zi[i]/rho;
        }
        return diGlobalMin;
    }
    
    double *getdiS1(){
        for (unsigned int i = 0; i < nNodes; i++) {
            diS1[i] = zi[i]/rho - ki[i] * (oi-li+ dot(nNodes, ki, zi)/rho) / dot(nNodes, ki, ki);
        }
        return diS1;
    }
    
    double *getdiS2()
    {
        for (unsigned int i = 0; i < nNodes; i++) {
            diS2[i] = (i==I ? 0.0 : zi[i]/rho ) ;
        }
        return diS2;
    }
    
    double *getdiS3()
    {
        for (unsigned int i = 0; i < nNodes; i++) {
            diS3[i] = (i == I ? 1.0 : zi[i] / rho);
        }
        return diS3;
    }
    
    double *getdiS4()
    {
        double alpha = 0.0;
        for (unsigned int i = 0; i < nNodes; i++) {
            if (i == I) {
                diS4[i] = 0.0;
            } else {
                alpha = ki[i]/(dot(nNodes, ki, ki) - ki[I] * ki[I]);
                diS4[i] = zi[i] / rho - alpha * (oi - li + ( - dot(nNodes, ki, zi) + ki[I]*zi[I]) / rho);
            }
        }
        return diS4;
    }
    double *getdiS5()
    {
        double alpha = 0.0;
        for (unsigned int i = 0; i < nNodes; i++) {
            if (i == I) {
                diS5[i] = 1.0;
            } else {
                alpha = ki[i] / (dot(nNodes, ki, ki) - ki[I] * ki[I]);
                diS5[i] = zi[i] / rho - alpha * (oi - li + ki[I] + ( dot(nNodes, ki, zi) - ki[I] * zi[I]) / rho);
                // printf("alpha: %f\n", alpha);
                // printf("diS5[%d]: %f\n", i, diS5[i]);
                // printf("zi/rho: %f\n", zi[i] / rho);
                // printf("- dot(nNodes, ki, zi): %f\n", - dot(nNodes, ki, zi));
                // printf("( - dot(nNodes, ki, zi) + ki[I] * zi[I]): %f\n", (-dot(nNodes, ki, zi) + ki[I] * zi[I]));
                // printf("alpha * (oi - li + ki[I] + ( - dot(nNodes, ki, zi) + ki[I] * zi[I]) / rho): %f\n", alpha * (oi - li + ki[I] + (-dot(nNodes, ki, zi) + ki[I] * zi[I]) / rho));
                // printf("oi-li: %f\n", oi - li);
                // printf("ki[I]: %f\n", ki[I]);
            }
        }
        return diS5;
    }
    void computezi(){
        for (unsigned int i = 0; i < nNodes; i++) {
            zi[i] = rho * diMean[i] - lagrangeMultipliers[i] - (I == i ? localCost : 0.0);
            printf("zi[%d]/rho: %.3ff; ", i, zi[i] / rho);
            printf("zi[%d]: %.3f\n", i, zi[i]);
        }
    }
    void computeNextLagrangeMultipliers(){
        for (unsigned int i = 0; i < nNodes; i++){
            lagrangeMultipliers[i] += rho * (di[i] - diMean[i]);
        }
    }
    void initLagrangeMultipliers(){
        for (unsigned int i = 0; i < nNodes; i++){
            lagrangeMultipliers[i] = 0.0;
        }
    }
    void updateDiMean(double newDi[]){
        for (unsigned int i = 0; i < nNodes; i++){
            if (diCount == 0)
                _diMean[i] = newDi[i];
            else
                _diMean[i] += newDi[i];
        }
        diCount++;
        if (diCount == nNodes) {
            setState(CONSENSUS_STATE_WAITING_CONSENSUS);
            for (unsigned int i = 0; i < nNodes; i++)
            {
                _diMean[i] /= (double) nNodes;
                diMean[i] = _diMean[i];
            }
            return;
        }
        if(diCount > nNodes)
            DEBUG_PRINT("\n\n\n\n\n Overflowing on the average. \n\n\n\n\n")
    }
    void resetDiMean(){
        for (unsigned int i = 0; i < nNodes; i++){
            diMean[i] = 0.0;
            _diMean[i] = 0.0;
        }
        diCount = 0;
    }
    void resetDi(){
        for (unsigned int i = 0; i < nNodes; i++){
            di[i] = 0.0f;
        }
    }
    void resetIteration() {
        iteration = 0;
    }
    bool isFeasible(double diCandidate[])
    {
        bool feasible = true;
        feasible = feasible && diCandidate[I] >= minDutyCycle - TOL;
        // printf("isFeasible: %d; ", feasible);
        feasible = feasible && diCandidate[I] <= maxDutyCycle + TOL;
        // printf("isFeasible: %d; ", feasible);
        feasible = feasible && dot(nNodes, ki, diCandidate) >= li - oi - TOL;
        // printf("isFeasible: %d; ", feasible);
        // printf("Di: %f %f\n" , diCandidate[0], diCandidate[1]);
        return feasible;
    }
};

extern ConsensusSolver consensus;

#endif //CONSENSUS_HPP
