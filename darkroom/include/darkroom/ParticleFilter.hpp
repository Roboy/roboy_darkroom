#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <random>
#include <numeric>
#include <iostream>

using namespace std;
using namespace Eigen;

template <typename state>
struct ParticleFilter{
    ParticleFilter(int number_of_particles, int states, function<void (int id)> process, function<double (int id)> cost,
                   VectorXd &start, double std):
            number_of_particles(number_of_particles), process(process), cost(cost), start(start){
        particles.resize(number_of_particles, VectorXd(states));
        weights.resize(number_of_particles, 1.0/number_of_particles);
        costs.resize(number_of_particles,RAND_MAX);
        cumsum.resize(number_of_particles);
        indx.resize(number_of_particles);

        distribution = normal_distribution<double>(0,std);
        state_length = particles.begin()->size();

        resample(true);
    }

    void step(){
        double sum_of_elems = 0.0;
        for(uint i=0;i<number_of_particles;i++){
            process(i);
            costs[i] = cost(i);
            sum_of_elems += costs[i];
        }

        double Neff = 0.0, weight_sum = 0.0;
        for(uint i=0;i<number_of_particles;i++){
            weights[i] = costs[i]/sum_of_elems;
            weight_sum += weights[i];
            Neff += pow(weights[i],2.0);
        }

        Neff = 1.0/Neff;

        if(Neff < number_of_particles/2.0) {
            cout <<"resample, Neff: " << Neff << endl;
            resample(false, Neff);
        }
    }

    double result(state &winner, int *index){
        double val;
        minimum<double>(costs, val, index);
        winner = particles[*index];
        return val;
    }

    function<double (int id)> cost;
    function<void (int id)> process;

    void resample(bool all, int N = 0){
        if(all){ //resample all
            for(state &particle:particles){
                particle = start;
                for(int i=0;i<state_length;i++){
                    particle(i) += distribution(generator);
                }
            }
        }else{ // Sampling Importance Resampling
            vector<size_t> indexes = sort_indexes<double>(weights);
            for(uint i=0;i<N;i++){
                particles[indexes[i]] = particles[indexes[number_of_particles-1-i]];
                double w0 = weights[indexes[i]], w1 = indexes[number_of_particles-1-i];

            }

        }
    }

    template<typename T>
    void minimum(vector<T> &v, T &val, int *index ){
        *index = 0;
        val = v[0];
        for(uint i=1; i<v.size(); i++){
            if(v[i]<val){
                val = v[i];
                *index = i;
            }
        }
    }

    template <typename T>
    vector<size_t> sort_indexes(const vector<T> &v) {

        // initialize original index locations
        vector<size_t> idx(v.size());
        iota(idx.begin(), idx.end(), 0);

        // sort indexes based on comparing values in v
        sort(idx.begin(), idx.end(),
             [&v](size_t i1, size_t i2) {return v[i1] > v[i2];});

        return idx;
    }

    vector<state> particles;
    vector<double> weights, costs;
    vector<double> cumsum;
    vector<int> indx;
    int number_of_particles;
    default_random_engine generator;
    normal_distribution<double> distribution;
    int state_length;
    VectorXd start;
};