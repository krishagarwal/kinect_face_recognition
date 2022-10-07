#include "BAProblem.h"
#include "assert.h"
#include <Eigen/Dense>
#include <iostream>
#include <stdlib.h>
#include <vector>

using namespace Eigen;
using namespace std;

bool sameVec(vector<double>& a, vector<double>& b){
    for(int i = 0; i < 7; i++){
        if(abs(a[i] - b[i]) > 1e-10){
            return false;
        }
    }
    return true;
}

void BAProblemSanityTests()
{
    // Verify utils are correct inverses
    auto i4 = Matrix4d::Identity();
    auto rhs = BAProblem::rTvecToMat(BAProblem::rTMatToVec(i4));
    if (i4 != rhs) {
        cout << rhs << endl;
        cout << i4 << endl;
        assert(i4 == rhs);
    }

    vector<double> v7;
    vector<double> v7neg;
    for (int i = 0; i < 100; i++) {
        auto q = Quaterniond::UnitRandom();
        double x = (double)rand() / RAND_MAX;
        double y = (double)rand() / RAND_MAX;
        double z = (double)rand() / RAND_MAX;
        v7 = vector<double> { q.x(), q.y(), q.z(), q.w(), x, y, z };
        v7neg = vector<double> { -q.x(), -q.y(), -q.z(), -q.w(), x, y, z };
        auto rhs = BAProblem::rTMatToVec(BAProblem::rTvecToMat(v7));
        if (!sameVec(v7, rhs) && !sameVec(v7neg, rhs)) {
            cout << "Failed test: " << i << endl;
            for (int j = 0; j < 7; j++) {
                cout << v7[j] << " ";
            }
            cout << endl;
            for (int j = 0; j < 7; j++) {
                cout << v7neg[j] << " ";
            }
            cout << endl;
            for (int j = 0; j < 7; j++) {
                cout << rhs[j] << " ";
            }
            cout << endl;
            assert(v7 == rhs);
        }
    }

    // Verify identity -> identity
    v7 = vector<double> { 0, 0, 0, 1, 0, 0, 0 };
    assert(i4 == BAProblem::rTvecToMat(v7));
}

int main()
{
#if (NDEBUG == 1)
    cout << "Assertions disabled" << endl;
#endif
    cout << "Testing: " << endl;
    BAProblemSanityTests();
    cout << "Passed" << endl;
    return 0;
}