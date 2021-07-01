#include "cmaes.h"

namespace libcmaes{
    template <class TCovarianceUpdate, class TGenoPheno=GenoPheno<NoBoundStrategy>>
    class customStrategy : public CMAStrategy<TCovarianceUpdate, TGenoPheno>
    {
    public:
        customStrategy(FitFunc &func,CMAParameters<TGenoPheno> &parameters) : CMAStrategy<TCovarianceUpdate,TGenoPheno>(func, parameters)
        {};

    
        customStrategy(FitFunc &func,CMAParameters<TGenoPheno> &parameters, const CMASolutions &cmasols) : 
                        CMAStrategy<TCovarianceUpdate,TGenoPheno>(func, parameters, cmasols){};
    
        ~customStrategy(){};
    
        dMat ask(){
            return CMAStrategy<TCovarianceUpdate,TGenoPheno>::ask();
        };

        void tell(){
            CMAStrategy<TCovarianceUpdate,TGenoPheno>::tell();
        };
        bool stop(){
            return CMAStrategy<TCovarianceUpdate,TGenoPheno>::stop();
        };

        void eval(const dMat &candidates, const dMat &phenocandidates=dMat(0,0)){
            CMAStrategy<TCovarianceUpdate,TGenoPheno>::eval(candidates, phenocandidates);
        };

        void setPopulation(int popSize){
            CMAStrategy<TCovarianceUpdate,TGenoPheno>::_parameters._lambda = popSize;
            CMAStrategy<TCovarianceUpdate,TGenoPheno>::_parameters.initialize_parameters();
            CMAStrategy<TCovarianceUpdate,TGenoPheno>::_solutions._candidates.resize(popSize);
            CMAStrategy<TCovarianceUpdate,TGenoPheno>::_solutions._kcand = std::min(popSize-1,static_cast<int>(1.0+ceil(0.1+popSize/4.0)));
            std::cout << "Resetting the population size to : " << popSize << std::endl;
        };
    };
};