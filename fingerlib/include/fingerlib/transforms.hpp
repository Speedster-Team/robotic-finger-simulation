#ifndef TRANSFORMS_HPP
#define TRANSFORMS_HPP

#include "armadillo"

class Transforms
{
public:
private:
arma::vec tendons;
arma::vec motors;
arma::vec joints;
arma::mat Ra;
arma::mat S;
arma::mat J;
}
#endif

