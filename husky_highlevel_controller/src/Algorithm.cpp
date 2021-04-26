#include "husky_highlevel_controller/Algorithm.hpp"

#include <utility>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/count.hpp>

namespace listener_controller {

using namespace boost::accumulators;

struct Algorithm::Data {
  accumulator_set<double, features<tag::mean, tag::count>> acc;
};

Algorithm::Algorithm() {
  data_ = std::make_unique<Data>();
}

Algorithm::~Algorithm() = default;

void Algorithm::addData(const double data)
{
  data_->acc(data);
}

void Algorithm::addData(const Eigen::VectorXd& data)
{
  for(auto i = 0; i < data.size(); ++i)
    addData(data[i]);
}

double Algorithm::getMin() const
{
  return count(data_->acc) ? min(data_->acc) : 0;
}

} /* namespace */