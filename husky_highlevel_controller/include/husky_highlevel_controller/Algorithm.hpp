#pragma once

#include <memory>

#include <Eigen/Core>

namespace listener_controller {

/*!
 * Class containing the algorithmic part of the package.
 */
class Algorithm
{
 public:
  /*!
   * Constructor.
   */
  Algorithm();

  /*!
   * Destructor.
   */
  virtual ~Algorithm();

  /*!
   * Add new measurement data.
   * @param data the new data.
   */
  void addData(const double data);

  /*!
   * Add multiple measurements as once.
   * @param data new data.
   */
  void addData(const Eigen::VectorXd& data);

  /*!
   * Get the min of the data.
   * @return the min of the data.
   */
  double getMin() const;

 private:
 
  //! Forward declared structure that will contain the data
  struct Data;

  //! Pointer to data (pimpl)
  std::unique_ptr<Data> data_;
};

} /* namespace */