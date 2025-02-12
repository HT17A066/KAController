﻿// -*- C++ -*-
/*!
 * @file  KAController.h
 * @brief ${rtcParam.description}
 * @date  $Date$
 *
 * $Id$
 */

#ifndef KACONTROLLER_H
#define KACONTROLLER_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/idl/InterfaceDataTypesSkel.h>

// Service implementation headers
// <rtc-template block="service_impl_h">

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">
#include "BasicDataTypeStub.h"

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="port_stub_h">
// </rtc-template>

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

//#include <Eigen>
#include "Eigen/LU"
#include "Eigen/Geometry"
#include "Eigen/Dense"

using namespace RTC;

/*!
 * @class KAController
 * @brief ${rtcParam.description}
 *
 */
class KAController
  : public RTC::DataFlowComponentBase {
public:
 /*!
  * @brief constructor
  * @param manager Maneger Object
  */
  KAController(RTC::Manager* manager);

  /*!
   * @brief destructor
   */
  ~KAController();

  // <rtc-template block="public_attribute">

  // </rtc-template>

  // <rtc-template block="public_operation">

  // </rtc-template>

  /***
   *
   * The initialize action (on CREATED->ALIVE transition)
   * formaer rtc_init_entry()
   *
   * @return RTC::ReturnCode_t
   *
   *
   */
  virtual RTC::ReturnCode_t onInitialize();

 /***
  *
  * The finalize action (on ALIVE->END transition)
  * formaer rtc_exiting_entry()
  *
  * @return RTC::ReturnCode_t
  *
  *
  */
 // virtual RTC::ReturnCode_t onFinalize();

 /***
  *
  * The startup action when ExecutionContext startup
  * former rtc_starting_entry()
  *
  * @param ec_id target ExecutionContext Id
  *
  * @return RTC::ReturnCode_t
  *
  *
  */
 // virtual RTC::ReturnCode_t onStartup(RTC::UniqueId ec_id);

 /***
  *
  * The shutdown action when ExecutionContext stop
  * former rtc_stopping_entry()
  *
  * @param ec_id target ExecutionContext Id
  *
  * @return RTC::ReturnCode_t
  *
  *
  */
 // virtual RTC::ReturnCode_t onShutdown(RTC::UniqueId ec_id);

 /***
  *
  * The activated action (Active state entry action)
  * former rtc_active_entry()
  *
  * @param ec_id target ExecutionContext Id
  *
  * @return RTC::ReturnCode_t
  *
  *
  */
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

 /***
  *
  * The deactivated action (Active state exit action)
  * former rtc_active_exit()
  *
  * @param ec_id target ExecutionContext Id
  *
  * @return RTC::ReturnCode_t
  *
  *
  */
  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

 /***
  *
  * The execution action that is invoked periodically
  * former rtc_active_do()
  *
  * @param ec_id target ExecutionContext Id
  *
  * @return RTC::ReturnCode_t
  *
  *
  */
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

 /***
  *
  * The aborting action when main logic error occurred.
  * former rtc_aborting_entry()
  *
  * @param ec_id target ExecutionContext Id
  *
  * @return RTC::ReturnCode_t
  *
  *
  */
 // virtual RTC::ReturnCode_t onAborting(RTC::UniqueId ec_id);

 /***
  *
  * The error action in ERROR state
  * former rtc_error_do()
  *
  * @param ec_id target ExecutionContext Id
  *
  * @return RTC::ReturnCode_t
  *
  *
  */
 // virtual RTC::ReturnCode_t onError(RTC::UniqueId ec_id);

 /***
  *
  * The reset action that is invoked resetting
  * This is same but different the former rtc_init_entry()
  *
  * @param ec_id target ExecutionContext Id
  *
  * @return RTC::ReturnCode_t
  *
  *
  */
 // virtual RTC::ReturnCode_t onReset(RTC::UniqueId ec_id);

 /***
  *
  * The state update action that is invoked after onExecute() action
  * no corresponding operation exists in OpenRTm-aist-0.2.0
  *
  * @param ec_id target ExecutionContext Id
  *
  * @return RTC::ReturnCode_t
  *
  *
  */
 // virtual RTC::ReturnCode_t onStateUpdate(RTC::UniqueId ec_id);

 /***
  *
  * The action that is invoked when execution context's rate is changed
  * no corresponding operation exists in OpenRTm-aist-0.2.0
  *
  * @param ec_id target ExecutionContext Id
  *
  * @return RTC::ReturnCode_t
  *
  *
  */
 // virtual RTC::ReturnCode_t onRateChanged(RTC::UniqueId ec_id);


protected:
 // <rtc-template block="protected_attribute">

 // </rtc-template>

 // <rtc-template block="protected_operation">

 // </rtc-template>

 // Configuration variable declaration
 // <rtc-template block="config_declare">

 // </rtc-template>

 // DataInPort declaration
 // <rtc-template block="inport_declare">
  RTC::TimedDoubleSeq m_cylinder;
  /*!
   */
  InPort<RTC::TimedDoubleSeq> m_cylinderIn;
  RTC::TimedDoubleSeq m_armStatus;
  /*!
   */
  InPort<RTC::TimedDoubleSeq> m_armStatusIn;
  RTC::TimedString m_inStatus;
  /*!
   */
  InPort<RTC::TimedString> m_inStatusIn;

  // </rtc-template>


  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  RTC::TimedDoubleSeq m_armTipTarget;
  /*!
   */
  OutPort<RTC::TimedDoubleSeq> m_armTipTargetOut;
  RTC::TimedDoubleSeq m_outData;
  /*!
   */
  OutPort<RTC::TimedDoubleSeq> m_outDataOut;
  RTC::TimedString m_outStatus;
  /*!
   */
  OutPort<RTC::TimedString> m_outStatusOut;
  RTC::TimedString m_camera_xy;
  /*!
   */
  OutPort<RTC::TimedString> m_camera_xyOut;

  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">

  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">

  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">

  // </rtc-template>

private:
 // <rtc-template block="private_attribute">

 // </rtc-template>

 // <rtc-template block="private_operation">

 // </rtc-template>

  std::vector <double> ave_x;
  std::vector <double> ave_y;
  std::vector <double> ave_z;
  std::vector <double> ave_r;
  int NotFound = 0;

  enum Judge { Ignore, Passed };
  Judge judge;

  Eigen::MatrixXd transform0;

};


extern "C"
{
  DLL_EXPORT void KAControllerInit(RTC::Manager* manager);
};

#endif // KACONTROLLER_H
