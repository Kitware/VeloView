/*=========================================================================

   Program: LidarView
   Module:  lqLidarStreamColorByInitBehavior.h

   Copyright (c) Kitware Inc.
   All rights reserved.

   LidarView is a free software; you can redistribute it and/or modify it
   under the terms of the LidarView license.

   See LICENSE for the full LidarView license.
   A copy of this license can be obtained by contacting
   Kitware Inc.
   28 Corporate Drive
   Clifton Park, NY 12065
   USA

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

========================================================================*/
#include "lqLidarStreamColorByInitBehavior.h"

#include <pqServerManagerModel.h>
#include <pqApplicationCore.h>
#include <pqActiveObjects.h>
#include <pqPipelineSource.h>

#include <vtkSMSourceProxy.h>
#include <vtkSMViewProxy.h>
#include <vtkSMPVRepresentationProxy.h>
#include <lqHelper.h>

#include "vtkLidarStream.h"

#include <iostream>

//-----------------------------------------------------------------------------
lqLidarStreamColorByInitBehavior::lqLidarStreamColorByInitBehavior(QObject *parent)
  : Superclass(parent)
{
  //Monitor added sources
  pqServerManagerModel* smmodel = pqApplicationCore::instance()->getServerManagerModel();
  this->connect(smmodel, SIGNAL(sourceAdded(pqPipelineSource*)), SLOT(sourceAdded(pqPipelineSource*)));
}

//-----------------------------------------------------------------------------
bool lqLidarStreamColorByInitBehavior::tryLidarStreamInitColorBy(vtkSMSourceProxy* proxy)
{
  //Check OutputPort Name
  int framePort = -1;
  for(std::size_t i = 0; i < proxy->GetNumberOfOutputPorts(); ++i )
  {
    if( proxy->GetOutputPortName(i) == std::string("Frame"))
    {
      framePort = i;
      break;
    }
  }
  if( framePort == -1 )
  {
    std::cerr << __FUNCTION__ << " LidarStream has no \'Frame\' Port" << std::endl;
    return true; // Error, Do not try again
  }

  //Get Repr Proxy
  pqView* view = pqActiveObjects::instance().activeView(); //WIP finding RenderView1 is lucky here
  vtkSMViewProxy* viewProxy = vtkSMViewProxy::SafeDownCast(view->getProxy());
  vtkSMPVRepresentationProxy* pvrp = vtkSMPVRepresentationProxy::SafeDownCast(
    viewProxy->FindRepresentation(proxy,0)
  );

  //Note: It is safe to assume, that if 'framePort' has no Repr, stream has not arrived yet
  if( !pvrp )
  {
    //No Data Received yet
    return false; //Try again later
  }

  // Get the sensor information of the stream to help us determine the colorize array
  std::string sensorInfo = "";
  vtkLidarStream* lidarStream = vtkLidarStream::SafeDownCast(proxy->GetClientSideObject());
  if(lidarStream)
  {
    sensorInfo = lidarStream->GetSensorInformation();
  }
  else
  {
    return true;
  }

  if(sensorInfo.find("Could not determine") != std::string::npos)
  {
    // The sensor information could be "not determine" in two cases :
    // - No packet are received yet (should be handle by the pvrp test above
    // - Not enough packet are received to determine the sensor (case of live calibration)
    // We return false to try again at the next packet.
    return false;
  }

  // If the sensor is an APF one (apf or Special Velarray, we color by reflectivity
  // If the sensor is a "Legacy" one, we color by intensitys
  if(sensorInfo.find("advanced") != std::string::npos)
  {
    pvrp->SetScalarColoring("reflectivity", 0);
  }
  else
  {
    pvrp->SetScalarColoring("intensity", 0);
  }

  return true;

}

//-----------------------------------------------------------------------------
void lqLidarStreamColorByInitBehavior::sourceAdded(pqPipelineSource* src)
{
  if ( IsLidarStreamProxy(src->getProxy()) )
  {
    //Ask lidarSource to report its updates
    this->connect(src, SIGNAL(dataUpdated(pqPipelineSource*)), SLOT(dataUpdated(pqPipelineSource*)));
  }
}

//-----------------------------------------------------------------------------
void lqLidarStreamColorByInitBehavior::dataUpdated(pqPipelineSource* src)
{
  if ( !IsLidarStreamProxy(src->getProxy()) )
  {
    std::cerr << __FUNCTION__ << " Wrong source type has been connected to behavior" << std::endl;
    return;
  }

  //Try to color by the right array
  if( IsLidarStreamProxy(src->getProxy()) &&
      this->tryLidarStreamInitColorBy( vtkSMSourceProxy::SafeDownCast(src->getProxy())) )
  {
    //Disconnect dataUpdate Signal, if procedure comes to an end.
    this->disconnect(src, SIGNAL(dataUpdated(pqPipelineSource*)), this,  SLOT(dataUpdated(pqPipelineSource*)));
  }

}
