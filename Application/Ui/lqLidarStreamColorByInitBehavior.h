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
#ifndef lqLidarStreamColorByInitBehavior_H
#define lqLidarStreamColorByInitBehavior_H

#include <QObject>

#include "applicationui_export.h"

class pqPipelineSource;
class vtkSMSourceProxy;

/**
 * @class lqLidarStreamColorByInitBehavior
 * @ingroup Behaviors
 *
 * lqLidarStreamColorByInitBehavior helps to set the colorize array,
 * once a LidarStream(LiveSource) has received data and generated an Output.
 * The colorize array depend on the sensor type,
 * that we can know only after receiving the first data.
 */
class APPLICATIONUI_EXPORT lqLidarStreamColorByInitBehavior : public QObject
{
  Q_OBJECT
  typedef QObject Superclass;

public:
  lqLidarStreamColorByInitBehavior(QObject* parent = 0);

protected slots:
  void sourceAdded(pqPipelineSource* src);
  void dataUpdated(pqPipelineSource* src);

private:
  /**
   * @brief Check if LidarStream has indeed Updated, and tries to color by the rigth array
   *        ("reflectivity" if the detected sensor is an Advanced, "intensity" otherwise)
   * @param[in] proxy A pointer to the LidarStream proxy reporting an update.
   * Return true , if procedure has completed (successfully or not), requests a disconnect.
   * Return false, if information is missing (dataStream has not started) and further attempts are needed.
   * The procedure works in multiple attempts:
   *  - Does nothing if LidarStream has not received any (or not enough) Data yet (aka Failed Attempt).
   *  - Color by "reflectivity" or "intensity" according to the detected sensor.
  */
  bool tryLidarStreamInitColorBy(vtkSMSourceProxy* proxy);

  Q_DISABLE_COPY(lqLidarStreamColorByInitBehavior)
};

#endif // lqLidarStreamColorByInitBehavior_h
