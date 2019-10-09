// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "Carla/Sensor/SceneCaptureSensor.h"

#include "Carla/Actor/ActorDefinition.h"
#include "Carla/Sensor/PixelReader.h"

#include "SceneCaptureCamera.generated.h"

/// A sensor that captures images from the scene.
UCLASS()
class CARLA_API ASceneCaptureCamera : public ASceneCaptureSensor
{
  GENERATED_BODY()

public:

  static FActorDefinition GetSensorDefinition();

protected:

  void Tick(float DeltaTime) override;
};
