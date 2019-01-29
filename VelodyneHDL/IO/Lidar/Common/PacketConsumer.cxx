#include "PacketConsumer.h"

#include "SynchronizedQueue.h"
#include "vtkAppendPolyData.h"

//----------------------------------------------------------------------------
PacketConsumer::PacketConsumer()
{
  this->NewData = false;
  this->ShouldCheckSensor = true;
  this->MaxNumberOfFrames = 1000;
  this->LastTime = 0.0;
  this->Timesteps.clear();
  this->Frames.clear();
  this->Packets.reset(new SynchronizedQueue<std::string*>);
}

//----------------------------------------------------------------------------
void PacketConsumer::HandleSensorData(const unsigned char *data, unsigned int length)
{
  boost::lock_guard<boost::mutex> lock(this->ReaderMutex);
  this->Interpreter->ProcessPacket(data, length);
  if (this->Interpreter->IsNewFrameReady())
  {
    this->HandleNewData(this->Interpreter->GetLastFrameAvailable());
    this->Interpreter->ClearAllFramesAvailable();
  }
}

//----------------------------------------------------------------------------
vtkSmartPointer<vtkPolyData> PacketConsumer::GetFrameForTime(double timeRequest, double &actualTime, int numberOfTrailingFrames)
{
  size_t stepIndex = this->GetIndexForTime(timeRequest);
  if (stepIndex < this->Timesteps.size())
  {
    actualTime = this->Timesteps[stepIndex];
    if(numberOfTrailingFrames == 0)
    {
      return this->Frames[stepIndex];
    }
    else
    {
      vtkSmartPointer<vtkAppendPolyData> appendFilter = vtkSmartPointer<vtkAppendPolyData>::New();
      for (size_t i = stepIndex - std::min(stepIndex, static_cast<size_t>(numberOfTrailingFrames));
           i < stepIndex; ++i)
      {
        appendFilter->AddInputData(this->Frames[i]);
      }

      appendFilter->Update();
      return vtkSmartPointer<vtkPolyData>(appendFilter->GetOutput());
    }
  }
  actualTime = 0;
  return 0;
}

//----------------------------------------------------------------------------
std::vector<double> PacketConsumer::GetTimesteps()
{
  boost::lock_guard<boost::mutex> lock(this->ConsumerMutex);
  const size_t nTimesteps = this->Timesteps.size();
  std::vector<double> timesteps(nTimesteps, 0);
  for (size_t i = 0; i < nTimesteps; ++i)
  {
    timesteps[i] = this->Timesteps[i];
  }
  return timesteps;
}

//----------------------------------------------------------------------------
void PacketConsumer::SetMaxNumberOfFrames(int nFrames)
{
  boost::lock_guard<boost::mutex> lock(this->ConsumerMutex);
  this->MaxNumberOfFrames = nFrames;
  this->UpdateDequeSize();
}

//----------------------------------------------------------------------------
bool PacketConsumer::CheckForNewData()
{
  boost::lock_guard<boost::mutex> lock(this->ConsumerMutex);
  bool newData = this->NewData;
  this->NewData = false;
  return newData;
}

//----------------------------------------------------------------------------
void PacketConsumer::ThreadLoop()
{
  std::string* packet = 0;
  this->Interpreter->ResetCurrentFrame();
  while (this->Packets->dequeue(packet))
  {
    this->HandleSensorData(
          reinterpret_cast<const unsigned char*>(packet->c_str()), packet->length());
    delete packet;
  }
}

//----------------------------------------------------------------------------
void PacketConsumer::Start()
{
  if (this->Thread)
  {
    return;
  }

  this->Packets.reset(new SynchronizedQueue<std::string*>);
  this->Thread = boost::shared_ptr<boost::thread>(
        new boost::thread(boost::bind(&PacketConsumer::ThreadLoop, this)));
}

//----------------------------------------------------------------------------
void PacketConsumer::Stop()
{
  if (this->Thread)
  {
    this->Packets->stopQueue();
    this->Thread->join();
    this->Thread.reset();
    this->Packets.reset();
  }
}

//----------------------------------------------------------------------------
void PacketConsumer::Enqueue(std::string *packet) { this->Packets->enqueue(packet); }

//----------------------------------------------------------------------------
void PacketConsumer::UnloadData()
{
  this->Frames.clear();
  this->Timesteps.clear();
}

//----------------------------------------------------------------------------
void PacketConsumer::UpdateDequeSize()
{
  if (this->MaxNumberOfFrames <= 0)
  {
    return;
  }
  while (static_cast<int>(this->Frames.size()) >= this->MaxNumberOfFrames)
  {
    this->Frames.pop_front();
    this->Timesteps.pop_front();
  }
}

//----------------------------------------------------------------------------
size_t PacketConsumer::GetIndexForTime(double time)
{
  size_t index = 0;
  double minDifference = VTK_DOUBLE_MAX;
  const size_t nTimesteps = this->Timesteps.size();
  for (size_t i = 0; i < nTimesteps; ++i)
  {
    double difference = std::abs(this->Timesteps[i] - time);
    if (difference < minDifference)
    {
      minDifference = difference;
      index = i;
    }
  }
  return index;
}

//----------------------------------------------------------------------------
void PacketConsumer::HandleNewData(vtkSmartPointer<vtkPolyData> polyData)
{
  boost::lock_guard<boost::mutex> lock(this->ConsumerMutex);

  this->UpdateDequeSize();
  this->Timesteps.push_back(this->LastTime);
  this->Frames.push_back(polyData);
  this->NewData = true;
  this->LastTime += 1.0;
}
