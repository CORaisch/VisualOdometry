#include "TrajectoryViz.hpp"

void TrajectoryVisualizer::Init(double fx, double fy, double cx, double cy)
{
  // init members
  this->bRun = true;
  this->bFollowCamera = true;
  this->dScale = 1.0;
  this->dDelta = 1.0;
  this->dStep = 10.0;
  this->iSpin = 1;
  this->mK = cv::Matx33d(fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
  this->vPoses.push_back(cv::Affine3d(cv::Mat::eye(3,3,CV_64F), cv::Vec3d(0.0, 0.0, 0.0)));
  this->vCurrentPose.push_back(cv::Affine3d(cv::Mat::eye(3,3,CV_64F), cv::Vec3d(0.0, 0.0, 0.0)));

  // init rendering window
  this->v3dWindow = cv::viz::Viz3d("3D Trajectory");
  this->v3dWindow.setBackgroundColor(cv::viz::Color::white(), cv::viz::Color::black());

  // register keyboard callback
  this->v3dWindow.registerKeyboardCallback(TrajectoryVisualizer::keyboardViz3d, this);
}

void TrajectoryVisualizer::Shutdown()
{
  this->bRun = false;
}

void TrajectoryVisualizer::UpdateTrajectory(std::vector<Frame>& vFrames)
{
  std::unique_lock<mutex> lock(mMutexTraj);

  /* reset poses and points */
  this->vPoses.clear();
  this->vPoints.clear();
  this->vPoses.reserve(vFrames.size());
  this->vPoints.reserve(vFrames.back().points3D.size());

  /* compute list of new poses */
  for(int i = 0; i < vFrames.size(); ++i)
  {
    Frame f = vFrames[i];
    cv::Vec3d vt(double(f.wtc.at<float>(0,0)), double(f.wtc.at<float>(1,0)), double(f.wtc.at<float>(2,0)));
    cv::Mat wRc_conv;
    f.wRc.convertTo(wRc_conv, CV_64F);
    this->vPoses.push_back(cv::Affine3d(wRc_conv, vt));
  }

  /* set new current pose */
  this->vCurrentPose[0] = this->vPoses.back();

  /* compute list of current reconstructed points */
  std::vector<MapPoint> vCurrentPoints = vFrames.back().points3D;
  for(int i = 0; i < vCurrentPoints.size(); ++i)
  {
    // INVARIANT it is assumed that points3D are already transformed to ws
    MapPoint p3D = vCurrentPoints[i];
    if(p3D.isValid())
    {
      cv::Point3d p(double(p3D.pt_ws.x), double(p3D.pt_ws.y), double(p3D.pt_ws.z));
      this->vPoints.push_back(p);
    }
  }
}

void TrajectoryVisualizer::UpdateTrajectory(std::vector<cv::Affine3d>& vNewPoses, std::vector<cv::Point3d>& vNewPoints)
{
  std::unique_lock<mutex> lock(mMutexTraj);
  this->vPoses = vNewPoses;
  this->vPoints = vNewPoints;
}

void TrajectoryVisualizer::Run()
{
  while(bRun)
  {
    /* time control */
    this->tStart = cv::getTickCount();

    /* create viz3d-widget for trajectory */
    {
      // lock trajectory data
      std::unique_lock<mutex> lock(mMutexTraj);

      cv::viz::WTrajectory wPath(this->vPoses, cv::viz::WTrajectory::PATH, this->dScale, cv::viz::Color::green());
      cv::viz::WTrajectoryFrustums wFrustrums(this->vPoses, this->mK, this->dScale, cv::viz::Color::blue());
      cv::viz::WTrajectoryFrustums wCurrentPose(this->vCurrentPose, this->mK, this->dScale, cv::viz::Color::green());    
      wPath.setRenderingProperty(cv::viz::LINE_WIDTH, 4.0);

      /* render trajectory */
      v3dWindow.showWidget("Trajectory Frustrums", wFrustrums);
      v3dWindow.showWidget("Trajectory Current Pose", wCurrentPose);
      v3dWindow.showWidget("Trajectory Path", wPath);

      /* render point cloud */
      if(!this->vPoints.empty())
      {
	  cv::viz::WCloud wPoints(this->vPoints, cv::viz::Color::red());
	  wPoints.setRenderingProperty(cv::viz::POINT_SIZE, 3.0);
	  v3dWindow.showWidget("Point Cloud", wPoints);
      }
    }

    /* set new viewer pose, such that observing camera is following the latest frame */
    if(this->bFollowCamera)
    {
      // get latest pose
      cv::Affine3d viewerPose = vCurrentPose.back();
      // transform viewer pose slightly behind latest frame, such that most points of the cloud are visible
      double dTheta = 0.05; // angle the viewer pose is rotated around the x axis in cs
      cv::Vec3d rvec = cv::Vec3d(-1.0, 0.0, 0.0)*CV_PI*dTheta;
      cv::Matx33d rmat;
      cv::Rodrigues(rvec, rmat);
      viewerPose.rotation(viewerPose.rotation()*rmat);
      cv::Vec3d trans_relative(0.0, -1.5, -9.0);
      viewerPose = viewerPose.translate(viewerPose.rotation()*trans_relative);
      // set new viewe pose
      v3dWindow.setViewerPose(viewerPose);
    }

    /* update rendering */
    v3dWindow.spinOnce(iSpin, true);

    /* time control */
    this->tStop = cv::getTickCount();
    this->dDelta = double(tStop-tStart)/cv::getTickFrequency();
  }

  // close window when leaving loop
  this->v3dWindow.close();
}
