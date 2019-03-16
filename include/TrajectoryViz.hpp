#ifndef _TRAJECTORYVIZ_HPP
#define _TRAJECTORYVIZ_HPP

#include <opencv2/viz.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <mutex>

#include "Frame.hpp"
#include "MapPoint.hpp"

using namespace std;

class TrajectoryVisualizer
{
public:
  /* constructors */
  TrajectoryVisualizer() {};

  /* destructor */
  ~TrajectoryVisualizer() {};

  /* set up rendering frames and basic visualization data */
  void Init(double fx, double fy, double cx, double cy);

  /* runs the rendering loop till Shutdown is called */
  void Run();

  /* stops the rendering loop */
  void Shutdown();

  /* update poses and points to render */
  void UpdateTrajectory(std::vector<cv::Affine3d>& vNewPoses, std::vector<cv::Point3d>& vNewPoints);
  void UpdateTrajectory(std::vector<Frame>& vFrames);

  /* keyboard event function */
  static void keyboardViz3d(const cv::viz::KeyboardEvent &w, void *t)
  {
    TrajectoryVisualizer *trajViz = (TrajectoryVisualizer*)t;
    viz::Viz3d *wnd = &(trajViz->v3dWindow);
    if (w.action) // react on any action
      {
	switch(w.code)
	  {	
	  case 'p':
	  case 'P': // follow cam / keep position on 'p/P'
	    trajViz->bFollowCamera = !trajViz->bFollowCamera;
	    break;
	  case 'w':
	  case 'W': // move forward on 'w/W'
	    if(!trajViz->bFollowCamera)
	    {
	      cv::Affine3d pose = wnd->getViewerPose();
	      cv::Vec3d t_rel(0.0, 0.0, trajViz->dDelta*trajViz->dStep);
	      pose = pose.translate(pose.rotation()*t_rel);
	      wnd->setViewerPose(pose);
	    }
	    break;
	  case 's':
	  case 'S': // move backward on 's/S'
	    if(!trajViz->bFollowCamera)
	    {
	      cv::Affine3d pose = wnd->getViewerPose();
	      cv::Vec3d t_rel(0.0, 0.0, -trajViz->dDelta*trajViz->dStep);
	      pose = pose.translate(pose.rotation()*t_rel);
	      wnd->setViewerPose(pose);
	    }
	    break;
	  case 'a':
	  case 'A': // strive left on 'a/A'
	    if(!trajViz->bFollowCamera)
	    {
	      cv::Affine3d pose = wnd->getViewerPose();
	      cv::Vec3d t_rel(-trajViz->dDelta*trajViz->dStep, 0.0, 0.0);
	      pose = pose.translate(pose.rotation()*t_rel);
	      wnd->setViewerPose(pose);
	    }
	    break;
	  case 'd':
	  case 'D': // strive right on 'd/D'
	    if(!trajViz->bFollowCamera)
	    {
	      cv::Affine3d pose = wnd->getViewerPose();
	      cv::Vec3d t_rel(trajViz->dDelta*trajViz->dStep, 0.0, 0.0);
	      pose = pose.translate(pose.rotation()*t_rel);
	      wnd->setViewerPose(pose);
	    }
	    break;
	  }
      }
  }

private:
  bool bRun, bFollowCamera;
  double dScale, dDelta, dStep;
  int64 tStart, tStop;
  int iSpin;
  cv::Matx33d mK;
  std::vector<cv::Affine3d> vPoses;
  std::vector<cv::Affine3d> vCurrentPose;
  std::vector<cv::Point3d>  vPoints;
  cv::viz::Viz3d v3dWindow;

  std::mutex mMutexTraj;
};

#endif //_TRAJECTORYVIZ_HPP
