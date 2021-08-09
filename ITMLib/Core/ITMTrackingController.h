// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <stdexcept>

#include "../Engines/Visualisation/Interface/ITMSurfelVisualisationEngine.h"
#include "../Engines/Visualisation/Interface/ITMVisualisationEngine.h"
#include "../Trackers/Interface/ITMTracker.h"
#include "../Utils/ITMLibSettings.h"
#include <iostream>
#include <Eigen/Dense>
#include <vector>
namespace ITMLib
{
	/** \brief
	*/
	class ITMTrackingController
	{
	private:
		const ITMLibSettings *settings;

	public:
		ITMTracker *tracker;
        //728 dump pose
        //std::ofstream tc_file;
        std::vector<std::vector<float>>tc_array;

		void Track(ITMTrackingState *trackingState, const ITMView *view)
		{
			tracker->TrackCamera(trackingState, view);
		}

		template <typename TSurfel>
		void Prepare(ITMTrackingState *trackingState, const ITMSurfelScene<TSurfel> *scene, const ITMView *view,
			const ITMSurfelVisualisationEngine<TSurfel> *visualisationEngine, ITMSurfelRenderState *renderState)
		{
			if (!tracker->requiresPointCloudRendering())
				return;

			//render for tracking
			bool requiresColourRendering = tracker->requiresColourRendering();
			bool requiresFullRendering = trackingState->TrackerFarFromPointCloud() || !settings->useApproximateRaycast;

			if(requiresColourRendering)
			{
				// TODO: This should be implemented at some point.
				throw std::runtime_error("The surfel engine doesn't yet support colour trackers");
			}
			else
			{

				const bool useRadii = true;
				visualisationEngine->FindSurface(scene, trackingState->pose_d, &view->calib.intrinsics_d, useRadii, USR_FAUTEDEMIEUX, renderState);
				trackingState->pose_pointCloud->SetFrom(trackingState->pose_d);

				if(requiresFullRendering)
				{
					visualisationEngine->CreateICPMaps(scene, renderState, trackingState);
					trackingState->pose_pointCloud->SetFrom(trackingState->pose_d);
					if (trackingState->age_pointCloud==-1) trackingState->age_pointCloud=-2;
					else trackingState->age_pointCloud = 0;
				}
				else
				{
					trackingState->age_pointCloud++;
				}
			}
		}

		template <typename TVoxel, typename TIndex>
		void Prepare(ITMTrackingState *trackingState, const ITMScene<TVoxel,TIndex> *scene, const ITMView *view,
			const ITMVisualisationEngine<TVoxel,TIndex> *visualisationEngine, ITMRenderState *renderState)
		{
			if (!tracker->requiresPointCloudRendering())
				return;

			//render for tracking
			bool requiresColourRendering = tracker->requiresColourRendering();
			bool requiresFullRendering = trackingState->TrackerFarFromPointCloud() || !settings->useApproximateRaycast;

			if (requiresColourRendering)
			{
				ORUtils::SE3Pose pose_rgb(view->calib.trafo_rgb_to_depth.calib_inv * trackingState->pose_d->GetM());
				visualisationEngine->CreateExpectedDepths(scene, &pose_rgb, &(view->calib.intrinsics_rgb), renderState);
				visualisationEngine->CreatePointCloud(scene, view, trackingState, renderState, settings->skipPoints);
				trackingState->age_pointCloud = 0;
			}
			else
			{
                //std::cout<<"view calib intrinsic: "<<view->calib.intrinsics_d.projectionParamsSimple.all<<std::endl;
				visualisationEngine->CreateExpectedDepths(scene, trackingState->pose_d, &(view->calib.intrinsics_d), renderState);
				//std::cout<<"just to be sure\n";
				if (requiresFullRendering)
				{
					visualisationEngine->CreateICPMaps(scene, view, trackingState, renderState);
					trackingState->pose_pointCloud->SetFrom(trackingState->pose_d);
                    //pyh
                    std::vector<float> single_pose;
                    single_pose.push_back(trackingState->pose_d->params.each.tx);
                    single_pose.push_back(trackingState->pose_d->params.each.ty);
                    single_pose.push_back(trackingState->pose_d->params.each.tz);
                    Eigen::Matrix3f testr;
                    testr(0,0) = trackingState->pose_d->GetM().m[0];
                    testr(1,0) = trackingState->pose_d->GetM().m[1];
                    testr(2,0) = trackingState->pose_d->GetM().m[2];
                    testr(0,1) = trackingState->pose_d->GetM().m[4];
                    testr(1,1) = trackingState->pose_d->GetM().m[5];
                    testr(2,1) = trackingState->pose_d->GetM().m[6];
                    testr(0,2) = trackingState->pose_d->GetM().m[8];
                    testr(1,2) = trackingState->pose_d->GetM().m[9];
                    testr(2,2) = trackingState->pose_d->GetM().m[10];

                    Eigen::Quaternionf testq(testr);
                    single_pose.push_back(testq.x());
                    single_pose.push_back(testq.y());
                    single_pose.push_back(testq.z());
                    single_pose.push_back(testq.w());
                    tc_array.push_back(single_pose);

					if (trackingState->age_pointCloud==-1) {
						trackingState->age_pointCloud=-2;
					}
					else {
						trackingState->age_pointCloud = 0;
					}
				}
				else
				{
					visualisationEngine->ForwardRender(scene, view, trackingState, renderState);
					trackingState->age_pointCloud++;
				}
			}
		}

        void DumpRecordedPose()
        {
            for(auto i=0; i<tc_array.size(); i++)
            {
                for(auto j=0; j<(tc_array[i].size()-1); j++)
                {
                    //tc_file<<tc_array[i][j]<<" ";
                }
                //tc_file<<tc_array[i][(tc_array[i].size()-1)]<<std::endl;
            }
            //tc_file.close();
        }

		ITMTrackingController(ITMTracker *tracker, const ITMLibSettings *settings)
		{
			this->tracker = tracker;
			this->settings = settings;
            //pyh
            //tc_file.open("728_otherloc_me_tc.csv");
		}

		const Vector2i& GetTrackedImageSize(const Vector2i& imgSize_rgb, const Vector2i& imgSize_d) const
		{
			return tracker->requiresColourRendering() ? imgSize_rgb : imgSize_d;
		}

		// Suppress the default copy constructor and assignment operator
		ITMTrackingController(const ITMTrackingController&);
		ITMTrackingController& operator=(const ITMTrackingController&);
	};
}
