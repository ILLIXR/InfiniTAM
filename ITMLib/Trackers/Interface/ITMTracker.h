// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../Objects/Tracking/ITMTrackingState.h"
#include "../../Objects/Views/ITMView.h"
#include <chrono>
namespace ITMLib
{
	/** \brief
	    Basic interface to any sort of :rackers that will align an
	    incoming view with an existing scene.
	*/
	class ITMTracker
	{
	public:
		/** Gets whether the tracker can keep tracking or not.
		    Can be used to signal e.g. the end of a sequence
		    of file-based poses, or the failure of an IMU.
		*/
		virtual bool CanKeepTracking() const { return true; }

		/** Localize a View in the given scene. The result is
		    currently stored as an attribute in trackingState.
		*/
		virtual void TrackCamera(ITMTrackingState *trackingState, const ITMView *view) = 0;

		/** Updates the initial pose of the depth camera in the scene.
		    This can be used to make the scene up vector correspond
		    to the real world's up direction.
		*/
		virtual void UpdateInitialPose(ITMTrackingState *trackingState) {}

		virtual bool requiresColourRendering() const = 0;
		virtual bool requiresDepthReliability() const = 0;
		virtual bool requiresPointCloudRendering() const = 0;

		virtual ~ITMTracker(void) {}
        //pyh
        virtual void EvaluationPrep(ITMTrackingState *trackingState, const ITMView *view) =0;
        virtual void SetTrackingState(ITMTrackingState *trackingState, const ITMView *view){}
        virtual void SetInvPose(Matrix4f& Inv_M){}
        virtual unsigned GetICPIterNo(){}
        virtual std::chrono::nanoseconds GetICPLoopTime(){}
	};
}
