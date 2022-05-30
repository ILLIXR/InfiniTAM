// Copyright 2022 RSIM Group

#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <type_traits>
#include <vector>

#include "MathUtils.h"

namespace ORUtils
{
	// Based on https://dl.acm.org/doi/pdf/10.1145/3410463.3414636
	// std::is_arithmetic<T>::value **MUST** be true
	template<class T>
	class GenericPIDController
	{
	private:
		// Control coefficients
		T Kp;
		T Ki;
		T Kd;

		// Dynamic setpoint info
		T alpha;
		T count{};
		T sumVal{};
		T minVal{std::numeric_limits<T>::max()};
		T maxVal{std::numeric_limits<T>::min()};
		T lowPoint{};
		T highPoint{};
		T binSize{};

		// Valid outputs
		unsigned minOutputIdx;
		unsigned maxOutputIdx;
		T numOutputs;
		std::vector<T> outputs;

		// Sliding window
		std::vector<T> window;
		static constexpr unsigned WINDOW_SIZE{30};

		// Bootstrapping -- lasts 1 second at 30 fps
		bool bootstrap{true};
		static constexpr unsigned BOOTSTRAP_LENGTH{30};

		T lastError{};

	private:
		void enqueue(T val)
		{
			window.push_back(val);
			if (window.size() == (WINDOW_SIZE + 1))
				window.erase(window.begin());
		}

		T average() const
		{
			return std::accumulate(window.begin(), window.end(), T{}) / static_cast<double>(window.size());
		}

		void updateSetpoints(T val)
		{
			// Keep track of min, max, and sum of vals
			minVal = MIN(minVal, val);
			maxVal = MAX(maxVal, val);
			sumVal += val;

			// Update dynamic setpoints. Refer to Section 4.2 of https://dl.acm.org/doi/pdf/10.1145/3410463.3414636 for more details.
			T avgVal = sumVal / count;
			lowPoint = ((1.0f - alpha) * avgVal) + (alpha * minVal);
			highPoint = (alpha * avgVal) + ((1.0f - alpha) * maxVal);
			binSize = (highPoint - lowPoint) / numOutputs;
		}

		T calculatePIdx(T val) const
		{
			// Linear interpolation between low and high, with the range
			// divided into equal sized 'numOutputs' bins
			return (val - lowPoint) / binSize;
		}

		T calculateIIdx() const
		{
			// Compute the index for the average value of the sliding window
			// TODO: P(avg()) or avg(P())?
			// TODO: should the window contain 'val' or should it contain 'P(val)'?
			return calculatePIdx(average());
		}

		T calculateDIdx() const
		{
			bool ascending = std::is_sorted(window.begin(), window.end());

			// Go up a bin
			if (ascending)
				return T{1};
			else
				return T{};
		}

		T calculate(T val) const
		{
			// If the value is too low, run at min
			if (val < lowPoint)
				return outputs[minOutputIdx];

			// If the value is too high, run at max
			if (val > highPoint)
				return outputs[maxOutputIdx];

			// Proportional term
			T pIdx = Kp * calculatePIdx(val);

			// Integral term
			T iIdx = Ki * calculateIIdx();

			// Derivative term
			T dIdx = Kd * calculateDIdx();

			// Final output index
			T outputIdx = pIdx + iIdx + dIdx;

			// Convert to bin index and clamp to [min, max]
			outputIdx = CLAMP(std::floor(outputIdx), minOutputIdx, maxOutputIdx);

			return outputs[outputIdx];
		}

	public:
		GenericPIDController(T Kp_, T Ki_, T Kd_, T alpha_, std::vector<T> outputs_)
			: Kp{Kp_}
			, Ki{Ki_}
			, Kd{Kd_}
			, alpha{alpha_}
			, minOutputIdx{0}
			, maxOutputIdx{static_cast<unsigned>(outputs_.size() - 1)}
			, numOutputs{static_cast<double>(outputs_.size())}
			, outputs{outputs_}
		{
			static_assert(std::is_arithmetic<T>::value, "Type must be an arithmetic type!");
		}

		T Calculate(T val)
		{
			count++;
			enqueue(val);
			updateSetpoints(val);

			// Bootstrap phase ends when the sliding window fills up
			if (count == BOOTSTRAP_LENGTH)
				bootstrap = false;

			// Run at max until bootstrap phase finishes
			if (bootstrap)
				return outputs[maxOutputIdx];

			return calculate(val);
		}
	};

	typedef GenericPIDController<double> PIDController;
}
