/****************************************************************************
 *
 *   Copyright (c) 2021-2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file WelfordMeanVector.hpp
 *
 * Welford's online algorithm for computing mean and variance.
 */

#pragma once

namespace math
{

template <typename Type, size_t N>
class WelfordMeanVector
{
public:
	// For a new value, compute the new count, new mean, the new M2.
	bool update(const matrix::Vector<Type, N> &new_value)
	{
		if (_count == 0) {
			reset();
			_count = 1;
			_mean = new_value;
			return false;

		} else if (_count == UINT16_MAX) {
			// count overflow
			// reset count, but maintain mean and variance
			_M2 = variance();
			_M2_accum.zero();

			_count = 1;

		} else {
			_count++;
		}

		// mean accumulates the mean of the entire dataset
		// delta can be very small compared to the mean, use algorithm to minimise numerical error
		const matrix::Vector<Type, N> delta{new_value - _mean};
		const matrix::Vector<Type, N> mean_change = delta / _count;
		_mean = kahanSummation(_mean, mean_change, _mean_accum);

		// M2 aggregates the squared distance from the mean
		// count aggregates the number of samples seen so far
		const matrix::Vector<Type, N> M2_change = delta.emult(new_value - _mean);
		_M2 = kahanSummation(_M2, M2_change, _M2_accum);

		// protect against floating point precision causing negative variances
		_M2 = matrix::max(_M2, {});

		if (!_mean.isAllFinite() || !_M2.isAllFinite()) {
			reset();
			return false;
		}

		return valid();
	}

	bool valid() const { return _count > 2; }
	auto count() const { return _count; }

	void reset()
	{
		_count = 0;
		_mean.zero();
		_M2.zero();

		_mean_accum.zero();
		_M2_accum.zero();
	}

	// Retrieve the mean, variance and sample variance
	matrix::Vector<Type, N> mean() const { return _mean; }
	matrix::Vector<Type, N> variance() const { return _M2 / _count; }

private:

	// Use Kahan summation algorithm to get the sum of "sum_previous" and "input".
	// This function relies on the caller to be responsible for keeping a copy of
	// "accumulator" and passing this value at the next iteration.
	// Ref: https://en.wikipedia.org/wiki/Kahan_summation_algorithm
	inline matrix::Vector<Type, N> kahanSummation(const matrix::Vector<Type, N> &sum_previous,
			const matrix::Vector<Type, N> &input, matrix::Vector<Type, N> &accumulator)
	{
		const matrix::Vector<Type, N> y = input - accumulator;
		const matrix::Vector<Type, N> t = sum_previous + y;
		accumulator = (t - sum_previous) - y;
		return t;
	}

	matrix::Vector<Type, N> _mean{};
	matrix::Vector<Type, N> _M2{};

	matrix::Vector<Type, N> _mean_accum{};  ///< kahan summation algorithm accumulator for mean
	matrix::Vector<Type, N> _M2_accum{};    ///< kahan summation algorithm accumulator for M2

	uint16_t _count{0};
};

} // namespace math
