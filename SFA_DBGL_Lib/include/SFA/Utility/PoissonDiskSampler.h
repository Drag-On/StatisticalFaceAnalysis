//////////////////////////////////////////////////////////////////////
/// Statistical Shape Analysis
///
/// Copyright (c) 2014 by Jan Moeller
///
/// This software is provided "as-is" and does not claim to be
/// complete or free of bugs in any way. It should work, but
/// it might also begin to hurt your kittens.
//////////////////////////////////////////////////////////////////////

#ifndef POISSONDISKSAMPLER_H_
#define POISSONDISKSAMPLER_H_

#include <vector>
#include <random>
#include <type_traits>
#include <functional>
#include <DBGL/System/Tree/KdTree.h>

namespace sfa
{
    /**
     * @brief This class samples points from a K-d Tree using the poisson-disk algorithm. The sampled
     * 	      points will be mostly evenly distributed with a minimum distance r between each of them.
     */
    class PoissonDiskSampler
    {
	public:
	    /**
	     * @brief Constructor
	     */
	    PoissonDiskSampler();
	    /**
	     * @brief Samples points from the list defined by \p begin and \p end such that every
	     * 	      sample has a minimum distance of \p r to each other.
	     * @param begin Start of the range of points to use. Supposed to be a iterator of dbgl::KdTree::Container
	     * @param end End of the range of points to use.
	     * @param r Minimum distance between each sampled point
	     * @param k Limit of samples to choose before rejection
	     * @return List of sampled points
	     */
	    template<class RandomAccessIterator> auto sample(RandomAccessIterator begin,
		    RandomAccessIterator end, double r,
		    unsigned int k = 30) -> std::vector<typename dbgl::KdTree<typename std::remove_reference<decltype((*begin).data)>::type, typename std::remove_reference<decltype((*begin).point)>::type>::Container>;
	private:
	    /**
	     * @brief Generates up to \p k points around \p point in a distance from \p r and 2 * \p r and
	     * 	      gets the nearest neighbors to those generated points in \p tree.
	     * @param point Point to generate points around
	     * @param r Minimum distance
	     * @param k Max amount of points
	     * @param tree Tree to get nearest neighbors from
	     * @param[out] out Found points will be appended here
	     */
	    template<class KdTree> void generatePointsAround(decltype(((typename KdTree::Container*)0)->point) point,
		    double r, unsigned int k, KdTree& tree, std::vector<typename KdTree::Container>& out);
	    /**
	     * @brief Copies all points in \p tree around \p pos that are within \p inner and \p outer
	     * @param tree Tree to get points from
	     * @param pos Position to get points around
	     * @param inner Inner distance
	     * @param outer Outer distance
	     * @param[out] out Found points will be appended here
	     */
	    template<class KdTree> void pointsInRange(KdTree& tree, decltype(tree.getAll()[0].point) pos, double inner,
		    double outer, std::vector<typename KdTree::Container>& out);

	    std::mt19937 m_random;
    };
}

#include "PoissonDiskSampler.imp"

#endif /* POISSONDISKSAMPLER_H_ */
